#ifndef _DiffDriver_h
#define _DiffDriver_h
#include "motor.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <cstring>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>

void euler_to_quaternion(geometry_msgs__msg__Quaternion& q, float roll, float pitch, float yaw) {
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
}

class DiffDriver {
private:
    Motor *left_, *right_;
    float L, r;
    float theta;
    float pose_x, pose_y;
    uint64_t lasttime;
    rcl_publisher_t* odom_publisher_;
public:
    DiffDriver(Motor* left, Motor* right, float wheel_radius, float wheel_distanse, rcl_publisher_t* odom_publisher) {
        pose_x = pose_y = 0;
        theta = 0;
        lasttime = 0;
        
        left_ = left;
        right_ = right;

        L = wheel_distanse;
        r = wheel_radius;
        odom_publisher_ = odom_publisher;
    }

    DiffDriver() {
    }

    void set_speed(float linear, float angular) {
        float v_r = linear + (angular*L)/2;
        float v_l = linear - (angular*L)/2;

        float theta_r = v_r / r;
        float theta_l = v_l / r;

        float target_rpm_r = theta_r / (2*M_PI) * 60;
        float target_rpm_l = theta_l / (2*M_PI) * 60;

        right_->set_speed(target_rpm_r);
        left_->set_speed(target_rpm_l);
    }

    void loop() {
        float travel_right = right_->loop() * r;
        float travel_left = left_->loop() * r;

        float deltaTravel = (travel_right + travel_left) / 2;
        float deltaTheta = (travel_right - travel_left) / L;
        uint64_t newTime = millis();
        float deltaTime = (float)(newTime - lasttime)/1000.0f;
        lasttime = newTime;

        float delta_x, delta_y;
        if(travel_right == travel_left) {
            delta_x = travel_left*cos(theta);
            delta_y = travel_left*sin(theta);
        }
        else{
            float radius = deltaTravel / deltaTheta;

            float iccX = pose_x - radius*sin(theta);
            float iccY = pose_y + radius*cos(theta);

            delta_x = cos(deltaTheta)*(pose_x - iccX) - sin(deltaTheta)*(pose_y - iccY) + iccX - pose_x;
            delta_y = sin(deltaTheta)*(pose_x - iccX) + cos(deltaTheta)*(pose_y - iccY) + iccY - pose_y;
        }

        pose_x += delta_x;
        pose_y += delta_y;
        theta += deltaTheta;
        if (theta >= 2 * M_PI){
            theta = theta - 2*M_PI;
        }

        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        nav_msgs__msg__Odometry odom;
        odom.header.frame_id.data = "odom";
        odom.child_frame_id.data = "base_link";
        odom.header.stamp.sec = tv_now.tv_sec;
        odom.header.stamp.nanosec = tv_now.tv_usec*1000;
        odom.pose.pose.position.x = pose_x;
        odom.pose.pose.position.y = pose_y;
        odom.pose.pose.position.z = 0;
        euler_to_quaternion(odom.pose.pose.orientation, 0, 0, theta);
        odom.twist.twist.linear.x = deltaTravel / deltaTime;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.x = 0;
        odom.twist.twist.angular.y = 0;
        odom.twist.twist.angular.z = deltaTheta / deltaTime;
        std::memset(odom.pose.covariance, 0, 36*sizeof(double));
        std::memset(odom.twist.covariance, 0, 36*sizeof(double));
        rcl_publish(odom_publisher_, &odom, NULL);

        right_->publish_angle();
        left_->publish_angle();
    }
};

#endif