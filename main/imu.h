#pragma once
#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include "driver/i2c.h"
#include "mpu6050.h"
#include <sensor_msgs/msg/imu.h>
#include <string>


static const float deg_to_rad_factor = M_PI / 180.0;

class ImuSensor {
private:
    mpu6050_handle_t mpu6050;
    const char* TAG = "IMU";
    char* frame_id_;
    rcl_publisher_t* imu_publisher_;
public:
    ImuSensor () {}

    ImuSensor(i2c_port_t port, char* frame_id, rcl_publisher_t* imu_publisher) {
        frame_id_ = frame_id;
        imu_publisher_ = imu_publisher;

        mpu6050 = mpu6050_create(port, MPU6050_I2C_ADDRESS);
        if (mpu6050 == NULL) ESP_LOGE(TAG, "MPU6050 create returned NULL");

        esp_err_t ret = mpu6050_config(mpu6050, ACCE_FS_4G, GYRO_FS_500DPS);
        ESP_ERROR_CHECK(ret);

        ret = mpu6050_wake_up(mpu6050);
        ESP_ERROR_CHECK(ret);

        uint8_t mpu6050_deviceid;
        ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
        ESP_ERROR_CHECK(ret);
        if (MPU6050_WHO_AM_I_VAL != mpu6050_deviceid) ESP_LOGE(TAG, "Who Am I register does not contain expected data");
    }

    // ~ImuSensor() {
    //     ESP_LOGE(TAG, "WHY I AM HERE");
    //     mpu6050_delete(mpu6050);
    // }
    
    void loop() {
        sensor_msgs__msg__Imu imu_data;

        mpu6050_acce_value_t acce;
        mpu6050_gyro_value_t gyro;
        esp_err_t ret = mpu6050_get_acce(mpu6050, &acce);
        ESP_ERROR_CHECK(ret);
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        ESP_ERROR_CHECK(ret);

        imu_data.linear_acceleration.x = acce.acce_x * 9.81;
        imu_data.linear_acceleration.y = acce.acce_y * 9.81;
        imu_data.linear_acceleration.z = acce.acce_z * 9.81;

        imu_data.angular_velocity.x =  gyro.gyro_x * deg_to_rad_factor;
        imu_data.angular_velocity.y =  gyro.gyro_y * deg_to_rad_factor;
        imu_data.angular_velocity.z =  gyro.gyro_z * deg_to_rad_factor;

        imu_data.header.frame_id.data = frame_id_;
        imu_data.header.frame_id.size = strlen(frame_id_);
        imu_data.header.frame_id.capacity = strlen(frame_id_);


        struct timeval tv_now;
        gettimeofday(&tv_now, NULL);
        imu_data.header.stamp.sec = tv_now.tv_sec;
        imu_data.header.stamp.nanosec = tv_now.tv_usec*1000;

        rcl_publish(imu_publisher_, &imu_data, NULL);
    }
};