/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/mcpwm_prelude.h"
#include "sdkconfig.h"
#include "esp_log.h"


#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <string>


#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "esp_netif_sntp.h"
#include "esp_sntp.h"


#include <limits>
#include <cmath>

#include "GyverPID.h"
#include "diff_driver.h"
#include "lidar.h"
#include "imu.h"

#include "esp32_serial_transport.h"

#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}



#define ENCODER_STEP (2*M_PI / 30)

#define ECHO_TEST_TXD (UART_PIN_NO_CHANGE)
#define ECHO_TEST_RXD (5)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (UART_NUM_2)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (4096)

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define TAG ("UART TEST")

extern "C" {
    rcl_subscription_t cmd_vel_subscriber;
    rcl_publisher_t lidar_publisher, imu_publisher, odom_publisher, left_angle_publisher, right_angle_publisher;
    geometry_msgs__msg__Twist twist_msg;

    DiffDriver driver;
    Lidar lidar;
    ImuSensor imu;

    Encoder left_enc, right_enc;
    Motor left_motor, right_motor;

    void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
            lidar.loop();
            imu.loop();
        }
    }

    void pid_compute(rcl_timer_t * timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
            driver.loop();
        }
    }

    void update_velocity(const void * msgin) {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        driver.set_speed(linear, angular);
    }

    void i2c_bus_init(void)
    {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)21;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_io_num = (gpio_num_t)22;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

        esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
        ESP_ERROR_CHECK(ret);

        ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
        ESP_ERROR_CHECK(ret);
    }

    void setup_diff_drive() {
        gpio_install_isr_service(0);
        left_enc = Encoder((gpio_num_t)13, ENCODER_STEP);
        right_enc = Encoder((gpio_num_t)14, ENCODER_STEP);

        left_motor = Motor("left_wheel", (gpio_num_t)27, (gpio_num_t)26, &left_enc, &left_angle_publisher);
        right_motor = Motor("right_wheel", (gpio_num_t)33, (gpio_num_t)25, &right_enc, &right_angle_publisher);

        driver = DiffDriver(&left_motor, &right_motor, 0.036, 0.1115, &odom_publisher);
    }

    void micro_ros_task(void * arg)
    {
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

    // #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    //     rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    //     // Static Agent IP and port can be used instead of autodisvery.
    //     RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    //     //RCCHECK(rmw_uros_discover_agent(rmw_options));
    // #endif

        // create init_options
        RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

        
        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        config.start = false;                       // start SNTP service explicitly (after connecting)
        config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
        config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
        config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
        config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
        esp_netif_sntp_init(&config);
        esp_netif_sntp_start();
        time_t now = 0;
        struct tm timeinfo;
        int retry = 0;
        const int retry_count = 15;
        while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
            ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        }
        time(&now);
        localtime_r(&now, &timeinfo);

        i2c_bus_init();
        lidar = Lidar(UART_NUM_1, (gpio_num_t)5, &lidar_publisher);
        imu = ImuSensor(I2C_NUM_0, (char*)"imu_link", &imu_publisher);
        setup_diff_drive();


        // create node
        rcl_node_t node;
        RCCHECK(rclc_node_init_default(&node, "socks_bot_node", "", &support));

        // create publisher
        RCCHECK(rclc_publisher_init_default(
            &lidar_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
            "scan"));

        RCCHECK(rclc_publisher_init_default(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu"
        ));

        RCCHECK(rclc_publisher_init_default(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "odom"
        ));

        RCCHECK(rclc_publisher_init_default(
            &left_angle_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_angle"
        ));

        RCCHECK(rclc_publisher_init_default(
            &right_angle_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_angle"
        ));

        RCCHECK(rclc_subscription_init_default(
            &cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"));


        // create timer,
        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        const unsigned int timer_timeout = 100;
        RCCHECK(rclc_timer_init_default2(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback,
            true));

        rcl_timer_t pid_timer = rcl_get_zero_initialized_timer();
        RCCHECK(rclc_timer_init_default2(
            &pid_timer,
            &support,
            RCL_MS_TO_NS(50),
            pid_compute,
            true));
        // create executor
        

        rclc_executor_t executor;
        RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
        RCCHECK(rclc_executor_add_timer(&executor, &timer));
        RCCHECK(rclc_executor_add_timer(&executor, &pid_timer));
        RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &update_velocity, ON_NEW_DATA));
        // RCL_SET_ERROR

        

        // Configure a temporary buffer for the incoming data
        

        while(1){
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            vTaskDelay( 1 / portTICK_PERIOD_MS);
        }

        // free resources
        RCCHECK(rcl_publisher_fini(&lidar_publisher, &node));
        RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
        RCCHECK(rcl_node_fini(&node));

        vTaskDelete(NULL);
    }

    static size_t uart_port = UART_NUM_2;

    void app_main(void)
    {
    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
        rmw_uros_set_custom_transport(
            true,
            (void *) &uart_port,
            esp32_serial_open,
            esp32_serial_close,
            esp32_serial_write,
            esp32_serial_read
        );
    #else
    #error micro-ROS transports misconfigured
    #endif  // RMW_UXRCE_TRANSPORT_CUSTOM

        #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
            ESP_ERROR_CHECK(uros_network_interface_initialize());
        #endif
        xTaskCreate(&micro_ros_task, "uart_echo_task", CONFIG_MICRO_ROS_APP_STACK*3, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
    }
}