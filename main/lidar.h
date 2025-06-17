#pragma once

#include "driver/uart.h"
#include "driver/gpio.h"
#include "xv_lib.h"
#include "driver/mcpwm_prelude.h"
#include <sensor_msgs/msg/laser_scan.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LIDAR_BUF_SIZE (360/4*22)

class Lidar : public XV{
private:
    int64_t last_lidar_timestamp = 0;
    int last_angle_deg = 0;
    mcpwm_cmpr_handle_t comparator_lidar = NULL;

    sensor_msgs__msg__LaserScan lidar_msg;
    rosidl_runtime_c__float__Sequence lidar_ranges, lidar_intensitives;
    uint8_t* data;
    rcl_publisher_t* publisher_;
    uart_port_t port_;
    const char* TAG = "LIDAR";
public:
    Lidar () {}

    Lidar(uart_port_t port, gpio_num_t rx_pin, rcl_publisher_t* publisher) : XV() {
        mcpwm_timer_handle_t timer_lidar = NULL;
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 1000,
            
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_lidar));

        mcpwm_oper_handle_t oper_lidar = NULL;
        mcpwm_operator_config_t operator_config = {
            .group_id = 0, // operator must be in the same group to the timer
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper_lidar));

        ESP_LOGI(TAG, "Connect timer and operator");
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_lidar, timer_lidar));

        ESP_LOGI(TAG, "Create comparator and generator from the operator");
        
        mcpwm_comparator_config_t comparator_config;
        comparator_config.intr_priority = 0;
        comparator_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper_lidar, &comparator_config, &comparator_lidar));

        mcpwm_gen_handle_t generator_lidar = NULL;
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = 32,
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper_lidar, &generator_config, &generator_lidar));
        
        
        // set the initial compare value, so that the servo will spin to the center position
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_lidar, 0));

        ESP_LOGI(TAG, "Set generator action on timer and compare event");
        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_lidar,
                                                                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_lidar,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_lidar, MCPWM_GEN_ACTION_LOW)));

        ESP_LOGI(TAG, "Enable and start timer");
        ESP_ERROR_CHECK(mcpwm_timer_enable(timer_lidar));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_lidar, MCPWM_TIMER_START_NO_STOP));

        lidar_ranges.data = (float*)malloc(360*sizeof(float));
        lidar_ranges.size = 360;
        lidar_ranges.capacity = 360;

        lidar_intensitives.data = (float*)malloc(360*sizeof(float));
        lidar_intensitives.size = 360;
        lidar_intensitives.capacity = 360;

        lidar_msg.header.frame_id.data = "lidar_link";
        lidar_msg.angle_increment = 2*M_PI/360;
        lidar_msg.scan_time = 0.02;
        lidar_msg.range_min = 0.15;
        lidar_msg.range_max = 12;

        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;

        ESP_ERROR_CHECK(uart_driver_install(port, LIDAR_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(port, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        
        enableMotor(true);

        data = (uint8_t *) malloc(LIDAR_BUF_SIZE);

        publisher_ = publisher;
        port_ = port;
    }

    void loop() {
        int len = uart_read_bytes(port_, data, (LIDAR_BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        for(int id = 0; id < len; id++) {
            processByte(data[id]);
        }
        
        if (!XV::loop()) {
            ESP_LOGI(TAG, "%f",scanRpmPID.input);
            ESP_LOGE(TAG, "motor error");
        }
    }

    void scan_callback(uint16_t angle_deg, uint16_t distance_mm,
        uint16_t quality, uint8_t err) override {
        // ESP_LOGI(TAG, "%d grad %d mm %d %d", angle_deg, distance_mm, quality, err);
            lidar_ranges.data[angle_deg] = (float)distance_mm/1000.0f;
            lidar_intensitives.data[angle_deg] = quality;
        
        if(last_angle_deg > angle_deg) {
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            lidar_msg.header.stamp.sec = tv_now.tv_sec;
            lidar_msg.header.stamp.nanosec = tv_now.tv_usec*1000;
            lidar_msg.angle_min = 0;
            lidar_msg.angle_max = (2 * M_PI);
            lidar_msg.ranges = lidar_ranges;
            lidar_msg.scan_time = ((float)esp_timer_get_time() - (float)last_lidar_timestamp) / 1000000;
            lidar_msg.time_increment = lidar_msg.scan_time / 360.0f;
            lidar_msg.intensities = lidar_intensitives;

            last_lidar_timestamp = esp_timer_get_time();
            RCSOFTCHECK(rcl_publish(publisher_, &lidar_msg, NULL));
        }
        last_angle_deg = angle_deg;
    }

    void motor_callback(float pwm) override {
        ESP_LOGI(TAG, "%f %f %d", pwm, scanRpmPID._minOut, motor_enabled);
        int pwm_value = 1000*pwm;
        
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_lidar, pwm_value));
    }
};