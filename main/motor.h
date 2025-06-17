#ifndef _Motor_h
#define _Motor_h

#include "esp_timer.h"
#include "GyverPID.h"
#include "encoder.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>

#include <std_msgs/msg/float32.h>

class Motor {
private:
    mcpwm_cmpr_handle_t comparator_a = NULL, comparator_b = NULL;
    bool is_forward;

    GyverPID pid_;
    Encoder* enc_;

    float angle = 0;
    rcl_publisher_t* angle_publisher_;

    bool motor_calibrating = false;
public:
    Motor() {}

    Motor(const char* name, gpio_num_t a_pin, gpio_num_t b_pin, Encoder* enc, rcl_publisher_t* angle_publisher, int group_id = 0) {
        enc_ = enc;
        pid_ = GyverPID(name, 50);
        pid_.tuner_.setParameters(NORMAL, 0.5, 0.25, 200, 0.5, 500, 50);
        pid_.setLimits(0, 1);
        angle_publisher_ = angle_publisher;
        ESP_LOGW("MOTOR", "hello");
        mcpwm_timer_handle_t timer = NULL;
        mcpwm_timer_config_t timer_config = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = 1000,
        };
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));
        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t operator_config = {
            .group_id = 0, // operator must be in the same group to the timer
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

        mcpwm_gen_handle_t generator_a = NULL, generator_b = NULL;
        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = a_pin,
        };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator_a));
        generator_config.gen_gpio_num = b_pin;
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator_b));

        mcpwm_comparator_config_t comparator_config;
        comparator_config.intr_priority = 0;
        comparator_config.flags.update_cmp_on_tez = true;
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_a));
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator_b));

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, 0));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_a,
                                                                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_a,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a, MCPWM_GEN_ACTION_LOW)));
        
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator_b,
                                                                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator_b,
                                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b, MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

        if(pid_.calibration_mode){
            ESP_LOGW(name, "calibrating mode");
            set_speed(50);
            motor_calibrating = true;
        }
    }

    void set_speed(float rpm) {
        pid_.setpoint = abs(rpm);
        is_forward = rpm < 0;
    }

    float loop() {
        float theta;
        float current_rpm = enc_->get_rpm(theta);
        
        pid_.input = current_rpm;
        // ESP_LOGI("ZALUPA2","%f",pid_.input);
        int pwm_value = 1000*pid_.getResultNow();

        if(motor_calibrating != pid_.calibration_mode) {
            motor_calibrating = false;
            set_speed(0);
        }

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_a, is_forward ? pwm_value : 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_b, is_forward ? 0 : pwm_value));

        if(is_forward){
            angle += theta;
        }
        else{
            angle -= theta;
        }
        return theta;
    }

    void publish_angle() {
        std_msgs__msg__Float32 angle_msg;
        angle_msg.data = angle;
        rcl_publish(angle_publisher_, &angle_msg, NULL);
    }
};
#endif