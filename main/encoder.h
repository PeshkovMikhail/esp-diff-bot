#ifndef _Encoder_h
#define _Encoder_h

#include <cmath>
#include "driver/gpio.h"

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint16_t* steps_ptr = (uint16_t*) arg;
    *steps_ptr += 1;
}

class Encoder {
private:
    uint16_t steps;
    float encoder_step_;
    int encoder_pin_;
    double last_timestamp;
public:
    Encoder() {}

    Encoder(int encoder_pin, float encoder_step) {
        steps = 0;
        encoder_step_ = encoder_step;
        encoder_pin_ = encoder_pin;
        last_timestamp = 0;

        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_POSEDGE;
        //bit mask of the pins, use GPIO4/5 here
        io_conf.pin_bit_mask = (1 << (int)encoder_pin_);
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = (gpio_pullup_t)1;
        gpio_config(&io_conf);

        gpio_isr_handler_add((gpio_num_t)encoder_pin_, gpio_isr_handler, (void*) &steps);
    }

    float get_rpm(float& theta) {
        gpio_isr_handler_remove((gpio_num_t)encoder_pin_);

        double current_time = (double)esp_timer_get_time() * 1.0e-6;
        double delta_time = current_time - last_timestamp;
        last_timestamp = current_time;

        theta = steps * encoder_step_;
        float rpm = theta / delta_time / (2*M_PI) * 60;
        steps = 0;

        gpio_isr_handler_add((gpio_num_t)encoder_pin_, gpio_isr_handler, (void*) &steps);

        return rpm;
    }
};

#endif