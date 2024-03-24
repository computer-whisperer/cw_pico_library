//
// Created by christian on 3/1/24.
//

#ifndef DRV8874_HPP
#define DRV8874_HPP
#include <hardware/pwm.h>
#include <cstdint>

#include "cw_gpio.hpp"

class DRV8874 {
    uint32_t in1_gpio;
    uint32_t in2_gpio;
    CWGPIO* pmode_gpio;
    CWGPIO* nsleep_gpio;
    CWGPIO* nfault_gpio;

    uint pwm_slice_num;
    uint in1_gpio_channel;
    uint in2_gpio_channel;

    absolute_time_t out1_start_time = nil_time;
    absolute_time_t out2_start_time = nil_time;
    float out1_last_value = 0.0f;
    float out2_last_value = 0.0f;

    const uint pwm_wrap_value = 1000;

    pwm_config pwm_config_state;

public:
    DRV8874(uint32_t in1_gpio_in, uint32_t in2_gpio_in, CWGPIO* pmode_gpio_in, CWGPIO* nsleep_gpio_in, CWGPIO* nfault_gpio_in) :
        in1_gpio(in1_gpio_in),
        in2_gpio(in2_gpio_in),
        pmode_gpio(pmode_gpio_in),
        nsleep_gpio(nsleep_gpio_in),
        nfault_gpio(nfault_gpio_in)
    {

        nsleep_gpio->gpio_set_dir(GPIO_OUT);
        nfault_gpio->gpio_set_dir(GPIO_IN);

        nsleep_gpio->gpio_put(true);

        // Independent half-bridge mode
        pmode_gpio->gpio_set_dir(GPIO_IN);

        pwm_slice_num = pwm_gpio_to_slice_num(in1_gpio);
        assert(pwm_gpio_to_slice_num(in1_gpio) == pwm_gpio_to_slice_num(in2_gpio));

        in1_gpio_channel = pwm_gpio_to_channel(in1_gpio);
        in2_gpio_channel = pwm_gpio_to_channel(in2_gpio);

        pwm_config_state = pwm_get_default_config();
        pwm_config_set_wrap(&pwm_config_state, pwm_wrap_value);
        pwm_config_set_clkdiv_int(&pwm_config_state, 9);
        pwm_set_chan_level(pwm_slice_num, in1_gpio_channel, 0);
        pwm_set_chan_level(pwm_slice_num, in2_gpio_channel, 0);
        pwm_init(pwm_slice_num, &pwm_config_state, true);


        gpio_set_dir(in1_gpio, GPIO_OUT);
        gpio_set_function(in1_gpio, GPIO_FUNC_PWM);

        gpio_set_dir(in2_gpio, GPIO_OUT);
        gpio_set_function(in2_gpio, GPIO_FUNC_PWM);
    }

    void set_independent_out1 (float out) {
        pwm_set_chan_level(pwm_slice_num, in1_gpio_channel, static_cast<uint16_t>(out * static_cast<float>(pwm_wrap_value)));
        out1_last_value = out;
    }

    void set_independent_out2 (float out) {
        pwm_set_chan_level(pwm_slice_num, in2_gpio_channel, static_cast<uint16_t>(out * static_cast<float>(pwm_wrap_value)));
        out2_last_value = out;
    }

    void set_independent_out1_economized (bool logical, float high, float low, uint32_t time_us) {
        float out = 0.0f;
        if (logical)
        {
            if (out1_last_value == 0.0f) {
                out1_start_time = get_absolute_time();
            }
            if (absolute_time_diff_us(out1_start_time, get_absolute_time()) < time_us) {
                out = high;
            }
            else {
                out = low;
            }
        }

        set_independent_out1(out);
    }

    void set_independent_out2_economized (bool logical, float high, float low, uint32_t time_us) {
        float out = 0.0f;
        if (logical)
        {
            if (out2_last_value == 0.0f) {
                out2_start_time = get_absolute_time();
            }
            if (absolute_time_diff_us(out2_start_time, get_absolute_time()) < time_us) {
                out = high;
            }
            else {
                out = low;
            }
        }

        set_independent_out2(out);
    }

    void update() {

    }
};

#endif //DRV8874_HPP
