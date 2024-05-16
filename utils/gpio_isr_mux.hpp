//
// Created by christian on 11/2/23.
//

#ifndef THERMO_SCOPE_GPIO_ISR_MUX_HPP
#define THERMO_SCOPE_GPIO_ISR_MUX_HPP
#include "stdint.h"
#include "hardware/gpio.h"

typedef void (*gpio_isr_mux_callback_t)(uint gpio, uint32_t event_mask, void* attr);

void register_gpio_isr(uint32_t gpio_pin, uint32_t event_mask, gpio_isr_mux_callback_t callback, void* attr);

#endif //THERMO_SCOPE_GPIO_ISR_MUX_HPP
