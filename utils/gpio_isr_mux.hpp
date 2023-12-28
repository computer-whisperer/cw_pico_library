//
// Created by christian on 11/2/23.
//

#ifndef THERMO_SCOPE_GPIO_ISR_MUX_HPP
#define THERMO_SCOPE_GPIO_ISR_MUX_HPP
#include "stdint.h"
#include "hardware/gpio.h"

void register_gpio_isr(uint32_t gpio_pin, uint32_t event_mask, gpio_irq_callback_t callback);

#endif //THERMO_SCOPE_GPIO_ISR_MUX_HPP
