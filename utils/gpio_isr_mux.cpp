//
// Created by christian on 11/2/23.
//

#include <vector>
#include "gpio_isr_mux.hpp"

struct gpio_callback_association {
  uint32_t gpio_pin;
  uint32_t event_mask;
  gpio_irq_callback_t callback;
};

std::vector<struct gpio_callback_association> registered_callbacks;

void main_gpio_isr(uint gpio_pin, uint32_t event_mask)
{
  for (auto& callback : registered_callbacks) {
    if ((gpio_pin == callback.gpio_pin) &&
        (event_mask & callback.event_mask)  &&
        callback.callback)
    {
      callback.callback(gpio_pin, event_mask);
      return;
    }
  }
}

void register_gpio_isr(uint32_t gpio_pin, uint32_t event_mask, gpio_irq_callback_t callback)
{
  registered_callbacks.push_back({gpio_pin, event_mask, callback});
  gpio_set_irq_enabled_with_callback(gpio_pin, event_mask, true, main_gpio_isr);
}