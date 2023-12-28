//
// Created by christian on 11/29/23.
//

#include "filtered_button.hpp"
#include "gpio_isr_mux.hpp"
#include "pico/time.h"

static void press_isr_redirect(uint gpio, uint32_t event_mask, void* attr)
{
  auto button = static_cast<FilteredButton*>(attr);
  button->on_press_isr();
}

static void release_isr_redirect(uint gpio, uint32_t event_mask, void* attr)
{
  auto button = static_cast<FilteredButton*>(attr);
  button->on_release_isr();
}

FilteredButton::FilteredButton(uint32_t gpio, bool inverted) : button_gpio(gpio), button_inverted(inverted) {
  gpio_init(gpio);
  gpio_set_dir(gpio, GPIO_IN);
  gpio_pull_up(gpio);
  if (button_inverted)
  {
    register_gpio_isr(button_gpio, GPIO_IRQ_EDGE_FALL, press_isr_redirect, (void*)this);
    register_gpio_isr(button_gpio, GPIO_IRQ_EDGE_RISE, release_isr_redirect, (void*)this);
  }
  else
  {
    register_gpio_isr(button_gpio, GPIO_IRQ_EDGE_RISE, press_isr_redirect, (void*)this);
    register_gpio_isr(button_gpio, GPIO_IRQ_EDGE_FALL, release_isr_redirect, (void*)this);
  }
}

void FilteredButton::on_press_isr() {
  if (absolute_time_diff_us(last_release_time, get_absolute_time()) > 100000)
  {
    did_activate_this_cycle = false;
    did_hold_activate_this_cycle = false;
  }
  last_press_time = get_absolute_time();
}



void FilteredButton::on_release_isr() {
  press_update();
  last_release_time = get_absolute_time();
}

void FilteredButton::update() {
  if (gpio_get(button_gpio) != button_inverted)
  {
    press_update();
  }
}

void FilteredButton::press_update() {
  if (!did_activate_this_cycle && absolute_time_diff_us(last_press_time, get_absolute_time()) > 20000)
  {
    activation_count++;
    did_activate_this_cycle = true;
  }
  if (!did_hold_activate_this_cycle && absolute_time_diff_us(last_press_time, get_absolute_time()) > 3000000)
  {
    hold_activation_count++;
    did_hold_activate_this_cycle = true;
  }
}
