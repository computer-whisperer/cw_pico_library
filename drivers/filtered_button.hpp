//
// Created by christian on 11/29/23.
//

#ifndef FILTERED_BUTTON_HPP
#define FILTERED_BUTTON_HPP

#include <cstdint>
#include "pico/types.h"
#include "pico/time.h"

class FilteredButton {
  uint32_t button_gpio;
  bool button_inverted;
  absolute_time_t last_press_time = nil_time;
  absolute_time_t last_release_time = nil_time;
  bool did_activate_this_cycle = false;
  bool did_hold_activate_this_cycle = false;
  void press_update();
  public:
  explicit FilteredButton(uint32_t gpio, bool inverted = true);
  void on_press_isr();
  void on_release_isr();
  void update();

  uint32_t activation_count = 0;
  uint32_t hold_activation_count = 0;
};

#endif //FILTERED_BUTTON_HPP
