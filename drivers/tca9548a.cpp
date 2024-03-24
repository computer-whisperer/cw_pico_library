//
// Created by christian on 3/1/24.
//

#include "tca9548a.hpp"
#include <pico/time.h>

void TCA9548A::initialize_device() {
  reset_gpio->gpio_put(false);
  sleep_us(10);
  reset_gpio->gpio_put(true);
  sleep_us(10);
  i2c_bus->read_timeout(i2c_addr, &current_config, 1, false);
}

void TCA9548A::update_device_presence() {
  uint8_t data = 0;
  auto ret = i2c_bus->read_timeout(i2c_addr, &data, 1, false);
  if (ret < 0) {
    is_present = false;
    return;
  }
  is_present = true;
}

void TCA9548A::update() {
}

void TCA9548A::select_channel(uint8_t channel) {
  if (!is_present) {
    return;
  }
  uint8_t new_config = 1 << channel;
  if (new_config == current_config) {
    return;
  }
  current_config = new_config;
  i2c_bus->write_timeout(i2c_addr, &current_config, 1, false);
}


