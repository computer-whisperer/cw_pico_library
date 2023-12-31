//
// Created by christian on 9/15/23.
//

#include "i2c_bus_manager.hpp"

#include <cstdio>

#include "hardware/gpio.h"

I2CBusManager::I2CBusManager(i2c_inst_t *i2c_bus_in, uint32_t scl_pin, uint32_t sda_pin): i2c_bus(i2c_bus_in) {
  // Check state of scl and sda
  gpio_set_function(sda_pin, GPIO_FUNC_SIO);
  gpio_set_function(scl_pin, GPIO_FUNC_SIO);
  gpio_set_dir(sda_pin, GPIO_IN);
  gpio_set_dir(scl_pin, GPIO_IN);
  sleep_us(100);
  bool sda_state = gpio_get(sda_pin);
  const bool scl_state = gpio_get(scl_pin);

  if (!scl_state) {
    // SCL blocked!
    printf("I2C SCL is forced low!\r\n");
    while(1){};
  }

  if (!sda_state) {
    // SDA blocked!
    printf("I2C SDA is forced low!\r\n");
    gpio_set_dir(scl_pin, GPIO_OUT);
    for (uint32_t i = 0; i < 10; i++) {
      gpio_put(scl_pin, true);
      sleep_us(10);
      gpio_put(scl_pin, false);
      sleep_us(10);
    }
    gpio_set_dir(scl_pin, GPIO_IN);
    sleep_us(100);
    // Bus should be clear now
    sda_state = gpio_get(sda_pin);
    if (!sda_state) {
      printf("I2C SDA is still forced low!\r\n");
      while(1){};
    }
  }

  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  //gpio_pull_up(scl_pin);
  //gpio_pull_up(sda_pin);
  i2c_init(i2c_bus_in, 400000);
  gpio_set_slew_rate(scl_pin, GPIO_SLEW_RATE_SLOW);
  gpio_set_slew_rate(sda_pin, GPIO_SLEW_RATE_SLOW);
}

void I2CBusManager::update() {

  if (absolute_time_diff_us(last_device_scan, get_absolute_time()) > 1000000)
  {
    for (auto driver : peripheral_drivers) {
      bool prev_connected = driver->is_present;
      bool new_connected = driver->check_device_presence();
      if (!prev_connected && new_connected) {
        driver->initialize_device();
      }
    }
    last_device_scan = get_absolute_time();
  }


  for (auto driver : peripheral_drivers) {
    if (driver->is_present) {
      driver->update();
    }
  }
}


