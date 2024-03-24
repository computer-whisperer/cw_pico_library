//
// Created by christian on 3/1/24.
//

#ifndef TCA9548A_HPP
#define TCA9548A_HPP

#include "i2c_bus_manager.hpp"
#include "cw_gpio.hpp"

class TCA9548A final : public I2CPeripheralDriver {
  CWGPIO* reset_gpio;

  uint8_t current_config = 0;
public:

  class Port final : public I2CHostInterface {
    TCA9548A* mux;
    uint8_t channel;
  public:
    Port(TCA9548A* mux_in, uint8_t channel_in) : mux(mux_in), channel(channel_in) {}

    int write_blocking (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override {
      if (!mux->is_present) {
        return -1;
      }
      mux->select_channel(channel);
      return mux->i2c_bus->write_blocking(addr, src, len, nostop);
    }

    int read_blocking (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override {
      if (!mux->is_present) {
        return -1;
      }
      mux->select_channel(channel);
      return mux->i2c_bus->read_blocking(addr, dst, len, nostop);
    }

    int read_timeout (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override {
      if (!mux->is_present) {
        return -1;
      }
      mux->select_channel(channel);
      return mux->i2c_bus->read_timeout(addr, dst, len, nostop);
    }

    int write_timeout (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override {
      if (!mux->is_present) {
        return -1;
      }
      mux->select_channel(channel);
      return mux->i2c_bus->write_timeout(addr, src, len, nostop);
    }

    void register_driver (I2CPeripheralDriver *driver) override {
      mux->i2c_bus->register_driver(driver);
    };
  };

  std::vector<Port> ports{};

  TCA9548A(I2CHostInterface * i2c_bus_in, uint8_t i2c_addr_in, CWGPIO* reset_gpio_in) :
    I2CPeripheralDriver(i2c_bus_in, i2c_addr_in),
    reset_gpio(reset_gpio_in) {
    reset_gpio->gpio_set_dir(true);
    reset_gpio->gpio_put(true);
    for (uint8_t i = 0; i < 8; i++) {
      ports.push_back(Port(this, i));
    }
  }

  void initialize_device() override;
  void update_device_presence() override;
  void update() override;

  void select_channel(uint8_t channel);
};

#endif //TCA9548A_HPP
