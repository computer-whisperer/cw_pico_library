//
// Created by christian on 2/29/24.
//

#ifndef CW_GPIO_HPP
#define CW_GPIO_HPP
#include <cstdint>
#include <hardware/gpio.h>

class CWGPIO {
public:
  virtual ~CWGPIO() = default;

  virtual void gpio_put(bool value) = 0;
  virtual bool gpio_get() = 0;
  virtual void gpio_set_dir(bool out) = 0;
};

class CWGPIOPicoHW final : public CWGPIO {
  uint32_t channel;

public:
  explicit CWGPIOPicoHW(const uint32_t channel_in) : channel(channel_in) {
    ::gpio_init(channel);
  };
  void gpio_put(const bool value) override {
    ::gpio_put(channel, value);
  }
  bool gpio_get() override {
    return ::gpio_get(channel);
  }
  void gpio_set_dir(const bool out) override {
    ::gpio_set_dir(channel, out);
  }
};

#endif //CW_GPIO_HPP
