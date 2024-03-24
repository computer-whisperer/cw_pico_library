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

  virtual void gpio_put(bool value) {}
  virtual bool gpio_get() {return false;}
  virtual void gpio_set_dir(bool out)  {}
};

class CWGPIOPicoHW final : public CWGPIO {
  uint32_t channel;

public:
  explicit CWGPIOPicoHW(const uint32_t channel_in) : channel(channel_in) {
    ::gpio_init(channel);
  };
  void gpio_put(const bool value) const {
    ::gpio_put(channel, value);
  }
  bool gpio_get() const {
    return ::gpio_get(channel);
  }
  void gpio_set_dir(const bool out) const {
    ::gpio_set_dir(channel, out);
  }
};

#endif //CW_GPIO_HPP
