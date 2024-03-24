//
// Created by christian on 2/26/24.
//

#ifndef TCAL9539_HPP
#define TCAL9539_HPP
#include "i2c_bus_manager.hpp"
#include "cw_gpio.hpp"
#include <cstdint>



class TCAL9539 : public I2CPeripheralDriver
{
public:
  uint32_t int_gpio;
  uint32_t reset_gpio;

  class RegisterMap {
  public:
    static constexpr uint8_t input_port_0 = 0x00;
    static constexpr uint8_t input_port_1 = 0x01;
    static constexpr uint8_t output_port_0 = 0x02;
    static constexpr uint8_t output_port_1 = 0x03;
    static constexpr uint8_t polarity_inversion_0 = 0x04;
    static constexpr uint8_t polarity_inversion_1 = 0x05;
    static constexpr uint8_t configuration_0 = 0x06;
    static constexpr uint8_t configuration_1 = 0x07;
    static constexpr uint8_t output_drive_strength_00 = 0x40;
    static constexpr uint8_t output_drive_strength_01 = 0x41;
    static constexpr uint8_t output_drive_strength_10 = 0x42;
    static constexpr uint8_t output_drive_strength_11 = 0x43;
    static constexpr uint8_t input_latch_0 = 0x44;
    static constexpr uint8_t input_latch_1 = 0x45;
    static constexpr uint8_t pullup_pulldown_enable_0 = 0x46;
    static constexpr uint8_t pullup_pulldown_enable_1 = 0x47;
    static constexpr uint8_t pullup_pulldown_selection_0 = 0x48;
    static constexpr uint8_t pullup_pulldown_selection_1 = 0x49;
    static constexpr uint8_t interrupt_mask_register_0 = 0x4A;
    static constexpr uint8_t interrupt_mask_register_1 = 0x4B;
    static constexpr uint8_t interrupt_status_register_0 = 0x4C;
    static constexpr uint8_t interrupt_status_register_1 = 0x4D;
    static constexpr uint8_t output_port_configuration_register = 0x4F;
  };

  class ConfigurationRegisters {
  public:
    uint8_t output_port_0 = 0x00;
    uint8_t output_port_1 = 0x00;
    uint8_t polarity_inversion_0 = 0x00;
    uint8_t polarity_inversion_1 = 0x00;
    uint8_t configuration_0 = 0xFF;
    uint8_t configuration_1 = 0xFF;
    uint8_t output_drive_strength_00 = 0x00;
    uint8_t output_drive_strength_01 = 0x00;
    uint8_t output_drive_strength_10 = 0x00;
    uint8_t output_drive_strength_11 = 0x00;
    uint8_t input_latch_0 = 0x00;
    uint8_t input_latch_1 = 0x00;
    uint8_t pullup_pulldown_enable_0 = 0x00;
    uint8_t pullup_pulldown_enable_1 = 0x00;
    uint8_t pullup_pulldown_selection_0 = 0x00;
    uint8_t pullup_pulldown_selection_1 = 0x00;
    uint8_t interrupt_mask_register_0 = 0x00;
    uint8_t interrupt_mask_register_1 = 0x00;
    uint8_t output_port_configuration_register = 0x00;
  };

  class StatusRegisters {
  public:
    uint8_t input_port_0;
    uint8_t input_port_1;
    uint8_t interrupt_status_register_0;
    uint8_t interrupt_status_register_1;
  };

  ConfigurationRegisters staged_configuration{};
  uint32_t staged_configuration_version = 0;
  ConfigurationRegisters current_configuration{};
  uint32_t current_configuration_version = 0;
  StatusRegisters last_status{};

  void push_configuration(bool force = false);
  void poll_status();

  void register_write(uint8_t addr, uint8_t value) const;
  uint8_t register_read(uint8_t addr) const;

  TCAL9539(I2CHostInterface * i2c_bus_in, uint8_t i2c_addr_in, uint32_t int_gpio_in, uint32_t reset_gpio_in);
  void initialize_device() override;
  void update_device_presence() override;
  void update() override;

  void gpio_put(uint32_t channel, bool value);
  bool gpio_get(uint32_t channel);
  void gpio_set_dir(uint32_t channel, bool out);

   absolute_time_t last_status_poll_time = nil_time;
};

class CWGPIOTCAL9539 : public CWGPIO {
  TCAL9539* tcal9539;
  uint32_t channel;
public:
  CWGPIOTCAL9539(TCAL9539* tcal9539_in, uint32_t channel_in) : tcal9539(tcal9539_in), channel(channel_in) {};

  void gpio_put(bool value) override {
    tcal9539->gpio_put(channel, value);
  }

  bool gpio_get() const {
    return tcal9539->gpio_get(channel);
  }

  void gpio_set_dir(bool out) override {
    tcal9539->gpio_set_dir(channel, out);
  }
};

#endif //TCAL9539_HPP
