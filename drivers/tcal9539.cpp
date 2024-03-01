//
// Created by christian on 2/26/24.
//
#include "tcal9539.hpp"
#include <hardware/gpio.h>

TCAL9539::TCAL9539(I2CHostInterface * i2c_bus_in, uint8_t i2c_addr_in, uint32_t int_gpio_in, uint32_t reset_gpio_in):
    I2CPeripheralDriver(i2c_bus_in, i2c_addr_in), int_gpio(int_gpio_in), reset_gpio(reset_gpio_in) {
  ::gpio_set_dir(reset_gpio, GPIO_OUT);
  ::gpio_put(reset_gpio, true);
};

void TCAL9539::register_write(const uint8_t addr, const uint8_t value) const {
  uint8_t buf[2];
  buf[0] = addr;
  buf[1] = value;
  i2c_bus->write_timeout(i2c_addr, buf, 2, false);
}

uint8_t TCAL9539::register_read(const uint8_t addr) const {
  i2c_bus->write_timeout(i2c_addr, &addr, 1, true);
  uint8_t data = 0;
  i2c_bus->read_timeout(i2c_addr, &data, 1, false);
  return data;
}

void TCAL9539::initialize_device() {
  ::gpio_put(reset_gpio, false);
  sleep_us(10);
  ::gpio_put(reset_gpio, true);
  sleep_us(10);
  push_configuration(true);
}

void TCAL9539::update_device_presence() {
  uint8_t addr = RegisterMap::input_port_0;
  auto ret = i2c_bus->write_timeout(i2c_addr, &addr, 1, true);
  if (ret < 0) {
    is_present = false;
    return;
  }
  uint8_t data = 0;
  ret = i2c_bus->read_timeout(i2c_addr, &data, 1, false);
  if (ret < 0) {
    is_present = false;
    return;
  }
  is_present = true;
}

void TCAL9539::push_configuration(bool force) {
  if (!is_present) return;

  if (current_configuration_version == staged_configuration_version && !force) {
    // No update required
    return;
  }

  if (staged_configuration.output_port_0 != current_configuration.output_port_0 || force) {
    register_write(RegisterMap::output_port_0, staged_configuration.output_port_0);
  }

  if (staged_configuration.output_port_1 != current_configuration.output_port_1 || force) {
    register_write(RegisterMap::output_port_1, staged_configuration.output_port_1);
  }

  if (staged_configuration.polarity_inversion_0 != current_configuration.polarity_inversion_0 || force) {
    register_write(RegisterMap::polarity_inversion_0, staged_configuration.polarity_inversion_0);
  }

  if (staged_configuration.polarity_inversion_1 != current_configuration.polarity_inversion_1 || force) {
    register_write(RegisterMap::polarity_inversion_1, staged_configuration.polarity_inversion_1);
  }

  if (staged_configuration.configuration_0 != current_configuration.configuration_0 || force) {
    register_write(RegisterMap::configuration_0, staged_configuration.configuration_0);
  }

  if (staged_configuration.configuration_1 != current_configuration.configuration_1 || force) {
    register_write(RegisterMap::configuration_1, staged_configuration.configuration_1);
  }

  if (staged_configuration.output_drive_strength_00 != current_configuration.output_drive_strength_00 || force) {
    register_write(RegisterMap::output_drive_strength_00, staged_configuration.output_drive_strength_00);
  }

  if (staged_configuration.output_drive_strength_01 != current_configuration.output_drive_strength_01 || force) {
    register_write(RegisterMap::output_drive_strength_01, staged_configuration.output_drive_strength_01);
  }

  if (staged_configuration.output_drive_strength_10 != current_configuration.output_drive_strength_10 || force) {
    register_write(RegisterMap::output_drive_strength_10, staged_configuration.output_drive_strength_10);
  }

  if (staged_configuration.output_drive_strength_11 != current_configuration.output_drive_strength_11 || force) {
    register_write(RegisterMap::output_drive_strength_11, staged_configuration.output_drive_strength_11);
  }

  if (staged_configuration.input_latch_0 != current_configuration.input_latch_0 || force) {
    register_write(RegisterMap::input_latch_0, staged_configuration.input_latch_0);
  }

  if (staged_configuration.input_latch_1 != current_configuration.input_latch_1 || force) {
    register_write(RegisterMap::input_latch_1, staged_configuration.input_latch_1);
  }

  if (staged_configuration.pullup_pulldown_enable_0 != current_configuration.pullup_pulldown_enable_0 || force) {
    register_write(RegisterMap::pullup_pulldown_enable_0, staged_configuration.pullup_pulldown_enable_0);
  }

  if (staged_configuration.pullup_pulldown_enable_1 != current_configuration.pullup_pulldown_enable_1 || force) {
    register_write(RegisterMap::pullup_pulldown_enable_1, staged_configuration.pullup_pulldown_enable_1);
  }

  if (staged_configuration.pullup_pulldown_selection_0 != current_configuration.pullup_pulldown_selection_0 || force) {
    register_write(RegisterMap::pullup_pulldown_selection_0, staged_configuration.pullup_pulldown_selection_0);
  }

  if (staged_configuration.pullup_pulldown_selection_1 != current_configuration.pullup_pulldown_selection_1 || force) {
    register_write(RegisterMap::pullup_pulldown_selection_1, staged_configuration.pullup_pulldown_selection_1);
  }

  if (staged_configuration.interrupt_mask_register_0 != current_configuration.interrupt_mask_register_0 || force) {
    register_write(RegisterMap::interrupt_mask_register_0, staged_configuration.interrupt_mask_register_0);
  }

  if (staged_configuration.interrupt_mask_register_1 != current_configuration.interrupt_mask_register_1 || force) {
    register_write(RegisterMap::interrupt_mask_register_1, staged_configuration.interrupt_mask_register_1);
  }

  if (staged_configuration.output_port_configuration_register != current_configuration.output_port_configuration_register || force) {
    register_write(RegisterMap::output_port_configuration_register, staged_configuration.output_port_configuration_register);
  }

  current_configuration = staged_configuration;
  current_configuration_version = staged_configuration_version;
}

void TCAL9539::poll_status() {
  if (!is_present) return;

  last_status.input_port_0 = register_read(RegisterMap::input_port_0);
  last_status.input_port_1 = register_read(RegisterMap::input_port_1);
  last_status.interrupt_status_register_0 = register_read(RegisterMap::interrupt_status_register_0);
  last_status.interrupt_status_register_1 = register_read(RegisterMap::interrupt_status_register_1);
}

void TCAL9539::update() {
  push_configuration();
  poll_status();
}

bool TCAL9539::gpio_get(uint32_t channel) const {
  if (channel < 8) {
    return (last_status.input_port_0 & (1 << channel)) != 0;
  }
  return (last_status.input_port_1 & (1 << (channel - 8))) != 0;
}

void TCAL9539::gpio_put(uint32_t channel, bool value) {
  if (channel < 8) {
    if (value) {
      staged_configuration.output_port_0 |= (1 << channel);
    } else {
      staged_configuration.output_port_0 &= ~(1 << channel);
    }
  }
  if (channel >= 8) {
    if (value) {
      staged_configuration.output_port_1 |= (1 << (channel - 8));
    } else {
      staged_configuration.output_port_1 &= ~(1 << (channel - 8));
    }
  }
  staged_configuration_version++;
}

void TCAL9539::gpio_set_dir(uint32_t channel, bool out) {
  if (channel < 8) {
    if (out) {
      staged_configuration.configuration_0 |= (1 << channel);
    } else {
      staged_configuration.configuration_0 &= ~(1 << channel);
    }
  }
  if (channel >= 8) {
    if (out) {
      staged_configuration.configuration_1 |= (1 << (channel - 8));
    } else {
      staged_configuration.configuration_1 &= ~(1 << (channel - 8));
    }
  }
}


