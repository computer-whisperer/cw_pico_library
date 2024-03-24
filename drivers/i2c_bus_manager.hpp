//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_I2C_BUS_MANAGER_HPP
#define THERMO_SCOPE_I2C_BUS_MANAGER_HPP

#include <vector>
#include "hardware/i2c.h"

class I2CPeripheralDriver;

class I2CBusManager {
  absolute_time_t last_device_scan = nil_time;
public:

  std::vector<I2CPeripheralDriver*> peripheral_drivers;
  void update();
};

class I2CHostInterface {
public:
  virtual ~I2CHostInterface() = default;

  virtual int write_blocking (uint8_t addr, const uint8_t *src, size_t len, bool nostop) {return -1;}
  virtual int read_blocking (uint8_t addr, uint8_t *dst, size_t len, bool nostop) {return -1;}

  virtual int write_timeout (uint8_t addr, const uint8_t *src, size_t len, bool nostop) {return -1;}
  virtual int read_timeout (uint8_t addr, uint8_t *dst, size_t len, bool nostop) {return -1;}

  virtual void register_driver (I2CPeripheralDriver *driver) = 0;
};

class I2CHostInterfacePicoHW final : public I2CHostInterface {
  i2c_inst_t* i2c_inst;
public:
  I2CBusManager manager;

  ~I2CHostInterfacePicoHW() override = default;
  I2CHostInterfacePicoHW(i2c_inst_t* i2c_in, uint32_t scl_pin, uint32_t sda_pin);

  int write_blocking (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override;
  int read_blocking (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override;

  int write_timeout (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override;
  int read_timeout (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override;

  void register_driver (I2CPeripheralDriver *driver) override {
    manager.peripheral_drivers.push_back(driver);
  };
};

class I2CPeripheralDriver {
protected:
  I2CHostInterface* i2c_bus;
  uint8_t i2c_addr;
public:
  virtual ~I2CPeripheralDriver() = default;

  bool is_present = false;
  I2CPeripheralDriver(I2CHostInterface* i2c_bus_in, uint8_t i2c_addr_in): i2c_bus(i2c_bus_in), i2c_addr(i2c_addr_in) {
    i2c_bus->register_driver(this);
  };
  virtual void initialize_device(){};
  virtual void update_device_presence(){};
  virtual void update(){};
};




#endif //THERMO_SCOPE_I2C_BUS_MANAGER_HPP
