//
// Created by christian on 9/15/23.
//

#ifndef THERMO_SCOPE_I2C_BUS_MANAGER_HPP
#define THERMO_SCOPE_I2C_BUS_MANAGER_HPP

#include <vector>
#include "hardware/i2c.h"

class I2CHostInterface {
public:
    virtual ~I2CHostInterface() = default;

    virtual int write_blocking (uint8_t addr, const uint8_t *src, size_t len, bool nostop) {return -1;}
    virtual int read_blocking (uint8_t addr, uint8_t *dst, size_t len, bool nostop) {return -1;}

    virtual int write_timeout (uint8_t addr, const uint8_t *src, size_t len, bool nostop) {return -1;}
    virtual int read_timeout (uint8_t addr, uint8_t *dst, size_t len, bool nostop) {return -1;}
};

class I2CHostInterfacePicoHW final : public I2CHostInterface {
    i2c_inst_t* i2c_inst;
public:
    ~I2CHostInterfacePicoHW() override = default;
    I2CHostInterfacePicoHW(i2c_inst_t* i2c_in, uint32_t scl_pin, uint32_t sda_pin);

    int write_blocking (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override;
    int read_blocking (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override;

    int write_timeout (uint8_t addr, const uint8_t *src, size_t len, bool nostop) override;
    int read_timeout (uint8_t addr, uint8_t *dst, size_t len, bool nostop) override;
};

class I2CPeripheralDriver {
protected:
  I2CHostInterface* i2c_bus;
  uint8_t i2c_addr;
public:
  virtual ~I2CPeripheralDriver() = default;

  bool is_present = false;
  I2CPeripheralDriver(I2CHostInterface* i2c_bus_in, uint8_t i2c_addr_in): i2c_bus(i2c_bus_in), i2c_addr(i2c_addr_in){};
  virtual void initialize_device(){};
  virtual void update_device_presence(){};
  virtual void update(){};
};


class I2CBusManager {
  absolute_time_t last_device_scan = nil_time;
public:

  std::vector<I2CPeripheralDriver*> peripheral_drivers;
  void update();
};

#endif //THERMO_SCOPE_I2C_BUS_MANAGER_HPP
