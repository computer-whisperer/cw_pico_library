//
// Created by christian on 12/29/23.
//

#ifndef TSYS01_HPP
#define TSYS01_HPP

#include <i2c_bus_manager.hpp>
#include <telemetry_manager.hpp>

class TSYS01 final : public I2CPeripheralDriver {
  uint16_t k0 = 0;
  uint16_t k1 = 0;
  uint16_t k2 = 0;
  uint16_t k3 = 0;
  uint16_t k4 = 0;
  double k0_coeff = 0;
  double k1_coeff = 0;
  double k2_coeff = 0;
  double k3_coeff = 0;
  double k4_coeff = 0;
  absolute_time_t last_fetch_timestamp = nil_time;
  TelemetryManager::Channel temp_channel{"tsys01_temp", "c"};
public:
  explicit TSYS01(i2c_inst_t* i2c_bus_in, bool addr_select_in);
  void do_normal_sample();
  bool check_device_presence() override;
  void update() override;
  void initialize_device() override;

  static constexpr uint8_t get_i2c_address(bool addr_select){
    return addr_select? 0x76 : 0x77;
  }
};


#endif //TSYS01_HPP
