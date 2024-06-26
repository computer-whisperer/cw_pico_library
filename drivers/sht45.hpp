//
// Created by christian on 12/29/23.
//

#ifndef SHT45_HPP
#define SHT45_HPP
#include <i2c_bus_manager.hpp>
#include <telemetry_manager.hpp>

class SHT45 final : public I2CPeripheralDriver {
  absolute_time_t last_fetch_timestamp = nil_time;
  TelemetryManager::Channel temp_channel;
  TelemetryManager::Channel humidity_channel;
public:
  explicit SHT45(I2CHostInterface* i2c_bus_in, const std::string& name_in = "sht45");
  void do_normal_sample();
  void update_device_presence() override;
  void update() override;
  void initialize_device() override;
};

#endif //SHT45_HPP
