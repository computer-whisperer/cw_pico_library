//
// Created by christian on 12/29/23.
//

#ifndef TELEMETRY_MANAGER_HPP
#define TELEMETRY_MANAGER_HPP
#include "influxdb_client.hpp"
#include <string>
#include <vector>
#include <pico/util/queue.h>


class TelemetryManager
{
  bool enable_influxdb = false;
  InfluxDBClient *influxdb_client = nullptr;
  std::string influxdb_measurement_heading;

  struct DataPoint {
    const char* field_name;
    absolute_time_t timestamp;
    double value;
  };

  queue_t intercore_influxdb_data_queue = {};
public:
  class Channel
  {
    std::string influx_field_name;
    std::string name;
    std::string unit;
  public:
    Channel(std::string name_in, std::string unit_in);
    explicit Channel(std::string name_in);
    void new_data(double value, absolute_time_t timestamp);
    void new_data(double value);
  };
  TelemetryManager();
  void use_influxdb(InfluxDBClient *influxdb_client_in, const std::string& measurement_heading);

  void push_data_to_influxdb();
};

#endif //TELEMETRY_MANAGER_HPP
