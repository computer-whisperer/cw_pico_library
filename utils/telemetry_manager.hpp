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
  double best_temperature_c = 0;
  double best_humidity_rh = 0;
  double best_pressure_kpa = 0;

  bool enable_influxdb = false;
  InfluxDBClient *influxdb_client = nullptr;
  std::string influxdb_measurement_heading;
  std::string influxdb_tags;

  struct DataPoint {
    const char* field_name;
    absolute_time_t timestamp;
    double value;
  };

  queue_t intercore_influxdb_data_queue = {};
public:
  static void set_best_temperature_c(double temperature_c);

  static void set_best_humidity_rh(double humidity_rh);

  static void set_best_pressure_kpa(double pressure_kpa);

  static double get_best_temperature_c();

  static double get_best_humidity_rh();

  static double get_best_pressure_kpa();

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
  void update_tags(std::string tags);

  void push_data_to_influxdb();
};

#endif //TELEMETRY_MANAGER_HPP
