//
// Created by christian on 12/29/23.
//

#include "telemetry_manager.hpp"
#include "pico/unique_id.h"

#include <cassert>
#include <time_manager.hpp>
#include <utility>

static TelemetryManager* telemetry_manager_instance = nullptr;
static std::vector<TelemetryManager::Channel*> channels;

void TelemetryManager::set_best_temperature_c(const double temperature_c) {
  telemetry_manager_instance->best_temperature_c = temperature_c;
}

void TelemetryManager::set_best_humidity_rh(const double humidity_rh) {
  telemetry_manager_instance->best_humidity_rh = humidity_rh;
}

void TelemetryManager::set_best_pressure_kpa(const double pressure_kpa) {
  telemetry_manager_instance->best_pressure_kpa = pressure_kpa;
}

double TelemetryManager::get_best_temperature_c() {
  return telemetry_manager_instance->best_temperature_c;
}

double TelemetryManager::get_best_humidity_rh() {
  return telemetry_manager_instance->best_humidity_rh;
}

double TelemetryManager::get_best_pressure_kpa() {
  return telemetry_manager_instance->best_pressure_kpa;
}

TelemetryManager::Channel::Channel(std::string name_in, std::string  unit_in) : name(std::move(name_in)), unit(std::move(unit_in)) {
  channels.push_back(this);
  influx_field_name = name + "_" + unit;
}

TelemetryManager::Channel::Channel(std::string name_in) : name(std::move(name_in)) {
  channels.push_back(this);
  influx_field_name = name;
}

void TelemetryManager::Channel::new_data(const double value, const absolute_time_t timestamp) {
  if (telemetry_manager_instance->enable_influxdb) {
    const DataPoint new_point {
    .field_name = influx_field_name.c_str(),
    .timestamp = timestamp,
    .value = value
    };
    queue_add_blocking(&telemetry_manager_instance->intercore_influxdb_data_queue, &new_point);
  }
}

void TelemetryManager::Channel::new_data(double value) {
  new_data(value, get_absolute_time());
}

TelemetryManager::TelemetryManager() {
  telemetry_manager_instance = this;
}

void TelemetryManager::use_influxdb(InfluxDBClient* influxdb_client_in, const std::string& measurement_heading) {
  influxdb_client = influxdb_client_in;
  queue_init(&intercore_influxdb_data_queue, sizeof(DataPoint), 100);
  influxdb_measurement_heading = measurement_heading;
  enable_influxdb = true;
}

void TelemetryManager::update_tags(std::string tags) {
  pico_unique_board_id_t unique_board_id;
  pico_get_unique_board_id(&unique_board_id);
  const auto board_id = unique_board_id.id[6] + (unique_board_id.id[7]<<8);
  influxdb_tags = ",pico_id=" + std::to_string(board_id) + "" + tags;
}

void TelemetryManager::push_data_to_influxdb() {
  DataPoint new_point{};
  while (queue_try_remove(&intercore_influxdb_data_queue, &new_point))
  {
    if (TimeManager::has_epoch() != 0)
    {
      influxdb_client->push_double(influxdb_measurement_heading.c_str(), influxdb_tags.c_str(), new_point.field_name, new_point.value,
                                 TimeManager::convert_to_unix_time_us(new_point.timestamp));
    }
  }
}
