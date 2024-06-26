//
// Created by christian on 12/29/23.
//

#include "tsys01.hpp"

float TSYS01::get_latest_temperature_c()
{
    return latest_temperature_c;
}

TSYS01::TSYS01(I2CHostInterface* i2c_bus_in, bool addr_select_in, const std::string& name_in) :
        I2CPeripheralDriver(i2c_bus_in, get_i2c_address(addr_select_in)),
        temp_channel(name_in + "_temp", "c")
{
}

void TSYS01::do_normal_sample() {
  // Read ADC
  uint8_t command = 0x00;
  auto ret = i2c_bus->write_timeout(i2c_addr, &command, 1, false);

  uint8_t rx_bytes[3];
  ret = i2c_bus->read_timeout(i2c_addr, rx_bytes, 3, false);
  if (rx_bytes[0] || rx_bytes[1] || rx_bytes[2]) {
    double raw_value = static_cast<double>(rx_bytes[2])/256 + static_cast<double>(rx_bytes[1]) + static_cast<double>(rx_bytes[0]*256);
    double raw_power = raw_value;
    double result = k0_coeff;
    result += k1_coeff * raw_power;
    raw_power *= raw_value;
    result += k2_coeff * raw_power;
    raw_power *= raw_value;
    result += k3_coeff * raw_power;
    raw_power *= raw_value;
    result += k4_coeff * raw_power;
    temp_channel.new_data(result);
    TelemetryManager::set_best_temperature_c(result);
    latest_temperature_c = result;
  }

  // Queue next conversion
  command = 0x48;
  ret = i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  if (ret < 0) {
    printf("TSYS01 Disconnected!\r\n");
    is_present = false;
  }
}

void TSYS01::update_device_presence(){
  if (is_present) {
    // Don't re-check if we think we are connected. It breaks this sensor.
    return;
  }
  // Send the reset command
  uint8_t command = 0x1E;
  auto ret = i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  is_present = (ret >= PICO_OK);
}

//! Byte swap unsigned short
static uint16_t swap_uint16( uint16_t val )
{
  return (val << 8) | (val >> 8 );
}

void TSYS01::initialize_device() {
  printf("Initializing TSYS01\r\n");
  sleep_ms(3);

  // Fetch calibration data
  uint8_t command = 0xA2;
  i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  i2c_bus->read_timeout(i2c_addr, reinterpret_cast<uint8_t *>(&k4), 2, false);

  command = 0xA4;
  i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  i2c_bus->read_timeout(i2c_addr, reinterpret_cast<uint8_t *>(&k3), 2, false);

  command = 0xA6;
  i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  i2c_bus->read_timeout(i2c_addr, reinterpret_cast<uint8_t *>(&k2), 2, false);

  command = 0xA8;
  i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  i2c_bus->read_timeout(i2c_addr, reinterpret_cast<uint8_t *>(&k1), 2, false);

  command = 0xAA;
  i2c_bus->write_timeout(i2c_addr, &command, 1, false);
  i2c_bus->read_timeout(i2c_addr, reinterpret_cast<uint8_t *>(&k0), 2, false);

  k0 = swap_uint16(k0);
  k1 = swap_uint16(k1);
  k2 = swap_uint16(k2);
  k3 = swap_uint16(k3);
  k4 = swap_uint16(k4);

  k0_coeff = -1.5E-2*static_cast<double>(k0);
  k1_coeff = 1E-6*static_cast<double>(k1);
  k2_coeff = -2E-11*static_cast<double>(k2);
  k3_coeff = 4E-16*static_cast<double>(k3);
  k4_coeff = -2E-21*static_cast<double>(k4);
}

void TSYS01::update() {
  if (absolute_time_diff_us(last_fetch_timestamp, get_absolute_time()) > 100000)
  {
    do_normal_sample();
    last_fetch_timestamp = get_absolute_time();
  }
}
