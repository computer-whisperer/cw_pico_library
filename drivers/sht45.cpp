//
// Created by christian on 12/29/23.
//

#include "sht45.hpp"

SHT45::SHT45(i2c_inst_t* i2c_bus_in) :
        I2CPeripheralDriver(i2c_bus_in, 0x44)
{
}

void SHT45::do_normal_sample() {
  uint8_t rx_bytes[6];
  auto ret = i2c_read_timeout_per_char_us(i2c_bus, i2c_addr, rx_bytes, 6, false, 100);
  if (ret >= 0) {
    auto t_ticks = rx_bytes[0] * 256 + rx_bytes[1];
    auto checksum_t = rx_bytes[2];
    auto rh_ticks = rx_bytes[3] * 256 + rx_bytes[4];
    auto checksum_rh = rx_bytes[5];
    auto t_degC = -45.0f + 175.0f * static_cast<float>(t_ticks)/65535.0f;
    auto rh_pRH = -6.0f + 125.0f * static_cast<float>(rh_ticks)/65535.0f;
    if (rh_pRH > 100)
      rh_pRH = 100;
    if (rh_pRH < 0)
      rh_pRH = 0;
    temp_channel.new_data(t_degC);
    humidity_channel.new_data(rh_pRH);
  }

  // Queue next conversion
  uint8_t command = 0xFD;
  ret = i2c_write_timeout_per_char_us(i2c_bus, i2c_addr, &command, 1, false, 100);
  if (ret < 0) {
    printf("SHT45 Disconnected!\r\n");
    is_present = false;
  }
}

bool SHT45::check_device_presence(){
  if (is_present) {
    // Don't re-check if we think we are connected. It breaks this sensor.
    return is_present;
  }
  uint8_t command = 0x89;
  auto ret = i2c_write_timeout_per_char_us(i2c_bus, i2c_addr, &command, 1, false, 100);
  if (ret < PICO_OK) {
    is_present = false;
    return false;
  }
  sleep_us(10000);
  uint8_t data[6];
  ret = i2c_read_timeout_per_char_us(i2c_bus, i2c_addr, data, 6, false, 100);
  if (ret < PICO_OK) {
    is_present = false;
    return false;
  }
  is_present = data[0] != 0;
  return is_present;
}

void SHT45::update() {
  if (absolute_time_diff_us(last_fetch_timestamp, get_absolute_time()) > 10000)
  {
    do_normal_sample();
    last_fetch_timestamp = get_absolute_time();
  }
}

void SHT45::initialize_device() {
  printf("Initializing TSYS01\r\n");
}