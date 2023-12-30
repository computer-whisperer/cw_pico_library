//
// Created by christian on 9/16/23.
//

#include <cstring>
#include "bmp581.hpp"

#include <cstdio>

BMP581::BMP581(i2c_inst_t* i2c_bus_in, bool addr_select_in) :
        I2CPeripheralDriver(i2c_bus_in, get_i2c_address(addr_select_in))
{
}

bool BMP581::check_device_presence(){
  auto value = this->field_read(FIELD_CHIP_ID);
  is_present = value == 0x50;
  return is_present;
}

void BMP581::initialize_device() {
  printf("Initializing BMP581\r\n");

  // Reset unit
  this->field_write(FIELD_CMD, 0xB6);
  sleep_ms(4);

  this->field_write(FIELD_PAD_IF_DRV, 0x01);

  this->field_write(FIELD_PRESS_EN, 0x01);
  this->set_fifo_mode(true, true);
  start_normal_mode(0x08, 0x03, 0x03);
}

//! Configure what measurements are stored in the bmp581 fifo registers
void BMP581::set_fifo_mode(bool temp_in_fifo_in, bool press_in_fifo_in) {
  this->temp_in_fifo = temp_in_fifo_in;
  this->press_in_fifo = press_in_fifo_in;
  // fifo_frame_sel is described in the datasheet. The 0x2 bit indicates pressure, and 0x1 bit indicates temperature.
  this->field_write(FIELD_FIFO_FRAME_SEL, (press_in_fifo_in << 1) | temp_in_fifo_in);
}

void BMP581::update() {
  if (absolute_time_diff_us(last_fetch_timestamp, get_absolute_time()) > 100000)
  {
    pull_from_fifo();
    last_fetch_timestamp = get_absolute_time();
  }
}

void BMP581::register_write(uint8_t addr, uint8_t value) {
  uint8_t buf[2];
  buf[0] = addr;
  buf[1] = value;
  //i2c_write_blocking_until(i2c_bus, i2c_addr, buf, 2, false, delayed_by_us(get_absolute_time(), 8000));
  i2c_write_timeout_per_char_us(i2c_bus, i2c_addr, buf, 2, false, 100);
}

uint8_t BMP581::register_read(uint8_t addr) {
  i2c_write_timeout_per_char_us(i2c_bus, i2c_addr, &addr, 1, true, 100);
  //i2c_write_blocking_until(i2c_bus, i2c_addr, &addr, 1, true, delayed_by_us(get_absolute_time(), 8000));
  uint8_t data = 0;
  i2c_read_timeout_per_char_us(i2c_bus, i2c_addr, &data, 1, false, 100);
  //i2c_read_blocking_until(i2c_bus, i2c_addr, &data, 1, false, delayed_by_us(get_absolute_time(), 8000));
  return data;
}

uint32_t BMP581::do_forced_measurement(float *temp_out, float *press_out) {
  this->field_write(FIELD_PWR_MODE, 0x02);
  for (uint8_t i = 0; i < 10; i++) {
    sleep_ms(10);
    if (this->field_read(FIELD_PWR_MODE) == 0x00) {
      // Now in standby mode again, time to read values
      *temp_out = process_temperature_data_C(
              this->field_read(FIELD_TEMP_MSB),
              this->field_read(FIELD_TEMP_LSB),
              this->field_read(FIELD_TEMP_XLSB));
      *press_out = process_pressure_data_kPa(
              this->field_read(FIELD_PRESS_MSB),
              this->field_read(FIELD_PRESS_LSB),
              this->field_read(FIELD_PRESS_XLSB));
      return 0;
    }
  }
  return 1;
}

/*!
 * Start the "normal" mode of the bmp581, where it will automatically sample on its own
 * @param odr_state Sample rate configuration, see datasheet for details
 */
void BMP581::start_normal_mode(int odr_state, int osr_p, int osr_t) {
  // These two cause the FIFO to automatically drop old entries and
  // hold the max number of frames (32)
  this->field_write(FIELD_FIFO_THRESHOLD, 0x00);
  this->field_write(FIELD_FIFO_MODE, 0x00);

  // Normal mode (periodic sample)
  this->field_write(FIELD_PWR_MODE, 0x01);

  // Oversampling configuration (disabled here)
  this->field_write(FIELD_OSR_P, osr_p);
  this->field_write(FIELD_OSR_T, osr_t);

  // Sample rate configuration, see datasheet for values
  this->field_write(FIELD_ODR, odr_state);

  if (!this->field_read(FIELD_ODR_IS_VALID)) {
    printf("Invalid ODR settings!\r\n");
    assert(false);
  }

  float datasheet_frequency = 0;
  switch (odr_state) {
    case 0x00:
      datasheet_frequency = 240.000;
      break;
    case 0x01:
      datasheet_frequency = 218.537;
      break;
    case 0x02:
      datasheet_frequency = 119.111;
      break;
    case 0x03:
      datasheet_frequency = 179.200;
      break;
    case 0x04:
      datasheet_frequency = 160.000;
      break;
    case 0x05:
      datasheet_frequency = 149.333;
      break;
    case 0x06:
      datasheet_frequency = 140.000;
      break;
    case 0x07:
      datasheet_frequency = 129.855;
      break;
    case 0x08:
      datasheet_frequency = 120.000;
      break;
    case 0x09:
      datasheet_frequency = 110.164;
      break;
    case 0x0A:
      datasheet_frequency = 100.299;
      break;
    case 0x0B:
      datasheet_frequency = 89.600;
      break;
    case 0x0C:
      datasheet_frequency = 80.000;
      break;
    case 0x0D:
      datasheet_frequency = 70.000;
      break;
    case 0x0E:
      datasheet_frequency = 60.000;
      break;
    case 0x0F:
      datasheet_frequency = 50.056;
      break;
    case 0x10:
      datasheet_frequency = 45.025;
      break;
    case 0x11:
      datasheet_frequency = 40.000;
      break;
    case 0x12:
      datasheet_frequency = 35.000;
      break;
    case 0x13:
      datasheet_frequency = 30.000;
      break;
    case 0x14:
      datasheet_frequency = 25.000;
      break;
    default:
    case 0x15:
      datasheet_frequency = 20.000;
      break;
  }

  sample_period_us = (uint32_t)(1000000.0f/datasheet_frequency);
}

void BMP581::pull_from_fifo() {
  uint8_t available_samples = this->field_read(FIELD_FIFO_COUNT);
  uint8_t bytes_per_sample = 0;
  if (this->temp_in_fifo) {
    bytes_per_sample += 3;
  }
  if (this->press_in_fifo) {
    bytes_per_sample += 3;
  }
  if ((bytes_per_sample > 0) && (available_samples > 0)) {
    // The fifo in the chip can store up to 32 24-bit values.
    uint8_t read_buffer[32 * 3];
    memset(read_buffer, 0, sizeof(read_buffer));
    uint8_t bytes_to_read = available_samples * bytes_per_sample;
    assert(bytes_to_read <= sizeof(read_buffer));
    uint8_t addr_value = BMP581::REG_FIFO_DATA;

    i2c_write_timeout_per_char_us(i2c_bus, i2c_addr, &addr_value, 1, true, 100);
    i2c_read_timeout_per_char_us(i2c_bus, i2c_addr, read_buffer, bytes_to_read, false, 100);

    uint64_t temp_data_timestamp = to_us_since_boot(get_absolute_time());
    uint64_t press_data_timestamp = to_us_since_boot(get_absolute_time());

    uint8_t *read_ptr = read_buffer;
    for (uint8_t i = 0; i < available_samples; i++) {
      if (read_ptr[0] == 0x7F) {
        // FIFO is empty or disabled
        // Somehow this state is actually reached sometimes. Nothing should break if this is reached, but it means that somehow
        // the bmp581 told us there were more samples available then are actually readable.
        break;
      }
      if (temp_in_fifo) {
        if (read_ptr[2] | read_ptr[1] | read_ptr[0]) {
          float value = process_temperature_data_C(read_ptr[2], read_ptr[1], read_ptr[0]);
          temp_channel.new_data(value, from_us_since_boot(temp_data_timestamp));
        }
        temp_data_timestamp -= sample_period_us;
        read_ptr += 3;
      }
      if (press_in_fifo) {
        if (read_ptr[2] | read_ptr[1] | read_ptr[0]) {
          float value = process_pressure_data_kPa(read_ptr[2], read_ptr[1], read_ptr[0]);
          press_channel.new_data(value, from_us_since_boot(temp_data_timestamp));
          TelemetryManager::set_best_pressure_kpa(value);
        }
        press_data_timestamp -= sample_period_us;
        read_ptr += 3;
      }
    }
  }
}
