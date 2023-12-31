//
// Created by christian on 9/17/23.
//

#ifndef THERMO_SCOPE_UBLOX_G7_HPP
#define THERMO_SCOPE_UBLOX_G7_HPP

#include "hardware/uart.h"
#include "ring_buffer.hpp"
#include <pico/sync.h>
#include <array>
#include <telemetry_manager.hpp>

class UBLOX_UBX {
  bool assistnow_online_enabled = false;
  std::string assistnow_online_token;
  absolute_time_t last_assistnow_online_download = nil_time;
  absolute_time_t last_assistnow_online_download_attempt = nil_time;

  const std::string assistnow_online_server = "online-live1.services.u-blox.com";
  const uint32_t assistnow_online_port = 443;
  std::string header_data;

  struct tcp_pcb *assistnow_pcb = nullptr;
  bool assistnow_connected = false;
  bool assistnow_connecting = false;

  absolute_time_t last_status_poll = nil_time;
  uart_inst_t * uart_dev;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t pps_gpio;
  absolute_time_t last_sleep_command_time = nil_time;
  absolute_time_t last_wake_command_time = nil_time;
  bool do_power_save = false;
  bool is_going_to_sleep = false;
  TelemetryManager::Channel longitude_channel{"gps_longitude"};
  TelemetryManager::Channel latitude_channel{"gps_latitude"};
  TelemetryManager::Channel h_acc_channel{"gps_horizontal_accuracy", "m"};

  uint64_t most_recent_timestamp_seen = 0;
public:
  UBLOX_UBX(uart_inst_t * uart_dev_in, uint32_t tx_gpio_in, uint32_t rx_gpio_in, uint32_t pps_gpio_in);

  void send_ubx(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len);
  void send_nmea(std::string body);

  void ubx_csum(uint8_t *data, uint16_t data_len, uint8_t* ck_a_out, uint8_t* ck_b_out);

  uint32_t discover_device_baudrate();

  void initialize_device();

  void update();

  void lwip_update();

  void handle_message(uint8_t* data, uint16_t data_len);
  void handle_ubx_message(uint8_t msg_class, uint8_t msg_id, uint8_t* payload, uint16_t payload_len);

  void start_sleep();
  void start_wake();

  void on_enter_dormant();
  void on_exit_dormant();

  void use_assistnow_online(std::string token);

  void start_assistnow_online_download();

  void dns_resolved_cb(const char *string, const ip_addr *pAddr);
  void tcp_err_handler(signed char i);
  err_t tcp_recv_handler(tcp_pcb *pPcb, pbuf *pPbuf, signed char err);
  err_t tcp_send_handler(struct tcp_pcb *tpcb, u16_t len);
  err_t connect_handler(tcp_pcb *pPcb, err_t i);
};

#endif //THERMO_SCOPE_UBLOX_G7_HPP
