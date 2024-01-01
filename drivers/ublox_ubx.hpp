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
  const uint32_t assistnow_online_port = 80;

  uint32_t assistnow_awaiting_content_bytes = 0;
  uint8_t * volatile latest_assistnow_data = nullptr;
  volatile uint32_t latest_assistnow_data_len = 0;
  volatile bool latest_assistnow_data_ready = false;

  struct tcp_pcb *assistnow_pcb = nullptr;
  bool assistnow_connected = false;
  bool assistnow_connecting = false;

  uint32_t last_time_accuracy_ns = -1;

  absolute_time_t last_status_poll = nil_time;
  uart_inst_t * uart_dev;
  uint32_t tx_gpio;
  uint32_t rx_gpio;
  uint32_t pps_gpio;
  absolute_time_t last_sleep_command_time = nil_time;
  absolute_time_t last_wake_command_time = nil_time;
  bool do_power_save = false;
  bool is_going_to_sleep = false;
  TelemetryManager::Channel valid_channel{"gps_valid"};
  TelemetryManager::Channel flags_channel{"gps_flags"};
  TelemetryManager::Channel fix_type_channel{"gps_fix_type"};
  TelemetryManager::Channel longitude_channel{"gps_longitude"};
  TelemetryManager::Channel latitude_channel{"gps_latitude"};
  TelemetryManager::Channel h_acc_channel{"gps_horizontal_accuracy", "m"};
  TelemetryManager::Channel v_acc_channel{"gps_vertical_accuracy", "m"};
  TelemetryManager::Channel altitude_channel{"gps_altitude", "m"};
  TelemetryManager::Channel fix_num_sv_channel{"gps_fix_num_sv"};
  TelemetryManager::Channel num_sv_channel{"gps_num_sv"};
  TelemetryManager::Channel t_acc_channel{"gps_t_acc", "s"};

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

  struct CFG_PM2_DATA {
    uint8_t version:8;
    uint8_t reserved_0:8;
    uint8_t reserved_1:8;
    uint8_t reserved_2:8;
    uint16_t reserved_3:13;
    uint8_t mode:2;
    bool doNotEnterOff:1;
    uint8_t reserved_4:3;
    bool updateEPH:1;
    bool updateRTC:1;
    bool waitTimeFix:1;
    uint8_t limitPeakCurr:2;
    uint8_t reserved_5:1;
    bool extintBackup:1;
    bool extintWake:1;
    bool extintSelect:1;
    uint8_t reserved_6:1;
  };
};

#endif //THERMO_SCOPE_UBLOX_G7_HPP
