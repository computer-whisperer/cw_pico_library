//
// Created by christian on 9/14/23.
//

#ifndef THERMO_SCOPE_INFLUXDB_EXPORT_HPP
#define THERMO_SCOPE_INFLUXDB_EXPORT_HPP


#include <cstdint>
#include <array>
#include <string>
#include <vector>
#include "lwip/ip_addr.h"
#include "uzlib.h"
#include "lwip/pbuf.h"
#include "pico/time.h"

class InfluxDBClient {
  void write_and_escape(const char* input_text);
  struct uzlib_comp uzlib_comp_data = {};
  static constexpr uint32_t uzlib_hash_bits = 12;
  uzlib_hash_entry_t uzlib_hashtable[1<<uzlib_hash_bits];

  std::string influxdb_url = "";
  std::string influxdb_token = "";
  uint32_t influxdb_port = 80;
  std::string header_data;
  std::string dut_name = "dut";
  std::string bucket = "";

  struct tcp_pcb* influxdb_pcb = nullptr;
  const bool do_compression = true;
  uint8_t ready_for_tx_buffer[32000];
  struct frame_metadata {
    uint32_t start;
    uint32_t length;
  };
  std::vector<struct frame_metadata> staged_frames;

  uint32_t compressor_bytes_in = 0;
  uint32_t compressor_bytes_out = 0;
  char working_buffer[16000];
  uint32_t working_buffer_pos = 0;
  void stage_working_buffer();
  bool do_connect = true;
  absolute_time_t last_status_msg = nil_time;
  public:
  uint32_t bytes_sent = 0;
  bool connecting = false;
  bool connected = false;
  bool is_sending_frame = false;
  bool is_waiting_on_response = false;
  absolute_time_t last_connect_attempt = nil_time;

  const char * current_measurement;
  uint64_t current_timestamp_us = 0;
  InfluxDBClient(const char* bucket_in);
  void try_connect();
  void start_measurement(const char* measurement, const char * tags, uint64_t timestamp_us);
  void end_measurement();
  void push_data_start(const char* measurement, const char * tags, const char* name, uint64_t timestamp_us);
  void push_double(const char* measurement, const char * tags, const char* name, double value, uint64_t timestamp_us);
  void push_float(const char* measurement, const char * tags, const char* name, float value, uint64_t timestamp_us);
  void push_uint32(const char* measurement, const char * tags, const char* name, uint32_t value, uint64_t timestamp_us);
  void push_string(const char* measurement, const char * tags, const char* name, const char* value, uint64_t timestamp_us);
  void push_data_end();
  void dns_resolved_cb(const char *string, const ip_addr *pAddr);
  void tcp_err_handler(signed char i);
  err_t tcp_recv_handler(tcp_pcb *pPcb, pbuf *pPbuf, signed char err);
  err_t tcp_send_handler(struct tcp_pcb *tpcb, u16_t len);
  err_t connect_handler(tcp_pcb *pPcb, err_t i);

  void try_send_frame();
  void update();

  void disconnect();
  void connect();

  void set_dut_name(std::string dut_name_in);

  void check_integrity();

  uint32_t get_staged_frame_count();
  float get_compression_ratio();
};

#endif //THERMO_SCOPE_INFLUXDB_EXPORT_HPP
