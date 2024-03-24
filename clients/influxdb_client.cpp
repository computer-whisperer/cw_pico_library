//
// Created by christian on 9/14/23.
//
#include <string>
#include <cmath>
#include "influxdb_client.hpp"

#include <secrets.hpp>
#include <utility>

#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/tcp.h"
#include "cyw43.h"
#include "uzlib.h"
#include "defl_static.h"
#include "hardware/dma.h"
#include "cyw43_shim.h"
#include "pico/cyw43_arch.h"

static void tcp_err_cb_redirect(void *arg, err_t tpcb)
{
  ((InfluxDBClient*)arg)->tcp_err_handler(tpcb);
}

static err_t tcp_recv_cb_redirect(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  return ((InfluxDBClient*)arg)->tcp_recv_handler(tpcb, p, err);
}

static err_t tcp_send_cb_redirect(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  return ((InfluxDBClient*)arg)->tcp_send_handler(tpcb, len);
}

static err_t connect_cb_redirect(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  return ((InfluxDBClient*)arg)->connect_handler(tpcb, err);
}

static void dns_resolved_cb_redirect(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
  ((InfluxDBClient*)callback_arg)->dns_resolved_cb(name, ipaddr);
}


InfluxDBClient::InfluxDBClient(
        std::string bucket_in,
        std::string influxdb_url_in,
        uint32_t influxdb_port_in,
        std::string influxdb_token_in) :
        influxdb_url(std::move(influxdb_url_in)),
        influxdb_token(std::move(influxdb_token_in)),
        influxdb_port(influxdb_port_in),
        bucket(std::move(bucket_in)){
  header_data = "POST /api/v2/write?org=Kalogon&bucket=" + bucket + " "
                "HTTP/1.1\r\n"
                "Connection: keep-alive\r\n"
                "Keep-Alive: timeout=60, max=0\r\n"
                "Content-Type: text/plain; charset=utf-8\r\n"
                "Host: " + influxdb_url + "\r\n"
                "Authorization: Token " + influxdb_token + "\r\n";
  if (do_compression)
  {
    header_data += "Content-Encoding: gzip\r\n";
  }
  //influxdb_pcb = tcp_new();
}



void InfluxDBClient::stage_working_buffer() {
  uint32_t data_to_stage_len;
  int dma_channel;

  if (!staged_frames.empty())
  {
    assert(staged_frames.back().start < sizeof(ready_for_tx_buffer));
  }

  // Start CRC process and compress now so that we know the data length
  if (do_compression) {
    // Queue DMA sniffer to CRC the data
    dma_channel = dma_claim_unused_channel(true);

    // 8 bit transfers. The read address increments after each transfer but
    // the write address remains unchanged pointing to the dummy destination.
    // No DREQ is selected, so the DMA transfers as fast as it can.
    dma_channel_config c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);

    // CRC32 specific sniff set-up
    channel_config_set_sniff_enable(&c, true);
    dma_sniffer_set_data_accumulator(~0);
    dma_sniffer_set_output_reverse_enabled(true);
    dma_sniffer_enable(dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, true);
    uint8_t dummy = 0;


    dma_channel_configure(
            dma_channel,          // Channel to be configured
            &c,            // The configuration we just created
            &dummy,     // The (unchanging) write address
            working_buffer,           // The initial read address
            working_buffer_pos,     // Total number of transfers inc. appended crc; each is 1 byte
            true           // Start immediately.
    );
    uzlib_comp_data.outlen = 0;
    uzlib_comp_data.noutbits = 0;
    uzlib_comp_data.outbits = 0;
    uzlib_comp_data.dict_size = 32768;
    uzlib_comp_data.hash_bits = uzlib_hash_bits;
    uzlib_comp_data.hash_table = uzlib_hashtable;
    memset(uzlib_hashtable, 0, sizeof(uzlib_hashtable));
    zlib_start_block(&uzlib_comp_data);
    uzlib_compress(&uzlib_comp_data, reinterpret_cast<const uint8_t *>(working_buffer), working_buffer_pos);
    zlib_finish_block(&uzlib_comp_data);
    if (!staged_frames.empty())
    {
      assert(staged_frames.back().start < sizeof(ready_for_tx_buffer));
    }

    data_to_stage_len = uzlib_comp_data.outlen + 18;


    compressor_bytes_in += working_buffer_pos;
    compressor_bytes_out += data_to_stage_len;
  }
  else
  {
    data_to_stage_len = working_buffer_pos;
  }

  assert (data_to_stage_len < sizeof(ready_for_tx_buffer));

  cyw43_arch_lwip_begin();
  // Allocate space in tx buffer
  uint32_t tx_buffer_insert_pos = 0;
  if (!staged_frames.empty())
  {
    if (staged_frames.back().start > sizeof(ready_for_tx_buffer) ||
       (staged_frames.back().start + staged_frames.back().length) > sizeof(ready_for_tx_buffer))
    {
      printf("Something very bad has happened!");
    }
    else
    {
      tx_buffer_insert_pos = staged_frames.back().start + staged_frames.back().length;
      if (tx_buffer_insert_pos+data_to_stage_len > sizeof(ready_for_tx_buffer))
      {
        tx_buffer_insert_pos = 0;
      }
    }
  }
  bool did_overwrite = false;

  // Delete overwritten frames
  while (!staged_frames.empty())
  {
    if ((staged_frames.front().start >= tx_buffer_insert_pos) &&
        (staged_frames.front().start < tx_buffer_insert_pos + data_to_stage_len))
    {
      staged_frames.erase(staged_frames.begin());
      did_overwrite = true;
    }
    else
    {
      break;
    }
  }

  assert (tx_buffer_insert_pos < sizeof(ready_for_tx_buffer));

  staged_frames.emplace_back();
  staged_frames.back().start = tx_buffer_insert_pos;
  staged_frames.back().length = data_to_stage_len;

  if (did_overwrite)
  {
    printf("Dropped influx tx frames!\r\n");
  }


  // Write data into tx buffer
  if (do_compression)
  {
    uint8_t* buffer_start = ready_for_tx_buffer + tx_buffer_insert_pos;
    uint32_t head = 0;
    // GZIP header
    buffer_start[head++] = 0x1f; // ID1
    buffer_start[head++] = 0x8b; // ID2
    buffer_start[head++] = 0x08; // CM
    buffer_start[head++] = 0x00; // FLG
    buffer_start[head++] = 0x00; // MTIME
    buffer_start[head++] = 0x00;
    buffer_start[head++] = 0x00;
    buffer_start[head++] = 0x00;
    buffer_start[head++] = 0x04; // XFL
    buffer_start[head++] = 0xFF; // OS
    memcpy(buffer_start + head, uzlib_comp_data.outbuf, uzlib_comp_data.outlen);
    head += uzlib_comp_data.outlen;
    // Wait for CRC DMA to finish
    dma_channel_wait_for_finish_blocking(dma_channel);
    uint32_t sniffed_crc = ~dma_sniffer_get_data_accumulator();
    dma_channel_unclaim(dma_channel);

    memcpy(buffer_start + head, &sniffed_crc, 4);
    head += 4;
    memcpy(buffer_start + head, &working_buffer_pos, 4);
    head += 4;
  }
  else
  {
    memcpy(ready_for_tx_buffer + tx_buffer_insert_pos, working_buffer, data_to_stage_len);
  }
  cyw43_arch_lwip_end();

  working_buffer_pos = 0;

  try_send_frame();
}



void InfluxDBClient::try_connect() {
  if (connected || connecting || influxdb_pcb)
  {
    return;
  }
  if (cyw43_shim_tcpip_link_status(CYW43_ITF_STA) != CYW43_LINK_UP)
  {
    return;
  }
  last_connect_attempt = get_absolute_time();
  ip_addr_t addr;
  cyw43_arch_lwip_begin();
  err_t ret = dns_gethostbyname_addrtype(influxdb_url.c_str(), &addr, dns_resolved_cb_redirect, this, LWIP_DNS_ADDRTYPE_IPV6_IPV4);
  cyw43_arch_lwip_end();
  connecting = true;
  if (ret == ERR_OK) {
    dns_resolved_cb(influxdb_url.c_str(), &addr);
  }
  else if (ret == ERR_INPROGRESS)
  {
    // Callback should be called once resolved or failed
  }
  else {
    connecting = false;
  }
}



void InfluxDBClient::dns_resolved_cb(const char *string, const ip_addr *pAddr) {
  if (pAddr == nullptr)
  {
    printf("InfluxDBClient: failed DNS resolution\r\n");
    connecting = false;
  }
  else
  {
    cyw43_arch_lwip_begin();
    influxdb_pcb = tcp_new();
    tcp_arg(influxdb_pcb, this);
    tcp_err(influxdb_pcb, tcp_err_cb_redirect);
    tcp_recv(influxdb_pcb, tcp_recv_cb_redirect);
    tcp_sent(influxdb_pcb, tcp_send_cb_redirect);
    auto ret = tcp_connect(influxdb_pcb, pAddr, influxdb_port, connect_cb_redirect);
    printf("Influxdb attempting to connect!\r\n");
    cyw43_arch_lwip_end();
    if (ret != ERR_OK)
    {
      printf("InfluxDBClient: failed to connect %d\r\n", ret);
      tcp_close(influxdb_pcb);
      influxdb_pcb = nullptr;
      connecting = false;
    }
  }
}

void InfluxDBClient::tcp_err_handler(signed char i) {
  printf("Influxdb error cb, %d\r\n", i);
  tcp_close(influxdb_pcb);
  connecting = false;
  influxdb_pcb = nullptr;
  connected = false;
}

err_t InfluxDBClient::tcp_recv_handler(tcp_pcb *pPcb, pbuf *pPbuf, signed char err) {
  if (pPbuf == NULL) {
    if (influxdb_pcb)
    {
      tcp_close(influxdb_pcb);
      connecting = false;
      connected = false;
      influxdb_pcb = nullptr;
    }
    printf("Disconnecting...\r\n");
  } else {
    /*
    for (uint32_t i = 0; i < pPbuf->len; i++)
    {
      putchar(((char*)pPbuf->payload)[i]);
    }
    printf("\r\n");*/
    tcp_recved(pPcb, pPbuf->len);
    is_waiting_on_response = false;
    pbuf_free(pPbuf);
    try_send_frame();
  }
  return 0;
}

err_t InfluxDBClient::tcp_send_handler(struct tcp_pcb *tpcb, u16_t len) {
  is_sending_frame = false;
  is_waiting_on_response = true;
  return ERR_OK;
}

err_t InfluxDBClient::connect_handler(tcp_pcb *pPcb, err_t err) {
  if (err)
  {
    printf("InfluxDB connection err: %d", err);
    connecting = false;
    connected = false;
    return err;
  }
  else
  {
    printf("InfluxDB connected!\r\n");
    connected = true;
    connecting = false;
    return ERR_OK;
  }
}

err_t tcp_write_ret = 0;

void InfluxDBClient::try_send_frame() {
  if (!connected)
  {
    return;
  }
  if (is_sending_frame || is_waiting_on_response)
  {
    return;
  }
  if (staged_frames.empty())
  {
    return;
  }
  check_integrity();
  auto frame = staged_frames.front();

  cyw43_arch_lwip_begin();

  tcp_write(influxdb_pcb, header_data.c_str(), header_data.length(), 0);
  bytes_sent += header_data.length();
  uint8_t remaining_header[200];
  uint32_t len = snprintf(reinterpret_cast<char *>(remaining_header), sizeof(remaining_header), "Content-Length: %lu\r\n\r\n", frame.length);
  tcp_write(influxdb_pcb, remaining_header, len, TCP_WRITE_FLAG_COPY);
  bytes_sent += len;
  check_integrity();
  tcp_write_ret = tcp_write(influxdb_pcb, ready_for_tx_buffer + frame.start, frame.length, TCP_WRITE_FLAG_COPY);
  bytes_sent += frame.length;
  if (tcp_write_ret!= ERR_OK)
  {
    printf("InfluxDB send error %d\r\n", tcp_write_ret);
  }
  is_sending_frame = true;
  tcp_output(influxdb_pcb);

  //printf("Sent %d bytes to influx\r\n", frame.length);
  check_integrity();
  staged_frames.erase(staged_frames.begin());
  check_integrity();
  cyw43_arch_lwip_end();
}

void InfluxDBClient::update() {
  bool ip_connected = cyw43_shim_tcpip_link_status(CYW43_ITF_STA) == CYW43_LINK_UP;
  if (!connected && !connecting && do_connect && ip_connected)
  {
    if (absolute_time_diff_us(last_connect_attempt, get_absolute_time()) > 1000000)
    {
      try_connect();
    }
  }
  if (false && absolute_time_diff_us(last_status_msg, get_absolute_time()) > 1000000)
  {
    last_status_msg = get_absolute_time();
    printf("InfluxDB ip connected: %d, influx connecting: %d, influx connected: %d\r\n", ip_connected, connecting, connected);
  }
}

void InfluxDBClient::disconnect() {
  do_connect = false;
  if (influxdb_pcb)
  {
    tcp_close(influxdb_pcb);
    connecting = false;
    influxdb_pcb = nullptr;
    connected = false;
  }
}

void InfluxDBClient::set_dut_name(std::string dut_name_in) {
  this->dut_name = dut_name_in;
}

void InfluxDBClient::check_integrity() {
  for (auto& staged_frame : staged_frames)
  {
    assert(staged_frame.start < sizeof(ready_for_tx_buffer));
    assert(staged_frame.start + staged_frame.length <= sizeof(ready_for_tx_buffer));
  }
}

void InfluxDBClient::write_and_escape(const char *input_text) {
  for (int i = 0; i < strlen(input_text); i++)
  {
    if (input_text[i] == ' ')
    {
      working_buffer[working_buffer_pos] = '\\';
      working_buffer_pos++;
    }
    working_buffer[working_buffer_pos] = input_text[i];
    working_buffer_pos++;
  }
}

void InfluxDBClient::start_measurement(const char *measurement, const char *tags, uint64_t timestamp_us) {
  if (current_measurement)
  {
    end_measurement();
  }
  write_and_escape(measurement);
  write_and_escape(tags);
  working_buffer[working_buffer_pos++] = ' ';
  current_measurement = measurement;
  current_timestamp_us = timestamp_us;
}

void InfluxDBClient::push_double(const char* measurement, const char * tags, const char* name, double value, uint64_t timestamp_us) {
  if (isnanf((float)value) || isinff((float)value))
  {
    return;
  }
  push_data_start(measurement, tags, name, timestamp_us);
  working_buffer_pos += snprintf(working_buffer+working_buffer_pos, sizeof(working_buffer) - working_buffer_pos,
                                 "=%.8e", value);
  push_data_end();
}

void InfluxDBClient::push_float(const char* measurement, const char * tags, const char* name, float value, uint64_t timestamp_us) {
  if (isnanf(value) || isinff(value))
  {
    return;
  }
  push_data_start(measurement, tags, name, timestamp_us);
  working_buffer_pos += snprintf(working_buffer+working_buffer_pos, sizeof(working_buffer) - working_buffer_pos,
                                 "=%.8e", value);
  push_data_end();
}


void InfluxDBClient::push_uint32(const char *measurement, const char *tags, const char *name, uint32_t value, uint64_t timestamp_us) {
  push_data_start(measurement, tags, name, timestamp_us);
  working_buffer_pos += snprintf(working_buffer+working_buffer_pos, sizeof(working_buffer) - working_buffer_pos,
                                 "=%ld", value);
  push_data_end();
}

void InfluxDBClient::push_string(const char* measurement, const char * tags, const char* name, const char* value, uint64_t timestamp_us) {
  if (!value)
  {
    return;
  }
  push_data_start(measurement, tags, name, timestamp_us);
  working_buffer_pos += snprintf(working_buffer+working_buffer_pos, sizeof(working_buffer) - working_buffer_pos,
                                 "=\"%s\"", value);
  push_data_end();
}

void InfluxDBClient::end_measurement() {
  working_buffer_pos += snprintf(working_buffer+working_buffer_pos, sizeof(working_buffer) - working_buffer_pos,
                                 " %lld\n", current_timestamp_us * 1000);
  assert(working_buffer_pos < sizeof(working_buffer));
  if (working_buffer_pos > (sizeof(working_buffer) - 300))
  {
    stage_working_buffer();
  }
  current_measurement = nullptr;
}

void InfluxDBClient::push_data_start(const char *measurement, const char *tags, const char *name, uint64_t timestamp_us) {
  if ((measurement != current_measurement) || (timestamp_us!= current_timestamp_us))
  {
    start_measurement(measurement, tags, timestamp_us);
  }
  if(working_buffer[working_buffer_pos-1] != ' ')
  {
    working_buffer[working_buffer_pos++] = ',';
  }
  write_and_escape(name);
}

void InfluxDBClient::push_data_end() {
  if (working_buffer_pos > (sizeof(working_buffer) - 300))
  {
    end_measurement();
  }
}

uint32_t InfluxDBClient::get_staged_frame_count() {
  return staged_frames.size();
}

float InfluxDBClient::get_compression_ratio() {
  return (float)compressor_bytes_in/(float)compressor_bytes_out;
}

void InfluxDBClient::connect() {
  do_connect = true;
}

