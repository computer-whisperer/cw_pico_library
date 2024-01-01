//
// Created by christian on 9/17/23.
//

#include <cstring>
#include <string>
#include "ublox_ubx.hpp"

#include <cyw43.h>
#include <cyw43_shim.h>
#include <time_manager.hpp>
#include <utility>
#include <lwip/dns.h>
#include <lwip/tcp.h>
#include <pico/cyw43_arch.h>

#include "hardware/gpio.h"
#include "gpio_isr_mux.hpp"

static constexpr uint32_t max_msg_len = 100;


static RingBuffer<std::array<uint8_t, max_msg_len>> latest_full_messages{20};

volatile static char rx_buffer[max_msg_len];
volatile static uint32_t rx_buffer_index = 0;
volatile static uint32_t key_characters_found = 0;

static uart_inst_t * isr_uart_dev = nullptr;

static critical_section_t critical_section;


static absolute_time_t pps_timestamp = nil_time;

static void pps_gpio_isr(uint gpio, uint32_t event_mask)
{
  if (event_mask & GPIO_IRQ_EDGE_RISE) {
    pps_timestamp = get_absolute_time();
  }
}

static void clear_message_buffer()
{
  critical_section_enter_blocking(&critical_section);
  latest_full_messages.clear();
  rx_buffer_index = 0;
  key_characters_found = 0;
  critical_section_exit(&critical_section);
}

static void on_uart_rx()
{
  while (uart_is_readable(isr_uart_dev)) {
    auto val = rx_buffer[rx_buffer_index++] = uart_getc(isr_uart_dev);
    if (rx_buffer_index >= sizeof(rx_buffer)) {
      rx_buffer_index = 0;
    }
    else if ((val == '$' || val == 0xB5) && (rx_buffer_index > 1))
    {
      key_characters_found++;
      auto dest = latest_full_messages.push_and_return_ptr();
      for (uint32_t i = 0; i < rx_buffer_index-1; i++)
      {
        dest->data()[i] = rx_buffer[i];
      }
      rx_buffer[0] = rx_buffer[rx_buffer_index-1];
      rx_buffer_index = 1;
    }
  }
}

UBLOX_UBX::UBLOX_UBX(uart_inst_t *uart_dev_in, uint32_t tx_gpio_in, uint32_t rx_gpio_in, uint32_t pps_gpio_in):
        uart_dev(uart_dev_in),
        tx_gpio(tx_gpio_in),
        rx_gpio(rx_gpio_in),
        pps_gpio(pps_gpio_in)
{
  uart_init(uart_dev, 9600);

  gpio_set_function(tx_gpio, GPIO_FUNC_UART);
  gpio_set_function(rx_gpio, GPIO_FUNC_UART);

  gpio_init(pps_gpio);
  gpio_set_dir(pps_gpio, GPIO_IN);
  register_gpio_isr(pps_gpio, GPIO_IRQ_EDGE_RISE, pps_gpio_isr);

  uart_set_hw_flow(uart_dev, false, false);

  uart_set_fifo_enabled(uart_dev, false);

  int UART_IRQ = uart_dev == uart0 ? UART0_IRQ : UART1_IRQ;

  isr_uart_dev = uart_dev;
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);

  uart_set_irq_enables(uart_dev, true, false);

  critical_section_init(&critical_section);
}

#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_UPD 0x09
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_AID 0x0B
#define UBX_CLASS_TIM 0x0D
#define UBX_CLASS_ESF 0x10
#define UBX_CLASS_MGA 0x13
#define UBX_CLASS_LOG 0x21
#define UBX_CLASS_SEC 0x27

#define UBX_AID_ALM 0x30
#define UBX_AID_ALPSRV 0x32
#define UBX_AID_ALP 0x50
#define UBX_AID_AOP 0x33
#define UBX_AID_DATA 0x10
#define UBX_AID_EPH 0x31
#define UBX_AID_HUI 0x02
#define UBX_AID_INI 0x01
#define UBX_AID_REQ 0x00

#define UBX_CFG_ANT 0x13
#define UBX_CFG_CFG 0x09
#define UBX_CFG_DAT 0x06
#define UBX_CFG_GNSS 0x3E
#define UBX_CFG_INF 0x02
#define UBX_CFG_ITFM 0x39
#define UBX_CFG_LOGFILTER 0x47
#define UBX_CFG_MSG 0x01
#define UBX_CFG_NAV5 0x24
#define UBX_CFG_NAVX5 0x23
#define UBX_CFG_NMEA 0x17
#define UBX_CFG_PM2 0x3B
#define UBX_CFG_PRT 0x00
#define UBX_CFG_RATE 0x08
#define UBX_CFG_RST 0x04
#define UBX_CFG_RXM 0x11
#define UBX_CFG_SBAS 0x16
#define UBX_CFG_TP5 0x31
#define UBX_CFG_USB 0x1B

#define UBX_NAV_AOPSTATUS 0x60
#define UBX_NAV_CLOCK 0x22
#define UBX_NAV_DGPS 0x31
#define UBX_NAV_DOP 0x04
#define UBX_NAV_POSECEF 0x01
#define UBX_NAV_POSLLH 0x02
#define UBX_NAV_PVT 0x07
#define UBX_NAV_SBAS 0x32
#define UBX_NAV_STATUS 0x03
#define UBX_NAV_SVINFO 0x30
#define UBX_NAV_TIMEGPS 0x20
#define UBX_NAV_TIMEUTC 0x21
#define UBX_NAV_VELECEF 0x11
#define UBX_NAV_VELNED 0x12

#define UBX_TIM_TP 0x01

#define UBX_ACK_ACK 0x01
#define UBX_ACK_NAK 0x00




void UBLOX_UBX::initialize_device() {
  uint32_t current_rate = discover_device_baudrate();

  if (true || current_rate != 115200)
  {
    // Module must be newly configured
    // Set device baudrate to 115200
    uint8_t payload[44];
    memset(payload, 0, sizeof(payload));
    payload[0] = 1; // IO port 1
    *((uint16_t*)&payload[2]) = 0x0000; // txReady
    *((uint32_t*)&payload[4]) = 0x000008C0; // mode
    *((uint32_t*)&payload[8]) = 115200; // baudrate
    *((uint16_t*)&payload[12]) = 0x0001; // inProtoMask
    *((uint16_t*)&payload[14]) = 0x0001; // outProtoMask
    *((uint16_t*)&payload[16]) = 0x0000; // flags
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, payload, 20);

    // Wait for message to send and process
    sleep_ms(50);

    uart_set_baudrate(uart_dev, 115200);
    clear_message_buffer();

    // Configure navigation settings
    memset(payload, 0, sizeof(payload));
    *reinterpret_cast<uint16_t *>(&payload[0]) = 0x00F7; // mask
    *reinterpret_cast<uint8_t *>(&payload[2]) = 0; // dynModel
    *reinterpret_cast<uint8_t *>(&payload[3]) = 3; // fixMode
    *reinterpret_cast<int32_t *>(&payload[4]) = 0; // fixedAlt
    *reinterpret_cast<uint32_t *>(&payload[8]) = 0; // fixedAltVar
    *reinterpret_cast<int8_t *>(&payload[12]) = 5; // minElev
    *reinterpret_cast<uint8_t *>(&payload[13]) = 0; // drLimit
    *reinterpret_cast<uint16_t *>(&payload[14]) = 25; // pDop
    *reinterpret_cast<uint16_t *>(&payload[16]) = 25; // tDop
    *reinterpret_cast<uint16_t *>(&payload[18]) = 100; // pAcc
    *reinterpret_cast<uint16_t *>(&payload[20]) = 300; // tAcc
    *reinterpret_cast<uint8_t *>(&payload[22]) = 0; // staticHoldThresh
    *reinterpret_cast<uint8_t *>(&payload[23]) = 60; // dgpsTimeOut
    *reinterpret_cast<uint8_t *>(&payload[24]) = 0; // cnoThreshNumSVs
    *reinterpret_cast<uint8_t *>(&payload[25]) = 0; // cnoThresh
    send_ubx(UBX_CLASS_CFG, UBX_CFG_NAV5, payload, 36);

    // Configure TP messages
    memset(payload, 0, sizeof(payload));
    payload[0] = 0; // TP0
    *((uint16_t*)&payload[4]) = 50; // antCableDelay
    *((uint16_t*)&payload[6]) = 0; // rfGroupDelay
    *((uint32_t*)&payload[8]) = 1000000; // freqPeriod;
    *((uint32_t*)&payload[12]) = 1000000; // freqPeriodLock;
    *((uint32_t*)&payload[16]) = 100000; // pulseLenRatio;
    *((uint32_t*)&payload[20]) = 100000; // pulseLenRatioLock;
    *((int32_t*)&payload[24]) = 0; // userConfigDelay;
    *((uint32_t*)&payload[28]) = 0x00000077; // flags;
    send_ubx(UBX_CLASS_CFG, UBX_CFG_TP5, payload, 32);

    // Send sleep config/command
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x01; // Version
    *((uint32_t*)&payload[4]) = 0x00031C00; // PSM config flags (cyclic mode, doNotEnterOff, updateRTC, updateEPH, waitTimeFix)
    *((uint32_t*)&payload[8]) = 1000; // Update period
    *((uint32_t*)&payload[12]) = 10000; // Search period
    *((uint32_t*)&payload[16]) = 0; // gridOffset
    *((uint16_t*)&payload[20]) = 200; // onTime
    *((uint16_t*)&payload[22]) = 60; // minAcqTime
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PM2, payload, 44);

    // Enable TIM-TP
    memset(payload, 0, sizeof(payload));
    payload[0] = UBX_CLASS_TIM;
    payload[1] = UBX_TIM_TP;
    payload[3] = 1; // Set rate to 1 on port 1
    send_ubx(UBX_CLASS_CFG, UBX_CFG_MSG, payload, 8);

    // Enable PVT
    memset(payload, 0, sizeof(payload));
    payload[0] = UBX_CLASS_NAV;
    payload[1] = UBX_NAV_PVT;
    payload[3] = 1; // Set rate to 1 on port 1
    send_ubx(UBX_CLASS_CFG, UBX_CFG_MSG, payload, 8);

    // Enter full power mode
    memset(payload, 0, sizeof(payload));
    payload[0] = 0x08; // Always set for some reason
    payload[1] = 0x00; // Continuous mode
    send_ubx(UBX_CLASS_CFG, UBX_CFG_RXM, payload, 2);

    // Save settings
    //memset(payload, 0, sizeof(payload));
    //*((uint32_t*)&payload[4]) = 0x0000061F;
    //*((uint32_t*)&payload[4]) = 0xFFFFFFFF;
    //send_ubx(UBX_CLASS_CFG, UBX_CFG_CFG, payload, 12);
  }



}

void UBLOX_UBX::send_ubx(uint8_t msg_class, uint8_t msg_id, uint8_t *payload, uint16_t payload_len)
{
  uint32_t total_len = 8+payload_len;
  uint8_t buf[total_len];
  buf[0] = 0xB5;
  buf[1] = 0x62;
  buf[2] = msg_class;
  buf[3] = msg_id;
  buf[4] = payload_len&0xFF;
  buf[5] = payload_len>>8;
  memcpy(&buf[6], payload, payload_len);
  ubx_csum(buf+2, payload_len+4, buf + payload_len + 6, buf + payload_len + 7);
  uart_write_blocking(uart_dev, buf, total_len);
}

void UBLOX_UBX::ubx_csum(uint8_t *data, uint16_t data_len, uint8_t* ck_a_out, uint8_t* ck_b_out) {
  *ck_a_out = 0;
  *ck_b_out = 0;
  for (uint16_t i = 0; i < data_len; i++) {
    *ck_a_out += data[i];
    *ck_b_out += *ck_a_out;
  }
}
void UBLOX_UBX::handle_message(uint8_t *data, uint16_t data_len) {
  if (data_len < 4)
  {
    return;
  }
  if (data[0] == 0xB5)
  {
    // Validate UBX message
    if ((data[1] != 0x62) || (data_len < 8))
    {
      return;
    }
    uint16_t payload_len = data[4] | (((uint16_t)data[5])<<8);
    if (payload_len+4 > data_len)
    {
      return; // Invalid message
    }
    uint8_t calculated_ck_a, calculated_ck_b;
    ubx_csum(data+2, payload_len+4, &calculated_ck_a, &calculated_ck_b);
    if ((data[payload_len+6] != calculated_ck_a) || (data[payload_len+7]!= calculated_ck_b))
    {
      return;
    }
    handle_ubx_message(data[2], data[3], data+6, payload_len);
  }
  if (data[0] == '$')
  {
    // NMEA message
    // Not implemented ATM
  }
}

void UBLOX_UBX::handle_ubx_message(uint8_t msg_class, uint8_t msg_id, uint8_t *payload, uint16_t payload_len) {
  bool handled = false;
  // Align payload
  uint8_t aligned_buf[payload_len];
  memcpy(aligned_buf, payload, payload_len);
  switch (msg_class) {
    case UBX_CLASS_ACK:
      switch (msg_id) {
        case UBX_ACK_ACK:
          handled = true;
          break;
        case UBX_ACK_NAK: {
          uint32_t clsid = *reinterpret_cast<uint32_t *>(aligned_buf + 0);
          uint32_t msgid = *reinterpret_cast<uint32_t *>(aligned_buf + 4);
          printf("UBX NAK! 0x%x, 0x%x\r\n", clsid, msgid);
          handled = true;
          break;
        }
        default:
          break;
      }
      break;
    case UBX_CLASS_TIM:
      if (msg_id == UBX_TIM_TP)
      {
        uint32_t towMS = *(uint32_t*)(aligned_buf + 0);
        uint32_t towSubMS = *(uint32_t*)(aligned_buf + 4);
        int32_t qErr = *(int32_t*)(aligned_buf + 8);
        uint16_t week = *(uint16_t*)(aligned_buf + 12);
        uint8_t flags = *(uint8_t*)(aligned_buf + 14);

        uint64_t timestamp_us = 315964800000000 + week*604800000000 + towMS*(uint64_t)1000;
        most_recent_timestamp_seen = timestamp_us;
        handled = true;
      }
      break;
    case UBX_CLASS_AID:
      switch (msg_id) {
        case UBX_AID_HUI: {
          if (payload_len < 72) {
            break;
          }
          printf("SVs: ");
          uint32_t sv_values = 0;
          memcpy(&sv_values, &aligned_buf[0], 4);
          for (int8_t i = 31; i >= 0; i--) {
            printf("%d", (sv_values >> i)&1);
          }
          printf("\r\n");
          handled = true;
          break;
        }
        case UBX_AID_EPH: {
          if (payload_len < 8) {
            break;
          }
          auto svid = *(uint32_t*)(aligned_buf + 0);
          auto how = *(uint32_t*)(aligned_buf + 4);
          handled = true;
          if (payload_len < 104) {
            // No ephemeris data
            break;
          }
          printf("Have ephemeris data for %d (%d)\r\n", svid, how);
          break;
        }
        case UBX_AID_ALM: {
          if (payload_len < 8) {
            break;
          }
          auto svid = *(uint32_t*)(aligned_buf + 0);
          auto week = *(uint32_t*)(aligned_buf + 4);
          handled = true;
          if (payload_len < 40) {
            // No ephemeris data
            break;
          }
          printf("Have almanac data for %d (%d)\r\n", svid, week);
          break;
        }
        default:
          break;
      }
      break;
    case UBX_CLASS_NAV:
      switch (msg_id) {
        case UBX_NAV_SVINFO: {
          if (payload_len < 8) {
            break;
          }
          auto iTOW = *(uint32_t*)(aligned_buf);
          auto numCh = *(uint8_t*)(aligned_buf + 4);
          auto globalFlags = *(uint8_t*)(aligned_buf + 5);
          printf("Have %d sv.\r\n", numCh);
          num_sv_channel.new_data(numCh);
          if (payload_len < 8 + numCh*12) {
            break;
          }
          for (uint32_t i = 0; i < numCh; i++) {
            uint8_t* channel_start = aligned_buf + 8 + i*12;
            auto chn = *(uint8_t*)(channel_start + 0);
            auto svid = *(uint8_t*)(channel_start + 1);
            auto flags = *(uint8_t*)(channel_start + 2);
            auto quality = *(uint8_t*)(channel_start + 3);
            auto cno = *(uint8_t*)(channel_start + 4);
            auto elev = *reinterpret_cast<int8_t *>(channel_start + 5);
            auto azim = *reinterpret_cast<int16_t *>(channel_start + 6);
            auto prRes = *reinterpret_cast<int32_t *>(channel_start + 8);
            printf("UBX SV: id:%d, flags:0x%x, qual:0x%x, cno:%d\r\n", svid, flags, quality, cno);
          }
          handled = true;
          break;
        }
        case UBX_NAV_POSLLH: {
          if (payload_len < 28) {
            break;
          }
          const auto iTOW = *reinterpret_cast<uint32_t *>(aligned_buf);
          const auto lon = *reinterpret_cast<int32_t *>(aligned_buf + 4);
          const auto lat = *reinterpret_cast<int32_t *>(aligned_buf + 8);
          const auto height = *reinterpret_cast<int32_t *>(aligned_buf + 12);
          const auto hMSL = *reinterpret_cast<int32_t *>(aligned_buf + 16);
          const auto hAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 20);
          const auto vAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 20);
          const float f_lat = static_cast<float>(lat) / 10000000.0f;
          const float f_lon = static_cast<float>(lon) / 10000000.0f;
          const float h_acc_m = static_cast<float>(hAcc)/1000;
          const float v_acc_m = static_cast<float>(vAcc)/1000;
          printf("UBX POSLLH: Lat: %f, Lon: %f, Height: %d, VAcc: %f, HAcc: %f\r\n", f_lat, f_lon, height, h_acc_m, v_acc_m);

          handled = true;
          break;
        }
        case UBX_NAV_PVT: {
          if (payload_len < 84) {
            break;
          }
          const auto iTOW = *reinterpret_cast<uint32_t *>(aligned_buf);
          const auto year = *reinterpret_cast<uint16_t *>(aligned_buf + 4);
          const auto month = *reinterpret_cast<uint8_t *>(aligned_buf + 6);
          const auto day = *reinterpret_cast<uint8_t *>(aligned_buf + 7);
          const auto hour = *reinterpret_cast<uint8_t *>(aligned_buf + 8);
          const auto minute = *reinterpret_cast<uint8_t *>(aligned_buf + 9);
          const auto second = *reinterpret_cast<uint8_t *>(aligned_buf + 10);
          const auto valid = *reinterpret_cast<uint8_t *>(aligned_buf + 11);
          const auto tAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 12);
          const auto nano = *reinterpret_cast<uint32_t *>(aligned_buf + 16);
          const auto fixType = *reinterpret_cast<uint8_t *>(aligned_buf + 20);
          const auto flags = *reinterpret_cast<uint8_t *>(aligned_buf + 21);
          const auto numSV = *reinterpret_cast<uint8_t *>(aligned_buf + 23);
          const auto lon = *reinterpret_cast<int32_t *>(aligned_buf + 24);
          const auto lat = *reinterpret_cast<int32_t *>(aligned_buf + 28);
          const auto height = *reinterpret_cast<int32_t *>(aligned_buf + 32);
          const auto hMSL = *reinterpret_cast<int32_t *>(aligned_buf + 36);
          const auto hAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 40);
          const auto vAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 44);
          const auto velN = *reinterpret_cast<int32_t *>(aligned_buf + 48);
          const auto velE = *reinterpret_cast<int32_t *>(aligned_buf + 52);
          const auto velD = *reinterpret_cast<int32_t *>(aligned_buf + 56);
          const auto gSpeed = *reinterpret_cast<int32_t *>(aligned_buf + 60);
          const auto headMot = *reinterpret_cast<int32_t *>(aligned_buf + 64);
          const auto sAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 68);
          const auto headingAcc = *reinterpret_cast<uint32_t *>(aligned_buf + 72);
          const auto pDOP = *reinterpret_cast<uint32_t *>(aligned_buf + 76);
          printf("UBX PVT: valid: 0x%x, fixType: 0x%x, tAcc %u, numSV: %u, flags: %x\r\n", valid, fixType, tAcc, numSV, flags);
          const float f_lat = static_cast<float>(lat) / 10000000.0f;
          const float f_lon = static_cast<float>(lon) / 10000000.0f;
          const float h_acc_m = static_cast<float>(hAcc)/1000;
          const float v_acc_m = static_cast<float>(vAcc)/1000;
          valid_channel.new_data(valid);
          flags_channel.new_data(flags);
          longitude_channel.new_data(f_lon);
          latitude_channel.new_data(f_lat);
          h_acc_channel.new_data(h_acc_m);
          v_acc_channel.new_data(v_acc_m);
          altitude_channel.new_data(hMSL);
          fix_num_sv_channel.new_data(numSV);
          t_acc_channel.new_data(static_cast<float>(tAcc)/1000000000.0);
          last_time_accuracy_ns = tAcc;
          handled = true;
          break;
        }
        default:
          break;
      }
      break;
    default:
      break;
  }
  if (!handled) {
    printf("Unhandled UBX message: %02x %02x\r\n", msg_class, msg_id);
  }
}


uint32_t UBLOX_UBX::discover_device_baudrate() {
  uint32_t baudrates_to_try[] = {115200, 9600};
  uint32_t current_rate;

  for (auto baudrate : baudrates_to_try) {
    current_rate = baudrate;
    uart_set_baudrate(uart_dev, baudrate);
    clear_message_buffer();
    uint8_t payload[1] = {0x01};
    send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, payload, sizeof(payload));
    bool found = false;
    for (uint32_t i = 0; i < 100; i++) {
      sleep_ms(1);
      if (!latest_full_messages.is_empty) {
        found = true;
        printf("Found ublox gps at %d baud.\r\n", baudrate);
        break;
      }
    }
    if (found) {
      // Wait for the rx message to complete
      sleep_ms(100);
      break;
    }
  }
  return current_rate;
}

void UBLOX_UBX::send_nmea(std::string body) {
  uint8_t cksum = 0;
  for (auto c : body) {
    cksum ^= c;
  }
  char hex_chars[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  char cksum_buf[3] = {hex_chars[cksum&0x0F], hex_chars[(cksum>>4)&0x0F], 0x00};
  std::string full_msg = "$" + body + "*" + cksum_buf + "\r\n";
  uart_write_blocking(uart_dev, (uint8_t*)full_msg.c_str(), full_msg.length());
}

void UBLOX_UBX::update() {
  bool were_there_messages = false;
  uint64_t prev_most_recent_timestamp_seen = most_recent_timestamp_seen;
  critical_section_enter_blocking(&critical_section);
  for (auto msg : latest_full_messages)
  {
    were_there_messages = true;
    handle_message(msg.data(), msg.size());
  }
  latest_full_messages.clear();
  critical_section_exit(&critical_section);
  if ((most_recent_timestamp_seen!= prev_most_recent_timestamp_seen) && (!is_nil_time(pps_timestamp))) {

    // Sanity check for recent pps timestamp
    if (!is_going_to_sleep && absolute_time_diff_us(pps_timestamp, get_absolute_time()) < 100*1000)
    {
      // New time sync!
      TimeManager::new_gps_time(most_recent_timestamp_seen, pps_timestamp, last_time_accuracy_ns);
    }
  }

  if (!is_going_to_sleep && absolute_time_diff_us(last_status_poll, get_absolute_time()) > 30*1000*1000) {
    last_status_poll = get_absolute_time();

    uint8_t payload[4] = {};
    send_ubx(UBX_CLASS_NAV, UBX_NAV_SVINFO, payload, 0);
    //send_ubx(UBX_CLASS_AID, UBX_AID_EPH, payload, 0);
    //send_ubx(UBX_CLASS_AID, UBX_AID_ALM, payload, 0);
  }

  if (latest_assistnow_data_ready) {
    auto start_time = get_absolute_time();
    uart_write_blocking(uart_dev, latest_assistnow_data, latest_assistnow_data_len);
    auto end_time = get_absolute_time();
    printf("Wrote %d bytes of assistnow data to gps in %lld us.\r\n", latest_assistnow_data_len, absolute_time_diff_us(start_time, end_time));
    latest_assistnow_data_ready = false;
  }

  // Check if it's time to sleep
  if (do_power_save &&
      were_there_messages &&
      !is_nil_time(TimeManager::get_time_of_most_recent_full_gps_fix()) &&
      (absolute_time_diff_us(TimeManager::get_time_of_most_recent_full_gps_fix(), get_absolute_time()) < 60*1000000) &&
      (is_nil_time(last_sleep_command_time) || (absolute_time_diff_us(last_sleep_command_time, get_absolute_time()) > 60*1000000)))
  {
    start_sleep();
  }

  // Check if it's time to wake (if it has been an hour since the last time sync)
  if (do_power_save &&
      !were_there_messages &&
      (absolute_time_diff_us(TimeManager::get_time_of_most_recent_full_gps_fix(), get_absolute_time()) > 60*60*1000000LL) &&
      (absolute_time_diff_us(last_wake_command_time, get_absolute_time()) > 60*1000000LL))
  {
    start_wake();
  }
}

void UBLOX_UBX::lwip_update() {
  if (assistnow_online_enabled) {
    if (cyw43_shim_tcpip_link_status(CYW43_ITF_STA) == CYW43_LINK_UP)
    {
      if (is_nil_time(last_assistnow_online_download)) {
        if (is_nil_time(last_assistnow_online_download_attempt) ||
          absolute_time_diff_us(last_assistnow_online_download_attempt, get_absolute_time()) > 5*1000*1000) {
          start_assistnow_online_download();
          }
      }
    }
  }
}

void UBLOX_UBX::on_enter_dormant() {
  start_sleep();
}

void UBLOX_UBX::on_exit_dormant() {
  start_wake();
}

void UBLOX_UBX::use_assistnow_online(std::string token) {
  assistnow_online_token = std::move(token);
  assistnow_online_enabled = true;
}

static void tcp_err_cb_redirect(void *arg, err_t tpcb)
{
  static_cast<UBLOX_UBX *>(arg)->tcp_err_handler(tpcb);
}

static err_t tcp_recv_cb_redirect(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  return static_cast<UBLOX_UBX *>(arg)->tcp_recv_handler(tpcb, p, err);
}

static err_t tcp_send_cb_redirect(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  return static_cast<UBLOX_UBX *>(arg)->tcp_send_handler(tpcb, len);
}

static err_t connect_cb_redirect(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  return static_cast<UBLOX_UBX *>(arg)->connect_handler(tpcb, err);
}

static void dns_resolved_cb_redirect(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
  static_cast<UBLOX_UBX *>(callback_arg)->dns_resolved_cb(name, ipaddr);
}
void UBLOX_UBX::start_assistnow_online_download() {
  if (assistnow_connected || assistnow_connecting || assistnow_pcb)
  {
    return;
  }
  if (cyw43_shim_tcpip_link_status(CYW43_ITF_STA) != CYW43_LINK_UP)
  {
    return;
  }
  last_assistnow_online_download_attempt = get_absolute_time();
  ip_addr_t addr;
  cyw43_arch_lwip_begin();
  err_t ret = dns_gethostbyname_addrtype(assistnow_online_server.c_str(), &addr, dns_resolved_cb_redirect, this, LWIP_DNS_ADDRTYPE_IPV6_IPV4);
  cyw43_arch_lwip_end();
  assistnow_connecting = true;
  if (ret == ERR_OK) {
    dns_resolved_cb(assistnow_online_server.c_str(), &addr);
  }
  else if (ret == ERR_INPROGRESS)
  {
    // Callback should be called once resolved or failed
  }
  else {
    assistnow_connecting = false;
  }
}

void UBLOX_UBX::dns_resolved_cb(const char* string, const ip_addr* pAddr) {
  if (pAddr == nullptr)
  {
    printf("AssistNow: failed DNS resolution\r\n");
    assistnow_connecting = false;
  }
  else
  {
    cyw43_arch_lwip_begin();
    assistnow_pcb = tcp_new();
    tcp_arg(assistnow_pcb, this);
    tcp_err(assistnow_pcb, tcp_err_cb_redirect);
    tcp_recv(assistnow_pcb, tcp_recv_cb_redirect);
    tcp_sent(assistnow_pcb, tcp_send_cb_redirect);
    auto ret = tcp_connect(assistnow_pcb, pAddr, assistnow_online_port, connect_cb_redirect);
    printf("AssistNow attempting to connect!\r\n");
    cyw43_arch_lwip_end();
    if (ret != ERR_OK)
    {
      printf("AssistNow: failed to connect %d\r\n", ret);
      tcp_close(assistnow_pcb);
      assistnow_pcb = nullptr;
      assistnow_connecting = false;
    }
  }
}

err_t UBLOX_UBX::tcp_recv_handler(tcp_pcb* pPcb, pbuf* pPbuf, signed char err) {
  if (pPbuf == NULL) {
    if (assistnow_pcb)
    {
      tcp_close(assistnow_pcb);
      assistnow_connecting = false;
      assistnow_connected = false;
      assistnow_pcb = nullptr;
    }
    printf("AssistNow: Disconnecting...\r\n");
  } else {
    printf("AssistNow: Got %d bytes from server!\r\n", pPbuf->len);
    last_assistnow_online_download = get_absolute_time();
    uint32_t data_start_offset = 0;
    if (assistnow_awaiting_content_bytes == 0) {
      // Parse through data for content length header
      const char * filter = "Content-Length: ";
      uint32_t bytes_needed = strlen(filter) + 5;
      for (uint32_t i = 0; i < pPbuf->len-bytes_needed; i++) {
        if (strncmp(static_cast<char *>(pPbuf->payload) + i, filter, strlen(filter)) == 0) {
          assistnow_awaiting_content_bytes = strtol(static_cast<char *>(pPbuf->payload) + i + strlen(filter), nullptr, 10);
          break;
        }
      }
      // Find start of data
      filter = "\r\n\r\n";
      bytes_needed = strlen(filter) + 5;
      for (uint32_t i = 0; i < pPbuf->len-bytes_needed; i++) {
        if (strncmp(static_cast<char *>(pPbuf->payload) + i, filter, strlen(filter)) == 0) {
          data_start_offset = i + sizeof(filter);
          break;
        }
      }
      // Start data logging
      latest_assistnow_data_ready = false;
      if (latest_assistnow_data) {
        free(latest_assistnow_data);
        latest_assistnow_data = nullptr;
      }
      latest_assistnow_data = static_cast<uint8_t *>(malloc(assistnow_awaiting_content_bytes));
      latest_assistnow_data_len = 0;
    }

    if (assistnow_awaiting_content_bytes > 0) {
      const uint32_t data_to_copy = std::min(assistnow_awaiting_content_bytes, static_cast<uint32_t>(pPbuf->len) - data_start_offset);
      memcpy(latest_assistnow_data + latest_assistnow_data_len, static_cast<uint8_t *>(pPbuf->payload)+data_start_offset, data_to_copy);
      latest_assistnow_data_len += data_to_copy;
      assistnow_awaiting_content_bytes -= data_to_copy;
      if (assistnow_awaiting_content_bytes == 0) {
        latest_assistnow_data_ready = true;
        printf("Finished AssistNow data download!\r\n");
      }
    }

    tcp_recved(pPcb, pPbuf->len);
    pbuf_free(pPbuf);
  }
  return 0;
}

err_t UBLOX_UBX::connect_handler(tcp_pcb *pPcb, err_t err) {
  if (err)
  {
    printf("AssistNow connection err: %d", err);
    assistnow_connecting = false;
    assistnow_connected = false;
    return err;
  }
  else
  {
    printf("AssistNow connected!\r\n");
    assistnow_connected = true;
    assistnow_connecting = false;
    std::string header_data = "GET /GetOnlineData.ashx?gnss=gps;datatype=eph,alm,aux;format=aid;token=" + assistnow_online_token + "; "
              "HTTP/1.1\r\n"
              "Host: " + assistnow_online_server + "\r\n"
              "Accept: */* \r\n\r\n";
    tcp_write(assistnow_pcb, header_data.c_str(), header_data.length(), 0);
    tcp_output(assistnow_pcb);
    return ERR_OK;
  }
}

void UBLOX_UBX::tcp_err_handler(signed char i) {
  printf("AssistNow error cb, %d\r\n", i);
  tcp_close(assistnow_pcb);
  assistnow_connecting = false;
  assistnow_pcb = nullptr;
  assistnow_connected = false;
}

err_t UBLOX_UBX::tcp_send_handler(struct tcp_pcb *tpcb, u16_t len) {
  return ERR_OK;
}

void UBLOX_UBX::start_sleep() {
  uint8_t payload[44];
  // Send sleep config/command
  memset(payload, 0, sizeof(payload));
  payload[0] = 0x01; // Version
  // This defaults to 0x00029000
  *((uint32_t*)&payload[4]) = 0x00000000; // PSM config flags
  *((uint32_t*)&payload[8]) = 0; // Update period
  *((uint32_t*)&payload[12]) = 5000; // Search period
  *((uint32_t*)&payload[16]) = 0; // gridOffset
  *((uint16_t*)&payload[20]) = 0; // onTime
  *((uint16_t*)&payload[22]) = 0; // minAcqTime
  send_ubx(UBX_CLASS_CFG, UBX_CFG_PM2, payload, 44);

  // Enter power save mode
  memset(payload, 0, sizeof(payload));
  payload[0] = 0x08; // Always set for some reason
  payload[1] = 0x01; // Low power mode
  send_ubx(UBX_CLASS_CFG, UBX_CFG_RXM, payload, 2);

  last_sleep_command_time = get_absolute_time();
  is_going_to_sleep = true;
}

void UBLOX_UBX::start_wake() {
  uint8_t payload = 0xFF;
  uart_write_blocking(uart_dev, &payload, sizeof(payload));
  is_going_to_sleep = false;

  last_wake_command_time = get_absolute_time();
}

