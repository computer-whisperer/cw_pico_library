//
// Created by christian on 10/26/23.
//

#ifndef _NTP_CLIENT_HPP
#define _NTP_CLIENT_HPP

#include <string>
#include "lwip/ip_addr.h"
#include "pico/time.h"

class NTPClient {
private:
  struct udp_pcb *pcb;
  ip_addr_t resolved_address{};
  const std::string ntp_server = "pool.ntp.org";
  const uint32_t ntp_port = 123;
  uint64_t current_offset_us = 0;
  absolute_time_t last_query_time = nil_time;
public:
  NTPClient();
  void dns_resolved_cb(const char *name, const ip_addr_t *ipaddr);
  void query();
  void ntp_recv(udp_pcb *pPcb, struct pbuf *pPbuf, const ip_addr_t *pAddr, u16_t port);
  uint64_t get_current_offset_us();
  void update();
};

#endif //_NTP_CLIENT_HPP
