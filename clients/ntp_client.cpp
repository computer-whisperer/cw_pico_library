//
// Created by christian on 10/26/23.
//

#include <cstring>
#include "ntp_client.hpp"

#include <time_manager.hpp>

#include "lwip/err.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "pico/time.h"
#include "pico/cyw43_arch.h"
#include "cyw43_shim.h"

static void dns_resolved_cb_redirect(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
  ((NTPClient*)callback_arg)->dns_resolved_cb(name, ipaddr);
}

static void recv_redirect(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  ((NTPClient*)arg)->ntp_recv(pcb, p, addr, port);
}

void NTPClient::query() {
  cyw43_arch_lwip_begin();
  err_t ret = dns_gethostbyname_addrtype(ntp_server.c_str(), &resolved_address, dns_resolved_cb_redirect, this, LWIP_DNS_ADDRTYPE_IPV6_IPV4);
  cyw43_arch_lwip_end();
  if (ret == ERR_OK) {
    dns_resolved_cb(ntp_server.c_str(), &resolved_address);
  }
}

void NTPClient::dns_resolved_cb(const char *name, const ip_addr_t *ipaddr) {
  if (ipaddr == NULL)
  {
    return;
  }
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 48, PBUF_RAM);
  uint8_t *req = (uint8_t *) p->payload;
  memset(req, 0, 48);
  req[0] = 0x1b;
  cyw43_arch_lwip_begin();
  udp_sendto(pcb, p, &resolved_address, ntp_port);
  cyw43_arch_lwip_end();
  pbuf_free(p);
}

NTPClient::NTPClient() {
  cyw43_arch_lwip_begin();
  pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
  udp_recv(pcb, recv_redirect, this);
  cyw43_arch_lwip_end();
}

#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
void NTPClient::ntp_recv(udp_pcb *pPcb, struct pbuf *pPbuf, const ip_addr_t *pAddr, u16_t port) {
  uint8_t mode = pbuf_get_at(pPbuf, 0) & 0x7;
  uint8_t stratum = pbuf_get_at(pPbuf, 1);

  // Check the result
  if (ip_addr_cmp(pAddr, &resolved_address) && port == ntp_port && pPbuf->tot_len == 48 &&
      mode == 0x4 && stratum != 0) {
    uint8_t seconds_buf[4] = {0};
    pbuf_copy_partial(pPbuf, seconds_buf, sizeof(seconds_buf), 40);
    uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
    uint64_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
    TimeManager::new_ntp_time(seconds_since_1970*1000*1000, get_absolute_time());
    current_offset_us = seconds_since_1970*1000*1000 - to_us_since_boot(get_absolute_time());
    has_fix = true;
    printf("Got epoch: %llu\r\n", seconds_since_1970);
  } else {
    printf("invalid ntp response\n");
  }
  pbuf_free(pPbuf);
}

uint64_t NTPClient::get_current_offset_us() {
  return current_offset_us;
}

void NTPClient::update() {
  if (cyw43_shim_tcpip_link_status(CYW43_ITF_STA) != CYW43_LINK_UP)
  {
    return;
  }

  if (pcb == nullptr)
  {
    cyw43_arch_lwip_begin();
    pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    udp_recv(pcb, recv_redirect, this);
    cyw43_arch_lwip_end();
  }

  bool do_query = false;
  if (!has_fix) {
    if (absolute_time_diff_us(last_query_time, get_absolute_time()) > 1000000)
    {
      do_query = true;
    }
  }
  // Re-query every hour
  if (absolute_time_diff_us(last_query_time, get_absolute_time()) > 3600LL*1000000)
  {
    do_query = true;
  }
  if (do_query) {
    last_query_time = get_absolute_time();
    query();
  }
}
