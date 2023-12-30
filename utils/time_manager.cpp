//
// Created by christian on 12/28/23.
//
#include "time_manager.hpp"

#include <cstdio>
#include <ring_buffer.hpp>
#include <pico/time.h>



volatile static float local_clock_correction = 0;
volatile static uint32_t recent_time_updates = 0;
volatile static int32_t time_error_since_last_sync_us = 0;
volatile static uint64_t ntp_timestamp_offset_us = 0;
volatile static uint64_t gps_timestamp_offset_us = 0;

static absolute_time_t most_recent_gps_time_sync = nil_time;

struct time_point_t {
  uint64_t gps_time_us;
  absolute_time_t local_time;
};
RingBuffer<struct time_point_t> recent_time_points(10);
uint64_t last_gps_time_point_us = 0;

bool TimeManager::has_epoch() {
  if (ntp_timestamp_offset_us) {
    return true;
  }
  return false;
}

static uint64_t apply_rate_compensation(absolute_time_t local_time)
{
  uint64_t local_time_us = to_us_since_boot(local_time);
  return (uint64_t)((int64_t)local_time_us + (int64_t)(((float)local_time_us)*local_clock_correction));
}

uint64_t TimeManager::get_unix_time_us() {
  return convert_to_unix_time_us(get_absolute_time());
}

uint64_t TimeManager::convert_to_unix_time_us(absolute_time_t local_time) {
  if (gps_timestamp_offset_us) {
    return gps_timestamp_offset_us + apply_rate_compensation(local_time);
  }
  if (ntp_timestamp_offset_us) {
    return to_us_since_boot(local_time) + ntp_timestamp_offset_us;
  }
  return to_us_since_boot(local_time);
}

void TimeManager::new_ntp_time(const uint64_t ntp_unix_time_us, const absolute_time_t local_time) {
  printf("New NTP time!\r\n");
  ntp_timestamp_offset_us = ntp_unix_time_us - to_us_since_boot(local_time);
}

void TimeManager::new_gps_time(uint64_t gps_unix_time_us, absolute_time_t local_time) {
  if ((gps_unix_time_us - last_gps_time_point_us) > 60LL*1000000LL) {
    recent_time_points.clear();
    if (recent_time_updates != 0)
    {
      time_error_since_last_sync_us = (int64_t)convert_to_unix_time_us(local_time) - (int64_t)gps_unix_time_us;
    }

    recent_time_updates = 0;
  }


  struct time_point_t new_time_point = {gps_unix_time_us, local_time};
  recent_time_points.push(new_time_point);

  if (recent_time_points.get_num_entries() > 5) {
    // Get approximation of derivative
    struct time_point_t oldest_time_point = recent_time_points.get_value_by_age((int32_t) recent_time_points.get_num_entries() - 1);
    struct time_point_t newest_time_point = recent_time_points.get_value_by_age(0);
    uint64_t gps_time_elapsed_us = newest_time_point.gps_time_us - oldest_time_point.gps_time_us;
    uint64_t local_time_elapsed_us = absolute_time_diff_us(oldest_time_point.local_time, newest_time_point.local_time);
    float current_clock_correction = (float) (gps_time_elapsed_us - local_time_elapsed_us) / (float) local_time_elapsed_us;
    if (current_clock_correction == 0)
    {
      local_clock_correction = current_clock_correction;
      printf("New GPS time fix: %llu\r\n", gps_unix_time_us);
    }
    else
    {
      local_clock_correction += (current_clock_correction - local_clock_correction)/6;
    }


    recent_time_updates++;
  }

  if (recent_time_updates >= 20) {
    most_recent_gps_time_sync = get_absolute_time();
  }

  last_gps_time_point_us = gps_unix_time_us;

  gps_timestamp_offset_us = (gps_unix_time_us - apply_rate_compensation(local_time));
}

absolute_time_t TimeManager::get_time_of_most_recent_full_gps_fix() {
  return most_recent_gps_time_sync;
}

