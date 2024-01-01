//
// Created by christian on 12/28/23.
//

#ifndef TIME_MANAGER_HPP
#define TIME_MANAGER_HPP
#include <pico/types.h>
#include <cstdint>

class TimeManager
{
  public:
    static bool has_epoch();
    static uint64_t get_unix_time_us();
    static uint64_t convert_to_unix_time_us(absolute_time_t local_time);
    static void new_ntp_time(uint64_t ntp_unix_time_us, absolute_time_t local_time);
    static void new_gps_time(uint64_t gps_unix_time_us, absolute_time_t local_time, uint32_t time_accuracy_ns);
    static absolute_time_t get_time_of_most_recent_full_gps_fix();
};

#endif //TIME_MANAGER_HPP
