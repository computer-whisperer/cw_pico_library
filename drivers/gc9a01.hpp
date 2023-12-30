//
// Created by christian on 12/30/23.
//

#ifndef GC9A01_HPP
#define GC9A01_HPP

#include <tft_espi.hpp>
#include <hardware/spi.h>
#include <src/hal/lv_hal_disp.h>

class GC9A01 final: public TFT_ESPI{
  static lv_disp_drv_t l_disp_drv;
public:
  GC9A01(spi_inst_t * spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, int32_t rst_gpio_in, int32_t bl_gpio_in);

  void do_display_callback(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) override;
};

#endif //GC9A01_HPP
