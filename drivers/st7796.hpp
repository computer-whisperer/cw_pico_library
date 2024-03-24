//
// Created by christian on 12/29/23.
//

#ifndef ST7796_HPP
#define ST7796_HPP
#include <tft_espi.hpp>
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <src/hal/lv_hal_disp.h>

class ST7796 final: public TFT_ESPI{
  static lv_disp_drv_t l_disp_drv;
  int dma_channel;
  dma_channel_config dma_channel_cfg;
public:
  ST7796(spi_inst_t * spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, CWGPIO* rst_gpio_in, int32_t bl_gpio_in);

  void do_display_callback(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) override;
};

#endif //ST7789V2_HPP
