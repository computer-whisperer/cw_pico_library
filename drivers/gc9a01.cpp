//
// Created by christian on 12/30/23.
//

#include "gc9a01.hpp"

#include <cstdint>

#define PROGMEM


#define TFT_HEIGHT 240
#define TFT_WIDTH 240
#include "GC9A01_Defines.h"

GC9A01::GC9A01(spi_inst_t* spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, int32_t rst_gpio_in, int32_t bl_gpio_in) :
  TFT_ESPI(spi_inst_in, cs_gpio_in, dc_gpio_in, rst_gpio_in, bl_gpio_in, TFT_WIDTH, TFT_HEIGHT)
{
#include "GC9A01_Init.h"

#include "GC9A01_Rotation.h"
}

static uint16_t swap_uint16( uint16_t val )
{
  return (val << 8) | (val >> 8 );
}

void GC9A01::do_display_callback(_lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {
  setCS(true);
  // Window write

  uint8_t addr;
  {
    uint16_t data[2];
    setDC(false);
    addr = TFT_CASET;
    spi_write_blocking(spi_inst, &addr, 1);
    setDC(true);
    data[0] = swap_uint16(area->x1+colstart);
    data[1] = swap_uint16(area->x2+colstart);
    spi_write_blocking(spi_inst, reinterpret_cast<uint8_t *>(data), 4);

    setDC(false);
    addr = TFT_PASET;
    spi_write_blocking(spi_inst, &addr, 1);
    setDC(true);
    data[0] = swap_uint16(area->y1+rowstart);
    data[1] = swap_uint16(area->y2+rowstart);
    spi_write_blocking(spi_inst, reinterpret_cast<uint8_t *>(data), 4);
  }

  setDC(false);
  addr = TFT_RAMWR;
  spi_write_blocking(spi_inst, &addr, 1);  /* RAMWR **/
  setDC(true);

  // Push pixels
  const uint32_t num_pixels = lv_area_get_width(area) * lv_area_get_height(area);
  const uint32_t num_bytes = sizeof(lv_color_t)*num_pixels;
  spi_write_blocking(spi_inst, reinterpret_cast<uint8_t *>(color_p), num_bytes);
  setCS(false);
  lv_disp_flush_ready(disp_drv);
}
