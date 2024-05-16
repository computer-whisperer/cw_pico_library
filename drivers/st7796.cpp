//
// Created by christian on 12/29/23.
//

#include "st7796.hpp"

#include <cstdint>
#include <hardware/dma.h>
#define PROGMEM

#define TFT_HEIGHT 320
#define TFT_WIDTH 480

//#define TFT_RGB_ORDER 1

#include "ST7796_Defines.h"

static int dma_chan;
static _lv_disp_drv_t* last_disp_drv = nullptr;
static void dma_end() {
  if (last_disp_drv) {
        lv_disp_flush_ready(last_disp_drv);
  }
  dma_hw->ints0 = 1u << dma_chan;
}

ST7796::ST7796(spi_inst_t* spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, CWGPIO* rst_gpio_in, int32_t bl_gpio_in) :
  TFT_ESPI(spi_inst_in, cs_gpio_in, dc_gpio_in, rst_gpio_in, bl_gpio_in, TFT_WIDTH, TFT_HEIGHT)
{
  m=1;
#include "ST7796_Init.h"

#include "ST7796_Rotation.h"

  dma_channel = dma_claim_unused_channel(true);
  dma_chan = dma_channel;
  dma_channel_cfg = dma_channel_get_default_config(dma_channel);
  channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&dma_channel_cfg, spi_get_dreq(spi_inst, true));
  channel_config_set_read_increment(&dma_channel_cfg, true);
  channel_config_set_write_increment(&dma_channel_cfg, false);

  // Tell the DMA to raise IRQ line 0 when the channel finishes a block
  dma_channel_set_irq0_enabled(dma_channel, true);

  // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
  irq_set_exclusive_handler(DMA_IRQ_0, dma_end);
  irq_set_enabled(DMA_IRQ_0, true);

}

static uint16_t swap_uint16( uint16_t val )
{
  return (val << 8) | (val >> 8 );
}



void ST7796::do_display_callback(_lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {

  last_disp_drv = disp_drv;
  // Window write

  while(true)
  {
    if(!dma_channel_is_busy(dma_channel))  break;
  }

  setCS(true);

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
  /*
  for (uint32_t i = 0; i < num_pixels; i++)
  {
    //uint16_t data = swap_uint16(color_p[i].full);
    uint16_t data = color_p[i].full;
    spi_write_blocking(spi_inst, reinterpret_cast<uint8_t *>(&data), 2);
  }*/
#if 1
  // Color fix
  for (uint32_t i = 0; i < num_pixels; i++) {
    color_p[i].full = ~color_p[i].full;
  }

  dma_channel_configure(
        dma_channel,             // Channel to be configured
        &dma_channel_cfg,                     // The configuration we just created
        &spi_get_hw(spi_inst)->dr,                    // The initial write address
        color_p,                    // The initial read address
        num_pixels*2,            // Number of transfers; in this case each is 1 byte.
        true                    // Start immediately.
  );
  #else
  //set all pixels to red for testing
  for (uint32_t i = 0; i < num_pixels; i++) {
    lv_color_t color;
    color.full = 0xF800;
    color_p[i] = color;
  }
  lv_disp_flush_ready(disp_drv);

  #endif
  //setCS(false);
}
