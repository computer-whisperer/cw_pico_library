//
// Created by christian on 12/30/23.
//

#include "tft_espi.hpp"

#include <cstdlib>
#include <pico/time.h>

#include "hardware/pwm.h"

#define TFT_INIT_DELAY 0x80

void TFT_ESPI::commandList(const uint8_t* addr) {
  uint8_t  numCommands;
  uint8_t  numArgs;
  uint8_t  ms;

  numCommands = *(addr++);   // Number of commands to follow

  while (numCommands--)                  // For each command...
  {
    setCS(true);
    setDC(false);
    spi_write_blocking(spi_inst, addr++, 1);
    setDC(true);
    numArgs = *(addr++);     // Number of args to follow
    ms = numArgs & TFT_INIT_DELAY;       // If high bit set, delay follows args
    numArgs &= ~TFT_INIT_DELAY;          // Mask out delay bit

    if (numArgs) {
      spi_write_blocking(spi_inst, addr, numArgs);
    }
    addr += numArgs;
    setCS(false);

    if (ms)
    {
      ms = *(addr++);        // Read post-command delay time (ms)
      sleep_ms( (ms==255 ? 500 : ms) );
    }
  }
}

void TFT_ESPI::writecommand(uint8_t cmd) {
  setCS(true);
  setDC(false);
  spi_write_blocking(spi_inst, &cmd, 1);
}

void TFT_ESPI::writedata(uint8_t data) {
  setCS(true);
  setDC(true);
  spi_write_blocking(spi_inst, &data, 1);
}

TFT_ESPI::TFT_ESPI(spi_inst_t * spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, CWGPIO* rst_gpio_in, int32_t bl_gpio_in, uint32_t width_in, uint32_t height_in):
    spi_inst(spi_inst_in),
    cs_gpio(cs_gpio_in),
    dc_gpio(dc_gpio_in),
    rst_gpio(rst_gpio_in),
    bl_gpio(bl_gpio_in),
    width(width_in),
    height(height_in){
  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(cs_gpio);
  gpio_set_dir(cs_gpio, GPIO_OUT);
  gpio_put(cs_gpio, true);
  gpio_init(dc_gpio);
  gpio_set_dir(dc_gpio, GPIO_OUT);
  gpio_put(dc_gpio, true);
  rst_gpio->gpio_set_dir(GPIO_OUT);
  rst_gpio->gpio_put(true);
  //gpio_init(bl_gpio);
  //gpio_set_dir(bl_gpio, GPIO_OUT);
  //gpio_put(bl_gpio, true);

  // Do reset
  rst_gpio->gpio_put(true);
  sleep_ms(10);
  rst_gpio->gpio_put(false);
  sleep_ms(100);
  rst_gpio->gpio_put(true);
  sleep_ms(100);

  _init_height = _height;
  _init_width = _width;


  // LCD BackLight PWM control
  gpio_set_function(bl_gpio, GPIO_FUNC_PWM);
  bl_pwm_slice_num = pwm_gpio_to_slice_num(bl_gpio);


  pwm_config config = pwm_get_default_config();
  // Set divider, reduces counter clock to sysclock/this value
  pwm_config_set_clkdiv(&config, 4.f);
  // Load the configuration into our PWM slice, and set it running.
  pwm_init(bl_pwm_slice_num, &config, true);

  pwm_set_wrap(bl_pwm_slice_num, 100);
  pwm_set_enabled(bl_pwm_slice_num, true);
  pwm_set_chan_level(bl_pwm_slice_num, pwm_gpio_to_channel(bl_gpio), 110);
  gpio_set_slew_rate(bl_pwm_slice_num, GPIO_SLEW_RATE_SLOW);
}

static void hacky_flush_cb (struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
  static_cast<TFT_ESPI *>(disp_drv->user_data)->do_display_callback(disp_drv, area, color_p);
}

void TFT_ESPI::register_with_lvgl() {
  // Allocate buffers
  uint32_t buffer_pixel_count = width * height / 10;
  display_buf1 = static_cast<lv_color_t *>(malloc(sizeof(lv_color_t) * buffer_pixel_count));
  display_buf2 = static_cast<lv_color_t *>(malloc(sizeof(lv_color_t) * buffer_pixel_count));
  /*Initialize `disp_buf` with the buffer(s). With only one buffer use NULL instead buf_2 */
  lv_disp_draw_buf_init(&lv_disp_buf, display_buf1, display_buf2, buffer_pixel_count);

  // Allocate driver
  lv_disp_drv_init(&lv_disp_drv);
  lv_disp_drv.draw_buf = &lv_disp_buf;
  lv_disp_drv.hor_res = static_cast<lv_coord_t>(width);
  lv_disp_drv.ver_res = static_cast<lv_coord_t>(height);
  lv_disp_drv.user_data = this;
  lv_disp_drv.flush_cb = hacky_flush_cb;
  lv_disp_drv_register(&lv_disp_drv);
}