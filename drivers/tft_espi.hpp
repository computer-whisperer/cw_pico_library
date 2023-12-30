//
// Created by christian on 12/30/23.
//

#ifndef TFT_ESPI_HPP
#define TFT_ESPI_HPP
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <src/hal/lv_hal_disp.h>

class TFT_ESPI {
protected:
    uint32_t width;
    uint32_t height;

    lv_color_t *display_buf1 = nullptr;
    lv_color_t *display_buf2 = nullptr;
    lv_disp_draw_buf_t lv_disp_buf = {};
    lv_disp_drv_t lv_disp_drv = {};

    uint32_t rotation = 0;
    uint32_t m = 0;
    uint32_t colstart = 0;
    uint32_t rowstart = 0;
    uint32_t _width = 0;
    uint32_t _height = 0;
    uint32_t _init_width = 0;
    uint32_t _init_height = 0;

    spi_inst_t * spi_inst;
    int32_t cs_gpio;
    int32_t dc_gpio;
    int32_t rst_gpio;
    int32_t bl_gpio;
    uint32_t bl_pwm_slice_num = 0;
    void commandList(const uint8_t *addr);
    void writecommand(uint8_t);
    void writedata(uint8_t);
    void setDC(bool is_data) const {
        gpio_put(dc_gpio, is_data);
    }
    void setCS(bool is_selected) const {
        gpio_put(cs_gpio, !is_selected);
    }
public:
    virtual ~TFT_ESPI() = default;

    TFT_ESPI(spi_inst_t * spi_inst_in, int32_t cs_gpio_in, int32_t dc_gpio_in, int32_t rst_gpio_in, int32_t bl_gpio_in, uint32_t width_in, uint32_t height_in);
    void register_with_lvgl();
    virtual void do_display_callback(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {assert(false);};
};

#endif //TFT_ESPI_HPP
