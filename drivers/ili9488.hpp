#ifndef ILI9341_HPP
#define ILI9341_HPP

#include "lvgl.h"

void ili9488_Init();

void ili9488_HardReset();

void ili9488_SstLED(uint16_t parcent);

void ili9488_SendInitStr();

void ili9488_SetCS(bool val);

void ili9488_SetDC(bool val);

void ili9488_SendData(uint8_t cmd, uint8_t *data, uint16_t length);

void ili9488_setRotate(uint16_t rot);

void ili9488_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);

void lcd_Flash_CB(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);

void lcd_Send_Color_DMA(void *buf, uint16_t length);

lv_coord_t lcd_Get_Width(void);

lv_coord_t lcd_Get_Height(void);

#endif
