#include <cstdio>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "ili9488.hpp"
#include "lvgl.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define LCD_SPI_PORT spi1
#define LCD_MISO 12
#define LCD_CS 13
#define LCD_SCK 14
#define LCD_MOSI 11
#define LCD_RESET 7
#define LCD_DC 6
#define LCD_LED 15
#define LCD_LED_PWM_MAX 2000

#define ILI9488_MADCTL_MY   0x80 // Row Address Order
#define ILI9488_MADCTL_MX   0x40 // Column Address Order
#define ILI9488_MADCTL_MV   0x20 // Row/Column Exchange
#define ILI9488_MADCTL_ML   0x10 // Vertical Refresh Order
#define ILI9488_MADCTL_BGR  0x08 // RGB-BGR ORDER
#define ILI9488_MADCTL_RGB  0x00
#define ILI9488_MADCTL_MH   0x04 // Horizontal Refresh Order


#define ILI9488_ORIENTATION_TOP     ILI9488_MADCTL_MY                                         // 320x480 ; FPC cable on the top side
#define ILI9488_ORIENTATION_RIGHT   ILI9488_MADCTL_MV                                         // 480x320 ; FPC cable on the right side
#define ILI9488_ORIENTATION_LEFT    ILI9488_MADCTL_MY | ILI9488_MADCTL_MX | ILI9488_MADCTL_MV // 480x320 ; FPC cable on the left side
#define ILI9488_ORIENTATION_BOTTOM  ILI9488_MADCTL_MX                                         // 320x480 ; FPC cable on the bottom side


#define ILI9488_MADCTL_DATA ((ILI9488_ORIENTATION_LEFT) | (ILI9488_MADCTL_BGR))


/* Level 1 Commands -------------- [section] Description */

#define ILI9488_NOP         0x00 // No Operation
#define ILI9488_SWRESET     0x01 // Software Reset
#define ILI9488_RDDIDIF     0x04 // Read Display Identification Information
#define ILI9488_RDNUMED     0x05 // Read Number of the Errors on DSI
#define ILI9488_RDDST       0x09 // Read Display Status
#define ILI9488_RDDPM       0x0A // Read Display Power Mode
#define ILI9488_RDDMADCTL   0x0B // Read Display MADCTL
#define ILI9488_RDDCOLMOD   0x0C // Read Display COLMOD
#define ILI9488_RDDIM       0x0D // Read Display Image Mode
#define ILI9488_RDDSM       0x0E // Read Display Signal Mode
#define ILI9488_RDDSDR      0x0F // Read Display Self-Diagnostic Result
#define ILI9488_SLPIN       0x10 // Sleep IN
#define ILI9488_SLPOUT      0x11 // Sleep OUT
#define ILI9488_PTLON       0x12 // Partial Mode ON
#define ILI9488_NORON       0x13 // Normal Display Mode ON
#define ILI9488_INVOFF      0x20 // Display Inversion OFF
#define ILI9488_INVON       0x21 // Display Inversion ON
#define ILI9488_ALLPOFF     0x22 // All Pixels OFF
#define ILI9488_ALLPON      0x23 // All Pixels ON
#define ILI9488_DISOFF      0x28 // Display OFF
#define ILI9488_DISON       0x29 // Display ON
#define ILI9488_CASET       0x2A // Column Address Set
#define ILI9488_PASET       0x2B // Page Address Set
#define ILI9488_RAMWR       0x2C // Memory Write
#define ILI9488_RAMRD       0x2E // Memory Read
#define ILI9488_PLTAR       0x30 // Partial Area
#define ILI9488_VSCRDEF     0x33 // Vertical Scrolling Definition
#define ILI9488_TEOFF       0x34 // Tearing Effect Line OFF
#define ILI9488_TEON        0x35 // Tearing Effect Line ON
#define ILI9488_MADCTL      0x36 // Memory Access Control
#define ILI9488_VSCRSADD    0x37 // Vertical Scrolling Start Address
#define ILI9488_IDMOFF      0x38 // Idle Mode OFF
#define ILI9488_IDMON       0x39 // Idle Mode ON
#define ILI9488_COLMOD      0x3A // Interface Pixel Format
#define ILI9488_RAMWRC      0x3C // Memory Write Continue
#define ILI9488_RAMRDRC     0x3E // Memory Read Continue
#define ILI9488_TESLWR      0x44 // Write Tear Scan Line
#define ILI9488_TESLRD      0x45 // Read Scan Line
#define ILI9488_WRDISBV     0x51 // Write Display Brightness Value
#define ILI9488_RDDISBV     0x52 // Read Display Brightness Value
#define ILI9488_WRCTRLD     0x53 // Write Control Display Value
#define ILI9488_RDCTRLD     0x54 // Read Control Display Value
#define ILI9488_WRCABC      0x55 // Write Content Adaptive Brightness Control Value
#define ILI9488_RDCABC      0x56 // Read Content Adaptive Brightness Control Value
#define ILI9488_WRCABCMB    0x5E // Write CABC Minimum Brightness
#define ILI9488_RDCABCMB    0x5F // Read CABC Minimum Brightness
#define ILI9488_RDABCSDR    0x68 // Read Automatic Brightness Control Self-diagnostic Result
#define ILI9488_RDID1       0xDA // Read ID1
#define ILI9488_RDID2       0xDB // Read ID2
#define ILI9488_RDID3       0xDC // Read ID3




#define ILI9488_IFMODE      0xB0 // Interface Mode Control
#define ILI9488_FRMCTR1     0xB1 // Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9488_FRMCTR2     0xB2 // Frame Rate Control (In Idle Mode/8 Colors)
#define ILI9488_FRMCTR3     0xB3 // Frame Rate Control (In Partial Mode/Full Colors)
#define ILI9488_INVTR       0xB4 // Display Inversion Control
#define ILI9488_PRCTR       0xB5 // Blanking Porch Control
#define ILI9488_DISCTRL     0xB6 // Display Function Control
#define ILI9488_ETMOD       0xB7 // Entry Mode Set
#define ILI9488_CECTRL1     0xB9 // Color Enhancement Control 1
#define ILI9488_CECTRL2     0xBA // Color Enhancement Control 2
#define ILI9488_HSLCTRL     0xBE // HS Lanes Control
#define ILI9488_PWCTRL1     0xC0 // Power Control 1
#define ILI9488_PWCTRL2     0xC1 // Power Control 2
#define ILI9488_PWCTRL3     0xC2 // Power Control 3 (For Normal Mode)
#define ILI9488_PWCTRL4     0xC3 // Power Control 4 (For Idle Mode)
#define ILI9488_PWCTRL5     0xC4 // Power Control 5 (For Partial Mode)
#define ILI9488_VMCTRL      0xC5 // VCOM Control
#define ILI9488_CABCCTRL1   0xC6 // CABC Control 1
#define ILI9488_CABCCTRL2   0xC8 // CABC Control 2
#define ILI9488_CABCCTRL3   0xC9 // CABC Control 3
#define ILI9488_CABCCTRL4   0xCA // CABC Control 4
#define ILI9488_CABCCTRL5   0xCB // CABC Control 5
#define ILI9488_CABCCTRL6   0xCC // CABC Control 6
#define ILI9488_CABCCTRL7   0xCD // CABC Control 7
#define ILI9488_CABCCTRL8   0xCE // CABC Control 8
#define ILI9488_CABCCTRL9   0xCF // CABC Control 9
#define ILI9488_NVMWR       0xD0 // NV Memory Write
#define ILI9488_NVMPKEY     0xD1 // NV Memory Protection Key
#define ILI9488_RDNVM       0xD2 // NV Memory Status Read
#define ILI9488_RDID4       0xD3 // Read ID4 - 0x009488
#define ILI9488_ADJCTL1     0xD7 // Adjust Control 1
#define ILI9488_RDIDV       0xD8 // Read ID Version
#define ILI9488_PGAMCTRL    0xE0 // Positive Gamma Control
#define ILI9488_NGAMCTRL    0xE1 // Negative Gamma Control
#define ILI9488_DGAMCTRL1   0xE2 // Ditigal Gamma Control 1
#define ILI9488_DGAMCTRL2   0xE3 // Ditigal Gamma Control 2
#define ILI9488_SETIMAGE    0xE9 // Set Image Function
#define ILI9488_ADJCTL2     0xF2 // Adjust Control 2
#define ILI9488_ADJCTL3     0xF7 // Adjust Control 3
#define ILI9488_ADJCTL4     0xF8 // Adjust Control 4
#define ILI9488_ADJCTL5     0xF9 // Adjust Control 5
#define ILI9488_RDEXTC      0xFB // Read EXTC command is SPI mode
#define ILI9488_ADJCTL6     0xFC // Adjust Control 6
#define ILI9488_ADJCTL7     0xFF // Adjust Control 7


uint slice_num;

typedef struct {
  uint8_t cmd;
  uint8_t dat[16];
  uint32_t datLen;
  uint32_t sleep;
} ili9488_ini_str_t;

ili9488_ini_str_t lcd_ini_str[] = {
        {ILI9488_SWRESET, {0x00}, 0, 200},                         /* software reset */
        {ILI9488_SLPOUT, {0x00}, 0, 20},
        {ILI9488_MADCTL, {ILI9488_MADCTL_DATA}, 1, 0},
        {ILI9488_COLMOD, {0x66}, 1, 0},
        {ILI9488_FRMCTR1, {0xA0}, 1, 0},
        {ILI9488_INVTR, {0x02}, 1, 0},
        {ILI9488_DISCTRL, {0x02, 0x02}, 2, 0},
        {ILI9488_PWCTRL1, {0x15, 0x17}, 2, 0},
        {ILI9488_PWCTRL1, {0x41}, 1, 0},
        {ILI9488_VMCTRL, {0x00, 0x12, 0x80}, 3, 0},
        {ILI9488_SETIMAGE, {0x00}, 1, 0},
        {ILI9488_ADJCTL3, {0xA9, 0x51, 0x2c, 0x82}, 4, 0},

        {ILI9488_PGAMCTRL, { 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F}, 15, 0},
        {ILI9488_NGAMCTRL, { 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F}, 15, 0},
        {ILI9488_INVOFF, {}, 0, 0},
        {ILI9488_NORON, {}, 0, 0},
        {ILI9488_DISON, {}, 0, 0},
        {0x00, {0x00}, 0, 0}                            /* EOL */
};

struct
{
  uint16_t width;
  uint16_t height;
} ili9488_resolution;

int dmaChannel;
dma_channel_config c;

void ili9488_Init()
{
  ili9488_resolution.width = 480;
  ili9488_resolution.height = 320;

  // // Get a free channel, panic() if there are none
  dmaChannel = dma_claim_unused_channel(true);
  c = dma_channel_get_default_config(dmaChannel);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_dreq(&c, DREQ_SPI1_TX);
  channel_config_set_read_increment(&c, true);
  channel_config_set_write_increment(&c, false);

  // SPI initialisation. This example will use SPI at 1MHz.
  spi_init(LCD_SPI_PORT, 46 * 1000 * 1000);
  gpio_set_function(LCD_MISO, GPIO_FUNC_SPI);
  gpio_set_function(LCD_CS, GPIO_FUNC_SIO);
  gpio_set_function(LCD_SCK, GPIO_FUNC_SPI);
  gpio_set_function(LCD_MOSI, GPIO_FUNC_SPI);

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(LCD_CS);
  gpio_set_dir(LCD_CS, GPIO_OUT);
  gpio_put(LCD_CS, true);
  gpio_init(LCD_DC);
  gpio_set_dir(LCD_DC, GPIO_OUT);
  gpio_put(LCD_DC, true);
  gpio_init(LCD_RESET);
  gpio_set_dir(LCD_RESET, GPIO_OUT);
  gpio_put(LCD_RESET, true);

  //gpio_init(LCD_LED);
  //gpio_set_dir(LCD_LED, GPIO_OUT);
  //gpio_put(LCD_LED, 1);

  // LCD BackLight PWM control
  gpio_set_function(LCD_LED, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(LCD_LED);

  pwm_config config = pwm_get_default_config();
  // Set divider, reduces counter clock to sysclock/this value
  pwm_config_set_clkdiv(&config, 4.f);
  // Load the configuration into our PWM slice, and set it running.
  pwm_init(slice_num, &config, true);

  pwm_set_wrap(slice_num, 8);
  pwm_set_enabled(slice_num, true);


  // initialize LCD
  ili9488_HardReset();

  ili9488_SstLED(10);
  ili9488_SendInitStr();
  ili9488_SetWindow(0, 0, ili9488_resolution.width, ili9488_resolution.height);
  ili9488_SstLED(100);
}

void ili9488_HardReset()
{
  gpio_put(LCD_RESET, true);
  sleep_ms(10);
  gpio_put(LCD_RESET, false);
  sleep_ms(100);
  gpio_put(LCD_RESET, true);
  sleep_ms(100);
}

void ili9488_SstLED(uint16_t parcent)
{
  if (parcent > 100)
  {
    parcent = 100;
  }
  pwm_set_chan_level(slice_num, pwm_gpio_to_channel(LCD_LED), parcent * 20);
}

void ili9488_SendInitStr()
{
  ili9488_SetCS(false);
  uint i = 0;
  while(lcd_ini_str[i].cmd != 0x00)
  {
    uint8_t cmd = lcd_ini_str[i].cmd;
    uint datLen = lcd_ini_str[i].datLen;
    uint8_t *dat;
    dat = &(lcd_ini_str[i].dat[0]);
    uint32_t slp = lcd_ini_str[i].sleep;

    ili9488_SetDC(false);
    spi_write_blocking(LCD_SPI_PORT, &cmd, 1);

    if(datLen > 0)
    {
      ili9488_SetDC(true);
      spi_write_blocking(LCD_SPI_PORT, dat, datLen);
    }
    if(slp > 0)
    {
      sleep_ms(slp);
    }
    i++;
  }
  ili9488_SetCS(true);
}

void ili9488_SetCS(bool val)
{
  asm volatile("nop\n");
  asm volatile("nop\n");
  gpio_put(LCD_CS, val);
  asm volatile("nop\n");
  asm volatile("nop\n");
}

void ili9488_SetDC(bool val)
{
  asm volatile("nop\n");
  asm volatile("nop\n");
  gpio_put(LCD_DC, val);
  asm volatile("nop\n");
  asm volatile("nop\n");
}

void ili9488_SendData(uint8_t cmd, uint8_t *data, uint16_t length)
{
  ili9488_SetCS(false);
  ili9488_SetDC(false);
  spi_write_blocking(LCD_SPI_PORT, &cmd, 1);
  ili9488_SetDC(true);
  spi_write_blocking(LCD_SPI_PORT, data, length);
  ili9488_SetCS(true);
}

void ili9488_SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  /* CASET */
  uint8_t cmd = ILI9488_CASET;
  uint8_t buf4[4];
  buf4[0] = (x >> 8) & 0xFF;
  buf4[1] = x & 0xFF;
  buf4[2] = ((x + w - 1) >> 8) & 0xFF;
  buf4[3] = (x + w - 1) & 0xFF;
  ili9488_SendData(cmd, buf4, 4);

  /* RASET */
  cmd = ILI9488_PASET;
  buf4[0] = (y >> 8) & 0xFF;
  buf4[1] = y & 0xFF;
  buf4[2] = ((y + h - 1) >> 8) & 0xFF;
  buf4[3] = (y + h - 1) & 0xFF;
  ili9488_SendData(cmd, buf4, 4);
}

void lcd_Flash_CB(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
  uint8_t cmd = ILI9488_RAMWR;
  uint x1, y1;
  x1 = area->x1;
  y1 = area->y1;
  uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);
  /*
   *  transfer pixel data via DMA function
   */
  while(true)
  {
    if(!dma_channel_is_busy(dmaChannel))  break;
  }
  ili9488_SetWindow(x1, y1, lv_area_get_width(area), lv_area_get_height(area));
  ili9488_SetCS(false);

  ili9488_SetDC(false);
  spi_write_blocking(LCD_SPI_PORT, &cmd, 1);  /* RAMWR **/
  ili9488_SetDC(true);

  // Convert from 32 to 24 bit data in-place.
  for (uint32_t i = 0; i < size; i++)
  {
    ((uint8_t*)color_p)[i*3 + 0] = color_p[i].ch.red;
    ((uint8_t*)color_p)[i*3 + 1] = color_p[i].ch.green;
    ((uint8_t*)color_p)[i*3 + 2] = color_p[i].ch.blue;
  }

  /*
   *  transfer pixel data via SPI function
   */
  //spi_write_blocking(LCD_SPI_PORT, (uint8_t *)color_p, size*3);

  //ili9488_SetCS(true);


  /*
   *  transfer pixel data via DMA function
   */
  lcd_Send_Color_DMA((void *) color_p,  size*3);
  lv_disp_flush_ready(disp_drv);
}

void lcd_Send_Color_DMA(void * buf, uint16_t length)
{
  dma_channel_configure(
          dmaChannel,             // Channel to be configured
          &c,                     // The configuration we just created
          &spi_get_hw(LCD_SPI_PORT)->dr,                    // The initial write address
          buf,                    // The initial read address
          length,                 // Number of transfers; in this case each is 1 byte.
          true                    // Start immediately.
  );
}

lv_coord_t lcd_Get_Width()
{
  return(lv_coord_t)(ili9488_resolution.width);
}

lv_coord_t lcd_Get_Height()
{
  return(lv_coord_t)(ili9488_resolution.height);
}