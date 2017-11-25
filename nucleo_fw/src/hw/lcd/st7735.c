/*
 *  st7735.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram
 */
#include "lcd/st7735.h"
#include "hw.h"

#ifdef _USE_HW_ST7735



#define _TFTWIDTH   128
#define _TFTHEIGHT  128


#define _PIN_DEF_DC     0
#define _PIN_DEF_CS     1
#define _PIN_DEF_RST    2


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04


uint32_t colstart = 3;
uint32_t rowstart = 2;


typedef struct
{
  GPIO_TypeDef *p_port;
  uint16_t      pin_num;
} st7735_pin_t;


static st7735_pin_t pin_tbl[3] =
  {
  {GPIOA, GPIO_PIN_11},
  {GPIOA, GPIO_PIN_8},
  {GPIOB, GPIO_PIN_1}
  };


static uint16_t _width  = 96;
static uint16_t _height = 96;



//uint16_t frame_buf[128*128];
uint16_t frame_buf[96*96];


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00,   0,             //     XSTART = 0
      0x00, 127,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00,   0,             //     XSTART = 0
      0x00, 127 },           //     XEND = 127

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay




void st7735WritePin(uint8_t pin_num, uint8_t pin_state);
void st7735InitRegs(const uint8_t *addr);





void st7735Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;


  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

  GPIO_InitStruct.Pin = pin_tbl[_PIN_DEF_DC].pin_num;           // DC
  HAL_GPIO_Init(pin_tbl[_PIN_DEF_DC].p_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = pin_tbl[_PIN_DEF_RST].pin_num;         // RST
  HAL_GPIO_Init(pin_tbl[_PIN_DEF_RST].p_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = pin_tbl[_PIN_DEF_CS].pin_num;          // CS
  HAL_GPIO_Init(pin_tbl[_PIN_DEF_CS].p_port, &GPIO_InitStruct);

  st7735WritePin(_PIN_DEF_CS,  _DEF_HIGH);
  st7735WritePin(_PIN_DEF_DC,  _DEF_HIGH);
  st7735WritePin(_PIN_DEF_RST, _DEF_HIGH);


  st7735WritePin(_PIN_DEF_RST, _DEF_HIGH);
  delay(50);
  st7735WritePin(_PIN_DEF_RST, _DEF_LOW);
  delay(50);
  st7735WritePin(_PIN_DEF_RST, _DEF_HIGH);
  delay(50);


  spiBegin(_DEF_SPI1);
  spiSetBitOrder(_DEF_SPI1, SPI_FIRSTBIT_MSB);
  spiSetClockDivider(_DEF_SPI1, SPI_BAUDRATEPRESCALER_2);
  spiSetDataMode(_DEF_SPI1, SPI_MODE0);


  st7735InitRegs(Rcmd1);
  st7735InitRegs(Rcmd2green144);
  st7735InitRegs(Rcmd3);
}

void st7735WritePin(uint8_t pin_num, uint8_t pin_state)
{
  if (pin_state == _DEF_HIGH)
  {
    HAL_GPIO_WritePin(pin_tbl[pin_num].p_port, pin_tbl[pin_num].pin_num, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(pin_tbl[pin_num].p_port, pin_tbl[pin_num].pin_num, GPIO_PIN_RESET);
  }
}

void st7735WriteCommand(uint8_t c)
{
  st7735WritePin(_PIN_DEF_DC, _DEF_LOW);
  st7735WritePin(_PIN_DEF_CS, _DEF_LOW);

  spiTransfer8(_DEF_SPI1, c);

  st7735WritePin(_PIN_DEF_CS, _DEF_HIGH);

}


void st7735WriteData(uint8_t c)
{
  st7735WritePin(_PIN_DEF_DC, _DEF_HIGH);
  st7735WritePin(_PIN_DEF_CS, _DEF_LOW);

  spiTransfer8(_DEF_SPI1, c);

  st7735WritePin(_PIN_DEF_CS, _DEF_HIGH);
}

void st7735InitRegs(const uint8_t *addr)
{

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  /*
  numCommands = *addr++;   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    st7735WriteCommand(*addr++); //   Read, issue command
    numArgs  = *addr++;    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--)
    {                   //   For each argument...
      st7735WriteData(*addr++);  //     Read, issue argument
    }

    if (ms)
    {
      ms = *addr++; // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
  */
  numCommands = *addr;   // Number of commands to follow
  addr++;
  while(numCommands--) {                 // For each command...
    st7735WriteCommand(*addr); //   Read, issue command
    addr++;
    numArgs  = *addr;    //   Number of args to follow
    addr++;
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--)
    {                   //   For each argument...
      st7735WriteData(*addr);  //     Read, issue argument
      addr++;
    }

    if (ms)
    {
      ms = *addr; // Read post-command delay time (ms)
      addr++;
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }

}

uint16_t st7735GetWidth(void)
{
  return _width;
}

uint16_t st7735GetHeight(void)
{
  return _height;
}

void st7735SetRotation(uint8_t mode)
{
  st7735WriteCommand(ST7735_MADCTL);


  switch (mode)
  {
   case 0:
     st7735WriteData(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
     _width  = _TFTWIDTH;
     _height = _TFTHEIGHT;
     break;

   case 1:
     st7735WriteData(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     _width  = _TFTHEIGHT;
     _height = _TFTWIDTH;
     break;

  case 2:
    st7735WriteData(MADCTL_BGR);
    _width  = _TFTWIDTH;
    _height = _TFTHEIGHT;
    break;

   case 3:
     st7735WriteData(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
     _width  = _TFTHEIGHT;
     _height = _TFTWIDTH;
     break;
  }
}
void st7735SetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  st7735WriteCommand(ST7735_CASET); // Column addr set
  st7735WriteData(0x00);
  st7735WriteData(x0+colstart);     // XSTART
  st7735WriteData(0x00);
  st7735WriteData(x1+colstart);     // XEND

  st7735WriteCommand(ST7735_RASET); // Row addr set
  st7735WriteData(0x00);
  st7735WriteData(y0+rowstart);     // YSTART
  st7735WriteData(0x00);
  st7735WriteData(y1+rowstart);     // YEND

  st7735WriteCommand(ST7735_RAMWR); // write to RAM
}

void st7735DrawFrame(bool wait)
{
  if (wait == true)
  {
    st7735SetAddrWindow(0, 0, _width-1, _height-1);

    st7735WritePin(_PIN_DEF_DC, _DEF_HIGH);
    st7735WritePin(_PIN_DEF_CS, _DEF_LOW);

    spiDmaTransfer(_DEF_SPI1, frame_buf, _width*_height*2, 100);


    st7735WritePin(_PIN_DEF_CS, _DEF_HIGH);
  }
  else
  {
    st7735SetAddrWindow(0, 0, _width-1, _height-1);

    st7735WritePin(_PIN_DEF_DC, _DEF_HIGH);
    st7735WritePin(_PIN_DEF_CS, _DEF_LOW);


    spiDmaTransfer(_DEF_SPI1, frame_buf, _width*_height*2, 0);


    //digitalWrite(_cs, HIGH);
  }
}

void st7735FillRect(int16_t x, int16_t y, int16_t w, int16_t h,  uint16_t color)
{
  int32_t x_o = x;
  int32_t y_o = y;

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  uint8_t hi = color >> 8, lo = color;

  color = lo<<8 | hi<<0;

  for(y=0; y<h; y++) {
    for(x=0; x<w; x++) {
      frame_buf[(y_o+y)*_width+(x_o+x)] = color;
    }
  }
}


#endif
