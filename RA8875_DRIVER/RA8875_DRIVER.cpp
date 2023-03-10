/*
  Author: Sugiarto Wibowo
  Email: osugiartow@gmail.com
  Github: https://github.com/osugiw
  File: RA8875_DRIVER.cpp

  Some codes are implemented from https://github.com/adafruit/Adafruit_RA8875
  WARNING:
          - This program has successfully implemented on ESP32-C312F, with display resolution of 800x480
          - If your project plan to use SD Card module, please use separate BUS with RA8875's BUS, because the RA8875 driver doesn't support Tri-State 
          - Some of functions are still in progress or have not found the solution yet. Those functions are DMA, BTE, and FastDrawing
          - If you plan to use with LVGL please use example from my repository, but be aware that the drawing process is not optimized yet.
            As far I know, this because we could not use PIO or I2C in Adafruit_RA8875 Module. Actually the datasheet says I2C and PIO can be used,
            however you need to modify the hardware.
          - Change pins assignment of the RA8875 and SD card driver
*/

#include "RA8875_DRIVER.h"

#define CLK_SPI_INIT  125000
#define CLK_SPI_WRITE 40000000      // (System Clock / 3)
#define CLK_SPI_READ  20000000      // (System Clock / 6)

// SPI Configuration & Inline Function
// @cond DISABLE
#if defined(ARDUINO_ARCH_ARC32)
/// @endcond
uint32_t spi_speed = 12000000; /*!< 12MHz */
/// @cond DISABLE
#else
/// @endcond
uint32_t spi_speed = CLK_SPI_WRITE; /*!< 60MHz */
/// @cond DISABLE
#endif
/// @endcond

SPIClass * fspi = NULL;
#ifdef SPI_HAS_TRANSACTION
static inline void SPI_Begin(void) __attribute__((always_inline));
static inline void SPI_Begin(void) {fspi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));}
static inline void SPI_End(void) __attribute__((always_inline));
static inline void SPI_End(void) {fspi->endTransaction();}
#else
#define SPI_Begin()
#define SPI_End()
#endif

/**********************************************************************************
                                    Constructor
***********************************************************************************/
TFT_RA8875::TFT_RA8875(uint8_t _wait, uint8_t _sclk, uint8_t _miso, uint8_t _mosi, uint8_t _cs, uint8_t _int, uint8_t _rst) {
  RA8875_WAIT = _wait;
  RA8875_SCLK = _sclk;
  RA8875_MISO = _miso;
  RA8875_MOSI = _mosi;
  RA8875_CS   = _cs;
  RA8875_INT  = _int;
  RA8875_RST  = _rst;
}

/**********************************************************************************
                              Perform Software Reset
***********************************************************************************/
void TFT_RA8875::softwareReset(){
  writeCommand(RA8875_PWRR_ADDR);
  writeData(RA8875_PWRR_DISPLAY_RESET);
  writeData(RA8875_PWRR_DISPLAY_NORMAL);
  delay(1);
}

/**********************************************************************************
                              Get Parameter Data
***********************************************************************************/
uint16_t TFT_RA8875::get_width(void){return screen_width;}
uint16_t TFT_RA8875::get_height(void){return screen_height;}
int8_t TFT_RA8875::get_Rotation(void){return _rotation;}
int16_t TFT_RA8875::get_window_color(void){return _window_color;}

/**********************************************************************************
                        Initialize Peripheral, PLL, & Window
***********************************************************************************/
void TFT_RA8875::set_rotation(int8_t rotation) {
  switch(rotation) {
    case 2:
      _rotation = rotation;
      break;
    default:
      _rotation = 0;
      break;
  }
}
boolean TFT_RA8875::init() {
  _rotation = 0;  
  pinMode(RA8875_CS, OUTPUT);
  digitalWrite(RA8875_CS, HIGH);
  pinMode(RA8875_RST, OUTPUT);
  digitalWrite(RA8875_RST, LOW);
  delay(1);
  digitalWrite(RA8875_RST, HIGH);
  delay(10);
  
  fspi = new SPIClass(FSPI);
  fspi->begin(RA8875_SCLK, RA8875_MISO, RA8875_MOSI, RA8875_CS);
  // SPI.begin(RA8875_SCLK, RA8875_MISO, RA8875_MOSI, RA8875_CS);

#ifdef SPI_HAS_TRANSACTION
/// @cond DISABLE
#if defined(ARDUINO_ARCH_ARC32)
  /// @endcond
  spi_speed = 2000000;
/// @cond DISABLE
#else
  /// @endcond
  spi_speed = CLK_SPI_INIT;
/// @cond DISABLE
#endif
/// @endcond
#else
#ifdef __AVR__
  fspi->setClockDivider(SPI_CLOCK_DIV128);
  fspi->setDataMode(SPI_MODE0);
#endif
#endif

  uint8_t x = readReg(0);
  if (x != 0x75) {
    printf("%d\n", x);
    return false;
  }

  PLL_init(); // Initialize PLL Clock Frequency
  writeReg(RA8875_SYSR_ADDR, RA8875_SYSR_COLOR_16BPP | RA8875_SYSR_MCU_8BIT); // 16bpp Color depth & 8 bit MCU Interface
  window_init();

#ifdef SPI_HAS_TRANSACTION
/// @cond DISABLE
#if defined(ARDUINO_ARCH_ARC32)
  /// @endcond
  spi_speed = 12000000;
#else
  spi_speed = CLK_SPI_WRITE;
#endif
#else
#ifdef __AVR__
  fspi->setClockDivider(SPI_CLOCK_DIV4);
#endif
#endif

  return true;
}

void TFT_RA8875::PLL_init(void) {
#if defined(RA8875_480x8) || defined(RA8875_480x128) || defined(RA8875_480x272)
  writeReg(RA8875_PLLC1_ADDR, RA8875_PLLC1_DIV_1 + 10);   // System Clock: 55MHz (20*(10+1)/2^2
  delay(1); // lock time
#else // RA8875_800x480
  writeReg(RA8875_PLLC1_ADDR, RA8875_PLLC1_DIV_1 + 11);   // System Clock: 60MHz (20*(11+1)/2^2)
  delay(1); // lock time
#endif
  writeReg(RA8875_PLLC2_ADDR, RA8875_PLLC2_DIV2); // Div1: 240 MHz, Div2: 120 MHz, Div4: 60 MHz
  delay(1); // lock time
}

void TFT_RA8875::window_init() {
#if defined(RA8875_480x80)  
  screen_width = 480; 
  screen_height = 80;
  pixclk = RA8875_PCSR_PDAT_FALLING | RA8875_PCSR_4CLK;
  hsync_nondisp = 10;
  hsync_start = 8;
  hsync_pw = 48;
  hsync_finetune = 0;
  vsync_nondisp = 3;
  vsync_start = 8;
  vsync_pw = 10;
  _voffset = 192; // This uses the bottom 80 pixels of a 272 pixel controller
#elif defined(RA8875_480x128)
  screen_width = 480; 
  screen_height = 128;
  pixclk = RA8875_PCSR_PDAT_FALLING | RA8875_PCSR_4CLK;
  hsync_nondisp = 10;
  hsync_start = 8;
  hsync_pw = 48;
  hsync_finetune = 0;
  vsync_nondisp = 3;
  vsync_start = 8;
  vsync_pw = 10;
  _voffset = 0;
#elif defined(RA8875_480x272)
  screen_width = 480; 
  screen_height = 272;
  pixclk = RA8875_PCSR_PDAT_FALLING | RA8875_PCSR_4CLK;
  hsync_nondisp = 10;
  hsync_start = 8;
  hsync_pw = 48;
  hsync_finetune = 0;
  vsync_nondisp = 3;
  vsync_start = 8;
  vsync_pw = 10;
  _voffset = 0;
#elif defined(RA8875_800x480)
  screen_width = 800; 
  screen_height = 480;
  pixclk = RA8875_PCSR_PDAT_FALLING | RA8875_PCSR_2CLK;   // 30 MHz = (System Clock / 2) = 60/2
  hsync_nondisp = 26;
  hsync_start = 32;
  hsync_pw = 96;
  hsync_finetune = 3; 
  // hsync_finetune = 0;
  vsync_nondisp = 32;
  vsync_start = 23;
  vsync_pw = 2;
  _voffset = 0;
#else
  printf("Setting for the size is not available, choose available sizes only!\n");
#endif
  
  writeReg(RA8875_PCSR_ADDR, pixclk);                                                           // Pixel Clock Settings
  delay(1);

  // Horizontal Settings Registers
  writeReg(RA8875_HDWR_ADDR, (screen_width / 8) - 1);                                           // LCD Horizontal Display Width (HDWR = (Width/8)-1)
  writeReg(RA8875_HNDFTR_ADDR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);                         // Period Fine Tuning for Horizontal Non-Display
  writeReg(RA8875_HNDR_ADDR, (hsync_nondisp - hsync_finetune - 2) / 7);                         // LCD Horizontal Non-Display Period (HNDR = (HSYNC_Nondisp-HSYNC_Finetune-2)/8)
  // writeReg(RA8875_HNDR_ADDR, (hsync_nondisp - hsync_finetune - 2) / 8);                         // LCD Horizontal Non-Display Period (HNDR = (HSYNC_Nondisp-HSYNC_Finetune-2)/8)
  writeReg(RA8875_HSTR_ADDR, (hsync_start / 8) - 2);                                            // HSYNC Start Position (HSTR = (HSYNC_Start/8)-1)
  // writeReg(RA8875_HSTR_ADDR, (hsync_start / 8) - 1);                                            // HSYNC Start Position (HSTR = (HSYNC_Start/8)-1)
  writeReg(RA8875_HPWR_ADDR, RA8875_HPWR_POLARITY_LOW + (hsync_pw / 8 - 12));                    // HSYNC Pulse Width (HPW = (HSYNC_PW/8)-1)
  // writeReg(RA8875_HPWR_ADDR, RA8875_HPWR_POLARITY_LOW + (hsync_pw / 8 - 1));                    // HSYNC Pulse Width (HPW = (HSYNC_PW/8)-1)

  // Vertical Setting Registers
  writeReg(RA8875_VDHR0_ADDR, (uint16_t)(screen_height - 1 + _voffset) & 0xFF);                 // LCD Vertical Height Register 0 (VDHR = Height-1+V_Offset)
  writeReg(RA8875_VDHR1_ADDR, (uint16_t)(screen_height - 1 + _voffset) >> 8);                   // LCD Vertical Height Register 1 (VDHR = Height-1+V_Offset)
  writeReg(RA8875_VNDR0_ADDR, vsync_nondisp - 12);                                               // LCD Vertical Non-Display Period (VNDR = VSYNC_NonDisplay - 1)
  // writeReg(RA8875_VNDR0_ADDR, vsync_nondisp - 1);                                               // LCD Vertical Non-Display Period (VNDR = VSYNC_NonDisplay - 1)
  writeReg(RA8875_VNDR1_ADDR, (vsync_nondisp - 1)  >> 8);                                             // LCD Vertical Non-Display Period (VNDR = VSYNC_NonDisplay - 1) >> 8
  writeReg(RA8875_VSTR0_ADDR, vsync_start - 17);                                                 // VSYNC Start Position Register (VSTR = VSYNC_Start - 1)
  // writeReg(RA8875_VSTR0_ADDR, vsync_start - 1);                                                 // VSYNC Start Position Register (VSTR = VSYNC_Start - 1)
  writeReg(RA8875_VSTR1_ADDR, (vsync_start - 1) >> 8);                                                // VSYNC Start Position Register (VSTR = VSYNC_Start - 1) >> 8
  writeReg(RA8875_VPWR_ADDR, (RA8875_VPWR_POLARITY_LOW + vsync_pw - 1));                        // VSYNC Pulse Width (VPWR = VSYNC_PW - 1)

  // Setting active area 
  setActiveWindow();
  
  // Clear Window
  writeReg(RA8875_MCLR_ADDR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  delay(500);
}

void TFT_RA8875::setActiveWindow(void) {
  // Active Window Configuration
  writeReg(RA8875_HSAW0_ADDR, 0);                                                               // Horizontal Start point is at 0
  writeReg(RA8875_HSAW1_ADDR, 0 >> 8);
  writeReg(RA8875_HEAW0_ADDR, (uint16_t)(screen_width - 1 ) & 0xFF);                            // Horizontal End point is at x
  writeReg(RA8875_HEAW1_ADDR, (uint16_t)(screen_width - 1 ) >> 8);
  
  writeReg(RA8875_VSAW0_ADDR, 0 + _voffset);                                                    // Vertical Start point is at 0
  writeReg(RA8875_VSAW1_ADDR, (0 + _voffset) >> 8);
  writeReg(RA8875_VEAW0_ADDR, (uint16_t)(screen_height - 1 + _voffset) & 0xFF);                 // Vertical End point is at y
  writeReg(RA8875_VEAW1_ADDR, (uint16_t)(screen_height - 1 + _voffset) >> 8);
}

/**********************************************************************************
                              Display On/Off
***********************************************************************************/
void TFT_RA8875::displayOn()  {writeReg(RA8875_PWRR_ADDR, RA8875_PWRR_DISPLAY_ON | RA8875_PWRR_DISPLAY_NORMAL);}
void TFT_RA8875::displayOff() {writeReg(RA8875_PWRR_ADDR, RA8875_PWRR_DISPLAY_OFF | RA8875_PWRR_DISPLAY_NORMAL);}

/**********************************************************************************
                            Enable/Disable GPIOX
***********************************************************************************/
void TFT_RA8875::enableGPIOX()  {writeReg(RA8875_GPIOX_ADDR, true);}
void TFT_RA8875::disableGPIOX() {writeReg(RA8875_GPIOX_ADDR, false);}

/**********************************************************************************
                      Enable/Disable PWM & Set Duty Cycle
    @param clock   PWM Clock Divider Selection
    @param dc      PWM Duty Cycle Value
***********************************************************************************/
void TFT_RA8875::enablePWM1(uint8_t clock)  {writeReg(RA8875_P1CR_ADDR, RA8875_P1CR_ENABLE | (clock & 0xF));}
void TFT_RA8875::disablePWM1(uint8_t clock)  {writeReg(RA8875_P1CR_ADDR, RA8875_P1CR_DISABLE | (clock & 0xF));}
void TFT_RA8875::setPWM1DutyCycle(uint8_t dc) {writeReg(RA8875_P1DCR_ADDR, dc);}
void TFT_RA8875::enablePWM2(uint8_t clock)  {writeReg(RA8875_P2CR_ADDR, RA8875_P2CR_ENABLE | (clock & 0xF));}
void TFT_RA8875::disablePWM2(uint8_t clock)  {writeReg(RA8875_P2CR_ADDR, RA8875_P2CR_DISABLE | (clock & 0xF));}
void TFT_RA8875::setPWM2DutyCycle(uint8_t dc) {writeReg(RA8875_P2DCR_ADDR, dc);}

/**********************************************************************************                        
                      Wait for the command to finished
    @param register_address   Register Status to read
    @param waitFlag           Return value of register status                 
***********************************************************************************/
boolean TFT_RA8875::waitPoll(uint8_t register_address, uint8_t waitFlag) {
  while(1){
    uint8_t temp = readReg(register_address);
    if (!(temp & waitFlag))
      return true;
  }
  return false;
}

/**********************************************************************************                        
                        Write Data to Register
  @param d    Data to write                  
***********************************************************************************/
void TFT_RA8875::writeData(uint8_t d) {
  digitalWrite(RA8875_CS, LOW);
  SPI_Begin();
  fspi->transfer(RA8875_DATAWRITE);
  fspi->transfer(d);
  SPI_End();
  digitalWrite(RA8875_CS, HIGH);
}

/**********************************************************************************                        
                         Read Data to Register                  
***********************************************************************************/
uint8_t TFT_RA8875::readData(void) {
  digitalWrite(RA8875_CS, LOW);
  SPI_Begin();
  fspi->transfer(RA8875_DATAREAD);
  uint8_t spi_data = fspi->transfer(0x0);
  SPI_End();
  digitalWrite(RA8875_CS, HIGH);
  return spi_data;
}

/**********************************************************************************                       
                        Write Command to Register  
    @param d    Command to write               
***********************************************************************************/
void TFT_RA8875::writeCommand(uint8_t data) {
  digitalWrite(RA8875_CS, LOW);
  SPI_Begin();
  fspi->transfer(RA8875_CMDWRITE);
  fspi->transfer(data);
  SPI_End();
  digitalWrite(RA8875_CS, HIGH);
}

/**********************************************************************************                        
                            Read Status Data               
***********************************************************************************/
uint8_t TFT_RA8875::readStatus(void) {
  digitalWrite(RA8875_CS, LOW);
  SPI_Begin();
  fspi->transfer(RA8875_CMDREAD);
  uint8_t spi_data = fspi->transfer(0x0);
  SPI_End();
  digitalWrite(RA8875_CS, HIGH);
  return spi_data;
}

/**********************************************************************************                       
                      Write to Register with Data 
    @param register_address   Register Status to write
    @param val                Value for register setting               
***********************************************************************************/
void TFT_RA8875::writeReg(uint8_t register_addr, uint8_t val) {
  writeCommand(register_addr);
  writeData(val);
}

/**********************************************************************************                       
                        Read data from Register      
    @param register_address   Register data to read
***********************************************************************************/
uint8_t TFT_RA8875::readReg(uint8_t register_addr) {
  writeCommand(register_addr);
  return readData();
}

/**********************************************************************************                       
                        Apply Rotation on X Axis 
    @param X  X-Axis coordinate to rotate
***********************************************************************************/
int16_t TFT_RA8875::applyRotationX(int16_t x)  {
  switch (_rotation) {
    case 2:
      x = screen_width - 1 - x;
      break;
  }
  return x;
}

/**********************************************************************************                       
                        Apply Rotation on Y Axis      
    @param Y  Y-Axis coordinate to rotate     
***********************************************************************************/
int16_t TFT_RA8875::applyRotationY(int16_t y)  {
  switch (_rotation) {
    case 2:
      y = screen_width - 1 - y;
      break;
  }
  return y + _voffset;
}

/**********************************************************************************                       
                            Draw Pixel    
    @param X  X-Axis Coordinate to draw pixel
    @param Y  Y-Axis coordinate to draw pixel           
***********************************************************************************/
void TFT_RA8875::graphicSetCoordinate(uint16_t x, uint16_t y)  {
  writeReg(RA8875_CURH0_ADDR, x);
  writeReg(RA8875_CURH1_ADDR, x >> 8);
  writeReg(RA8875_CURV0_ADDR, y);
  writeReg(RA8875_CURV1_ADDR, y >> 8);
}

void TFT_RA8875::drawPixel(uint16_t x, uint16_t y, int16_t color)  {
  enableGraphicsMode();
  x = applyRotationX(x);
  y = applyRotationY(y);
  
  // Setting Coordinate
  graphicSetCoordinate(x, y);

  // Draw Pixel
  writeCommand(RA8875_MRWC_ADDR);
  digitalWrite(RA8875_CS, LOW);
  fspi->transfer(RA8875_DATAWRITE);
  fspi->transfer16(color);
  digitalWrite(RA8875_CS, HIGH);
}
void TFT_RA8875::drawPixels(int16_t x, int16_t y, uint16_t *p, uint32_t num)  {
  x = applyRotationX(x);
  y = applyRotationY(y);
  // spi_speed = 40000000;

  // Setting Coordinate
  graphicSetCoordinate(x, y);

  // Pixels direction - LRTD: Default, on Rotation: RLTD
  uint8_t dir = RA8875_MWCR0_LRTD;
  if (_rotation == 2) {
    dir = RA8875_MWCR0_RLTD;
  }
  writeReg(RA8875_MWCR0_ADDR, (readReg(RA8875_MWCR0_ADDR) & ~RA8875_MWCR0_DIRMASK) | dir);

  // Draw Pixels
  writeCommand(RA8875_MRWC_ADDR);
  digitalWrite(RA8875_CS, LOW);
  fspi->transfer(RA8875_DATAWRITE);
  while(num--) {
    fspi->transfer16(*p++);
  }
  digitalWrite(RA8875_CS, HIGH);
}

/**********************************************************************************                       
            HW accelerated function to push a chunk of raw pixel data
    @param num The number of pixels to push
    @param p   The pixel color to use      
***********************************************************************************/
void TFT_RA8875::pushPixels(uint32_t num, uint16_t* p) {
  digitalWrite(RA8875_CS, LOW);
  fspi->transfer(RA8875_DATAWRITE);
  while(num--)  {
    fspi->transfer16(*p);
    p++;
  }
  digitalWrite(RA8875_CS, HIGH);
}

/**********************************************************************************                       
                        Filling Foreground Color
    @param Color  Color to set for foreground color (filling shape)              
***********************************************************************************/
void TFT_RA8875::foregroundColor(uint16_t color) {
  writeReg(RA8875_FGCR0_ADDR, (color & 0xF800) >> 11);
  writeReg(RA8875_FGCR1_ADDR, (color & 0x07E0) >> 5);
  writeReg(RA8875_FGCR2_ADDR, (color & 0x001F));
}

/**********************************************************************************                       
                            Background Color
    @param Color  Color to set for background color               
***********************************************************************************/
void TFT_RA8875::backgroundColor(uint16_t color)  {
  writeReg(RA8875_BGCR0_ADDR, (color & 0xF800) >> 11);
  writeReg(RA8875_BGCR1_ADDR, (color & 0x07E0) >> 5);
  writeReg(RA8875_BGCR2_ADDR, (color & 0x001F));
}

/**********************************************************************************                       
                    Helper for Drawing Shapes / Graphics  
    @param X            X-Axis start point coordinate
    @param Y            Y-Axis start point coordinate
    @param W            Width of rectangle shape
    @param h            Height of rectangle shape
    @param filled       State for fill/empty the shape
    @param r            Radius of circle
    @param color        Color of the shape
    @param long_axis    Long axis of Ellipse shape
    @param short-axis   Short axis of Ellipse shape
    @param curvePart    Curve Part of the circle selection
    @param  x0          X-Axis first point coordinate triangle
    @param  y0          Y-Axis first point coordinate triangle
    @param  x1          X-Axis second point coordinate triangle
    @param  y1          Y-Axis second point coordinate triangle
    @param  x2          X-Axis third point coordinate triangle
    @param  y2          Y-Axis third point coordinate triangle
                
***********************************************************************************/
void TFT_RA8875::rectHelper(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w + x - 1);
  h = applyRotationY(h + y - 1);

  // Setting X and Y start point
  writeReg(RA8875_DLHSR0_ADDR, x);
  writeReg(RA8875_DLHSR1_ADDR, x >> 8);
  writeReg(RA8875_DLVSR0_ADDR, y);
  writeReg(RA8875_DLVSR1_ADDR, y >> 8);

  // Setting X and Y end point
  writeReg(RA8875_DLHER0_ADDR, w);
  writeReg(RA8875_DLHER1_ADDR, w >> 8);
  writeReg(RA8875_DLVER0_ADDR, h);
  writeReg(RA8875_DLVER1_ADDR, h >> 8);

  // Set the foreground color
  foregroundColor(color);

  // Draw Square
  if (filled) {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_DRAWSQUARE | RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL);
  } else {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_DRAWSQUARE | RA8875_DCR_LINESQUTRI_START | RA8875_DCR_NOFILL);
  }

  // Wait until drawing status finished
  waitPoll(RA8875_DCR_ADDR, RA8875_DCR_LINESQUTRI_STATUS);
}
void TFT_RA8875::circleHelper(uint16_t x, uint16_t y, uint16_t r, int16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  // Setting Center Coordinate
  writeReg(RA8875_DCHR0_ADDR, x);
  writeReg(RA8875_DCHR1_ADDR, x >> 8);
  writeReg(RA8875_DCVR0_ADDR, y);
  writeReg(RA8875_DCVR1_ADDR, y >> 8);

  // Setting Circle Radius
  writeReg(RA8875_DCRR_ADDR, r);

  // Setting Color
  foregroundColor(color);

  // Draw Circle
  if (filled) {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_CIRCLE_START | RA8875_ELLIPSE_NOFILL);
  }

  // Wait until finished drawing
  waitPoll(RA8875_DCR_ADDR, RA8875_DCR_CIRCLE_STATUS);
}
void TFT_RA8875::ellipseHelper(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, int16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);

  // Draw Center of Ellipse
  writeReg(RA8875_DEHR0_ADDR, x);
  writeReg(RA8875_DEHR1_ADDR, x >> 8);
  writeReg(RA8875_DEVR0_ADDR, y);
  writeReg(RA8875_DEVR1_ADDR, y >> 8);

  // Set Long and Short Axis
  writeReg(RA8875_ELL_A0_ADDR, long_axis);
  writeReg(RA8875_ELL_A1_ADDR, long_axis >> 8);
  writeReg(RA8875_ELL_B0_ADDR, short_axis);
  writeReg(RA8875_ELL_B1_ADDR, short_axis >> 8);

  // Set Color
  foregroundColor(color);

  // Draw Ellipse
  if (filled) {
    writeReg(RA8875_ELLIPSE_ADDR, RA8875_ELLIPSE_DRAW | RA8875_ELLIPSE_FILL | RA8875_ELLIPSE_START);
  } else {
    writeReg(RA8875_ELLIPSE_ADDR, RA8875_ELLIPSE_DRAW | RA8875_ELLIPSE_NOFILL | RA8875_ELLIPSE_START);
  }

  // Wait until finished drawing
  waitPoll(RA8875_ELLIPSE_ADDR, RA8875_ELLIPSE_STATUS);
}
void TFT_RA8875::curveHelper(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, uint8_t curvePart, int16_t color, bool filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  curvePart = (curvePart + _rotation) % 4;

  // Draw Center of Curve
  writeReg(RA8875_DEHR0_ADDR, x);
  writeReg(RA8875_DEHR1_ADDR, x >> 8);
  writeReg(RA8875_DEVR0_ADDR, y);
  writeReg(RA8875_DEVR1_ADDR, y >> 8);

  // Set Long and Short Axis
  writeReg(RA8875_ELL_A0_ADDR, long_axis);
  writeReg(RA8875_ELL_A1_ADDR, long_axis >> 8);
  writeReg(RA8875_ELL_B0_ADDR, short_axis);
  writeReg(RA8875_ELL_B1_ADDR, short_axis >> 8);

  // Set Color
  foregroundColor(color);

  // Draw Curve
  if (filled) {
    writeReg(RA8875_ELLIPSE_ADDR, (RA8875_ELLIPSE_CURVE_DRAW | RA8875_ELLIPSE_DRAW | RA8875_ELLIPSE_START | RA8875_ELLIPSE_FILL) | (curvePart & 0x03));
  } else {
    writeReg(RA8875_ELLIPSE_ADDR, (RA8875_ELLIPSE_CURVE_DRAW | RA8875_ELLIPSE_DRAW | RA8875_ELLIPSE_START | RA8875_ELLIPSE_NOFILL) | (curvePart & 0x03));
  }

  // Wait until finished drawing
  waitPoll(RA8875_ELLIPSE_ADDR, RA8875_ELLIPSE_STATUS);
}
void TFT_RA8875::triangleHelper(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, int16_t color, bool filled) {
  x0 = applyRotationX(x0);
  y0 = applyRotationY(y0);
  x1 = applyRotationX(x1);
  y1 = applyRotationY(y1);
  x2 = applyRotationX(x2);
  y2 = applyRotationY(y2);

  // Point 0
  writeReg(RA8875_DLHSR0_ADDR, x0);
  writeReg(RA8875_DLHSR1_ADDR, x0 >> 8);
  writeReg(RA8875_DLVSR0_ADDR, y0);
  writeReg(RA8875_DLVSR1_ADDR, y0 >> 8);

  // Point 1
  writeReg(RA8875_DLHER0_ADDR, x1);
  writeReg(RA8875_DLHER1_ADDR, x1 >> 8);
  writeReg(RA8875_DLVER0_ADDR, y1);
  writeReg(RA8875_DLVER1_ADDR, y1 >> 8);

  // Point 2
  writeReg(RA8875_DTPH0_ADDR, x2);
  writeReg(RA8875_DTPH1_ADDR, x2 >> 8);
  writeReg(RA8875_DTPV0_ADDR, y2);
  writeReg(RA8875_DTPV1_ADDR, y2 >> 8);

  // Set Color
  foregroundColor(color);

  // Draw Triangle
  if(filled) {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_DRAWTRIANGLE | RA8875_DCR_FILL |RA8875_DCR_LINESQUTRI_START);
  } else {
    writeReg(RA8875_DCR_ADDR, RA8875_DCR_DRAWTRIANGLE | RA8875_DCR_NOFILL |RA8875_DCR_LINESQUTRI_START);
  }

  // Wait until finished drawing
  waitPoll(RA8875_DCR_ADDR, RA8875_DCR_LINESQUTRI_STATUS);
}
void TFT_RA8875::roundedRectHelper(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, int16_t color, int16_t filled) {
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationX(w + x - 1);
  h = applyRotationY(h + y - 1);

  // if (x > w)
  //   swap(x, w);
  //   // w = x / w;
  // if (y > h)
  //   swap(y, h);
  //   // h = y / h;

  // Start Point
  writeReg(RA8875_DLHSR0_ADDR, x);
  writeReg(RA8875_DLHSR1_ADDR, x >> 8);
  writeReg(RA8875_DLVSR0_ADDR, y);
  writeReg(RA8875_DLVSR1_ADDR, y >> 8);

  // End Point
  writeReg(RA8875_DLHER0_ADDR, w);
  writeReg(RA8875_DLHER1_ADDR, w >> 8);
  writeReg(RA8875_DLVER0_ADDR, h);
  writeReg(RA8875_DLVER1_ADDR, h >> 8);

  // Circle Corner
  writeReg(RA8875_ELL_A0_ADDR, r);
  writeReg(RA8875_ELL_A1_ADDR, r >> 8);
  writeReg(RA8875_ELL_B0_ADDR, r);
  writeReg(RA8875_ELL_B1_ADDR, r >> 8);

  // Set Color
  foregroundColor(color);

  // Draw Rounded Rectangle
  if (filled) {
    writeReg(RA8875_ELLIPSE_ADDR, RA8875_CIRCLESQUARE_DRAW | RA8875_ELLIPSE_FILL | RA8875_ELLIPSE_START);
  } else {
    writeReg(RA8875_ELLIPSE_ADDR, RA8875_CIRCLESQUARE_DRAW | RA8875_ELLIPSE_NOFILL | RA8875_ELLIPSE_START);
  }

  // Wait until Finished Drawing
  waitPoll(RA8875_ELLIPSE_ADDR, RA8875_ELLIPSE_STATUS);
}

/**********************************************************************************                       
                              Drawing Shapes     
    @param X            X-Axis start point coordinate
    @param Y            Y-Axis start point coordinate
    @param W            Width of rectangle shape
    @param h            Height of rectangle shape
    @param r            Radius of circle
    @param color        Color of the shape
    @param long_axis    Long axis of Ellipse shape
    @param short-axis   Short axis of Ellipse shape
    @param curvePart    Curve Part of the circle selection
    @param  x0          X-Axis first point coordinate triangle
    @param  y0          Y-Axis first point coordinate triangle
    @param  x1          X-Axis second point coordinate triangle
    @param  y1          Y-Axis second point coordinate triangle
    @param  x2          X-Axis third point coordinate triangle
    @param  y2          Y-Axis third point coordinate triangle          
***********************************************************************************/
void TFT_RA8875::drawEmptyRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int16_t color){rectHelper(x, y, w, h, color, false);}
void TFT_RA8875::drawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, int16_t color){rectHelper(x, y, w, h, color, true);}
void TFT_RA8875::drawFilledCircle(uint16_t x, uint16_t y, uint16_t r, int16_t color){circleHelper(x, y, r, color, true);}
void TFT_RA8875::drawEmptyCircle(uint16_t x, uint16_t y, uint16_t r, int16_t color){circleHelper(x, y, r, color, false);}
void TFT_RA8875::drawFilledEllipse(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, int16_t color){ellipseHelper(x, y, long_axis, short_axis, color, true);}
void TFT_RA8875::drawEmptyEllipse(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, int16_t color){ellipseHelper(x, y, long_axis, short_axis, color, false);}
void TFT_RA8875::drawFilledCurve(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, uint8_t curvePart, int16_t color){
  curveHelper(x, y, long_axis, short_axis, curvePart, color, true);}
void TFT_RA8875::drawEmptyCurve(uint16_t x, uint16_t y, uint16_t long_axis, uint16_t short_axis, uint8_t curvePart, int16_t color){
  curveHelper(x, y, long_axis, short_axis, curvePart, color, false);}
void TFT_RA8875::drawLine(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, int16_t color) {
  x  = applyRotationX(x);
  y  = applyRotationY(y);
  x1 = applyRotationX(x1);
  y1 = applyRotationY(y1);

  // Start Point
  writeReg(RA8875_DLHSR0_ADDR, x);
  writeReg(RA8875_DLHSR1_ADDR, x >> 8);
  writeReg(RA8875_DLVSR0_ADDR, y);
  writeReg(RA8875_DLVSR1_ADDR, y >> 8);

  // End Point
  writeReg(RA8875_DLHER0_ADDR, x1);
  writeReg(RA8875_DLHER1_ADDR, x1 >> 8);
  writeReg(RA8875_DLVER0_ADDR, y1);
  writeReg(RA8875_DLVER1_ADDR, y1 >> 8);

  // Set Color & Start Drawing
  foregroundColor(color);
  writeReg(RA8875_DCR_ADDR, RA8875_DCR_DRAWLINE | RA8875_DCR_LINESQUTRI_START);

  // Wait until finished drawing
  waitPoll(RA8875_DCR_ADDR, RA8875_DCR_LINESQUTRI_STATUS);
}
void TFT_RA8875::drawFilledTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, int16_t color){triangleHelper(x0, y0, x1, y1, x2, y2, color, true);}
void TFT_RA8875::drawEmptyTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, int16_t color){triangleHelper(x0, y0, x1, y1, x2, y2, color, false);}
void TFT_RA8875::drawFilledRoundedRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, int16_t color){roundedRectHelper(x, y, w, h, r, color, true);}
void TFT_RA8875::drawEmptyRoundedRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t r, int16_t color){roundedRectHelper(x, y, w, h, r, color, false);}

/**********************************************************************************                       
                      BMP Helper             
***********************************************************************************/
uint16_t TFT_RA8875::read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t TFT_RA8875::read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

uint16_t TFT_RA8875::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

byte TFT_RA8875::decToBcd(byte val){
  // Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}

/**********************************************************************************                       
                      Drawing BMP from SD Card
    @param filename   Related BMP filename
    @param x          X-Axis starting position
    @param y          Y-Axis starting position               
***********************************************************************************/
void TFT_RA8875::bmpDraw(const char *filename, int x, int y)
{
  File     bmpFile;
  int      bmpget_width, bmpget_height;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpget_width; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;

  if((x >= get_width()) || (y >= get_height())) return;
  printf("Loading image '%s'\n", filename);

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == false) {
    printf("File not found");
    return;
  }
  
  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    printf("File size: %d\n", read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    printf("Image Offset: %d\n", bmpImageoffset);

    // Read DIB header
    printf("Header size: %d\n", read32(bmpFile));
    bmpget_width  = read32(bmpFile);
    bmpget_height = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      printf("Bit Depth: %d\n", bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        printf("Image size: %dx%d\n", bmpget_width, bmpget_height);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpget_width * 3 + 3) & ~3;

        // If bmpget_height is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpget_height < 0) {
          bmpget_height = -bmpget_height;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpget_width;
        h = bmpget_height;
        if((x+w-1) >= get_width())  w = get_width()  - x;
        if((y+h-1) >= get_height()) h = get_height() - y;

        // Set TFT address window to clipped image bounds
      

        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpget_height - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
          pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
                lcdidx = 0;
              }

              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx] = color565(r,g,b);
            drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
          } // end pixel
        } // end scanline

        // Write any remaining data to LCD
        if(lcdidx > 0) {
          drawPixel(col+x, row+y, lcdbuffer[lcdidx]);
        }
        printf("Loaded in %d ms\n", (millis() - startTime));
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) printf("BMP format not recognized.\n");
}

/**********************************************************************************                       
                  Drawing Fast BMP from SD Card
    @param filename   Related BMP filename
    @param x          X-Axis starting position
    @param y          Y-Axis starting position               
***********************************************************************************/
void TFT_RA8875::bmpDrawFast(const char *filename, int x, int y) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel in buffer (R+G+B per pixel)
  uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col, xpos, ypos;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
  uint8_t  lcdidx = 0;

  if((x >= get_width()) || (y >= get_height())) return;

  printf("Loading image '%s'\n", filename);

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == false) {
    printf("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    printf("File size: %d\n", read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    printf("Image Offset: %d\n");

    // Read DIB header
    printf("Header size: %d\n", read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      printf("Bit Depth: %d\n", bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        printf("Image size: %dx%d\n", bmpWidth, bmpHeight);
  
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= get_width())  w = get_width()  - x;
        if((y+h-1) >= get_height()) h = get_height() - y;

        // Set TFT address window to clipped image bounds
        ypos = y;
        for (row=0; row<h; row++) { // For each scanline...
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;

          if (bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
          xpos = x;
          for (col=0; col<w; col++) { // For each column...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              // Push LCD buffer to the display first
              if(lcdidx > 0) {
                drawPixels(xpos, ypos, lcdbuffer, lcdidx);
                xpos += lcdidx;
                lcdidx = 0;
              }

              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            lcdbuffer[lcdidx++] = color565(r,g,b);
            if (lcdidx >= sizeof(lcdbuffer) || (xpos - x + lcdidx) >= w) {
              drawPixels(xpos, ypos, lcdbuffer, lcdidx);
              lcdidx = 0;
              xpos += lcdidx;
            }
          } // end pixel
            ypos++;
        } // end scanline

        // Write any remaining data to LCD
        if(lcdidx > 0) {
          drawPixels(xpos, ypos, lcdbuffer, lcdidx);
          xpos += lcdidx;
        }

        printf("Loaded in %d ms\n", millis() - startTime);
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) printf("BMP format not recognized.");

}

/**********************************************************************************                       
                         Background Color Window 
    @param color  Color for the window background              
***********************************************************************************/
void TFT_RA8875::setBackgroundWindow(uint16_t color) {
  _window_color = color;
  drawFilledRectangle(0, 0, screen_width - 1, screen_height - 1, color);
}

/**********************************************************************************                       
                        Select Layer to Read/Write 
                        --------------------------              
      "This function only supports on resolution <= 480x400 or color depth = 8bpp.
        Otherwise, always using layer 1"
***********************************************************************************/
// void TFT_RA8875::selectLayer(uint8_t l) {writeReg(RA8875_MWCR1_ADDR, l);}

/**********************************************************************************                       
                         Window Scroll Function
    @param X            X-Axis start coordinate window to scrolled
    @param Y            Y-Axis start coordinate window to scrolled
    @param W            Width of the window to be scrolled        
    @param H            Heigh of the window to be scrolled
    @param scroll_mode  Scrolling selection         
    @param x_offset     X-Axis Scrolling distance
    @param y_offser     Y-Axis Scrolling distance                       
***********************************************************************************/
void TFT_RA8875::setScrollWindowRange(uint16_t x, uint16_t y, uint16_t w, uint16_t h,  uint8_t scroll_mode) {
  // Scroll Start Point of Window
  writeReg(RA8875_HSSW0_ADDR, x);
  writeReg(RA8875_HSSW1_ADDR, x >> 8);
  writeReg(RA8875_VSSW0_ADDR, y);
  writeReg(RA8875_VSSW1_ADDR, y >> 8);

  // Scroll End Point of Window
  writeReg(RA8875_HESW0_ADDR, (w + x));
  writeReg(RA8875_HESW1_ADDR, (w + x) >> 8);
  writeReg(RA8875_VESW0_ADDR, (h + y));
  writeReg(RA8875_VESW1_ADDR, (h + y) >> 8);

  // Layer Config
  writeReg(RA8875_LTPR0_ADDR, scroll_mode);
}

void TFT_RA8875::scrollWindowX(uint16_t x_offset)  {
  writeReg(RA8875_HOFS0_ADDR, x_offset);
  writeReg(RA8875_HOFS1_ADDR, x_offset >> 8);
}

void TFT_RA8875::scrollWindowY(uint16_t y_offset)  {
  writeReg(RA8875_VOFS0_ADDR, y_offset);
  writeReg(RA8875_VOFS1_ADDR, y_offset >> 8);
}

/**********************************************************************************                       
                        Graphic/Text Mode Selection               
***********************************************************************************/
void TFT_RA8875::enableGraphicsMode(void) {
  writeCommand(RA8875_MWCR0_ADDR);
  uint8_t temp = readData();
  temp &= ~RA8875_MWCR0_TEXT_MODE;
  writeData(temp);
  // writeReg(RA8875_MWCR0_ADDR, RA8875_MWCR0_GRAPHIC_MODE);
}
void TFT_RA8875::enableTextMode(void) {
  // Set text mode
  writeCommand(RA8875_MWCR0_ADDR);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_TEXT_MODE;
  writeData(temp);
  // writeReg(RA8875_MWCR0_TEXT_MODE);

  // Select internal ROM Font
  writeCommand(RA8875_FNCR0_ADDR);
  temp = readData();
  temp &= ~((1 << 7) | (1 << 5));
  writeData(temp);
}

/**********************************************************************************                       
                        Graphic Cursor Setting
    @param X          X-Axis start coordinate
    @param Y          Y-Axis start coordinate
    @param color1     GCC0 Color
    @param color2     GCC1 Color
    @param cursorBit  Cursor bit selection
***********************************************************************************/
void TFT_RA8875::drawGraphicRAM(uint16_t x, uint16_t y, int16_t color)  {
  x = applyRotationX(x);
  y = applyRotationY(y);

  // Graphic cursor position
  writeReg(RA8875_GCHP0_ADDR, x);
  writeReg(RA8875_GCHP1_ADDR, x >> 8);
  writeReg(RA8875_GCVP0_ADDR, y);
  writeReg(RA8875_GCVP1_ADDR, y >> 8);

  // Draw Colors
  writeCommand(RA8875_MRWC_ADDR);
  digitalWrite(RA8875_CS, LOW);
  fspi->transfer(RA8875_DATAWRITE);
  for(int i=384000; i>0; i--) {
    fspi->transfer16(color);
  }
  digitalWrite(RA8875_CS, HIGH);
}

/**********************************************************************************                       
                            Text Cursor Setting
    @param X    X-Axis text position
    @param Y    Y-Axis text position          
***********************************************************************************/
void TFT_RA8875::setTextCoordinate(uint16_t x, uint16_t y)  {
  // Setting horizontal & vertical Position
  writeReg(RA8875_F_CURXL_ADDR, x);
  writeReg(RA8875_F_CURXH_ADDR, x >> 8);
  writeReg(RA8875_F_CURYL_ADDR, y);
  writeReg(RA8875_F_CURYH_ADDR, y >> 8);
}    

/**********************************************************************************                       
                          Rotate Text 90 Degree      
***********************************************************************************/
void TFT_RA8875::verticalText(uint8_t vdir, uint8_t hdir) {
  writeReg(RA8875_FNCR1_ADDR, RA8875_FNCR1_EN_ROTATION);
  writeReg(RA8875_DPCR_ADDR, (vdir | hdir));
}
void TFT_RA8875::horizontalText(uint8_t vdir, uint8_t hdir) {
  writeReg(RA8875_FNCR1_ADDR, RA8875_FNCR1_DIS_ROTATION);
  writeReg(RA8875_DPCR_ADDR, (vdir | hdir));
}

/**********************************************************************************                       
                            Text Color Setting
    @param forecolor      Text foreground color
    @param bgcolor        Text background color
    @param transparent    Text background transparency          
***********************************************************************************/
void TFT_RA8875::setTextColor(int16_t foreColor, int16_t bgColor, bool transparent)  {
  foregroundColor(foreColor);
  backgroundColor(bgColor);

  // Make the font background transparent
  if (transparent)
    writeReg(RA8875_FNCR1_ADDR, RA8875_FNCR1_BG_TRANSPARENT);
  else {
    writeCommand(RA8875_FNCR1_ADDR);
    uint8_t temp = readData();
    temp &= ~(1 << 6);
    writeData(temp);
  }
}

/**********************************************************************************                       
                              Text Enlarge
    @param scale    0 = 1x Zoom
                    1 = 2x Zoom
                    2 = 3x Zoom
                    3 = 4x Zoom
***********************************************************************************/
void TFT_RA8875::textEnlarge(uint8_t scale) {
  scale = (scale > 3) ? 3: scale;
  writeCommand(RA8875_FNCR1_ADDR);
  uint8_t temp = readData();
  temp &= ~(0xF);       // Clear bits 0-3
  temp |= scale << 2;   // Set bit 2 & 3 (Horizontal)
  temp |= scale;        // Set bit 0 & 1 (Vertical)
  writeData(temp);
  _textScale = temp;
  delay(1);
}

/**********************************************************************************                       
                              Cursor Blink
    @param frame_rate     Frames Time (1-256) / (0x00-0xFF)
***********************************************************************************/
void TFT_RA8875::cursorBlink(uint8_t frame_rate)  {
  // Make cursor visible & blink
  writeCommand(RA8875_MWCR0_ADDR);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_CURSOR_VISIBLE | RA8875_MWCR0_CURSOR_EN_BLINK;
  writeData(temp);

  // Set frame rate
  frame_rate = (frame_rate > 255) ? 255 : frame_rate;
  writeReg(RA8875_BTCR_ADDR, frame_rate);
}

/**********************************************************************************                       
                          Write Text to Display
    @param fnt    Font Selection from Internal CGROM
***********************************************************************************/
void TFT_RA8875::selectInternalFont(uint8_t fnt)  {
  fnt = (fnt > 0x03) ? 0x03 : fnt;
  writeReg(RA8875_FNCR0_ADDR, RA8875_FNCR0_INTERNAL_CGROM | fnt);
}

/**********************************************************************************                       
                          Write Text to Display
    @param x              X-Axis start coordinate
    @param y              Y-Axis start coordiante
    @param len            Character to clear
***********************************************************************************/
void TFT_RA8875::clearText(uint16_t x, uint16_t y, uint16_t len, int16_t bgColor)  {
  enableTextMode();

  // Set Text coordinate
  setTextCoordinate(x, y);

  // Set Text color & Size
  setTextColor(bgColor, bgColor, false);

  // Write Text
  writeCommand(RA8875_MRWC_ADDR);
  for (int i=0; i<len; i++) {writeData('\0');}
}

/**********************************************************************************                       
                          Write Text to DDRAM
    @param x              X-Axis start coordinate
    @param y              Y-Axis start coordinate
    @param buffer         Character to write
    @param foreColor      Text foreground color
    @param bgColor        Text background color
    @param transparent    Text Background transparency
***********************************************************************************/
void TFT_RA8875::writeText(uint16_t x, uint16_t y, const char* buffer, int16_t foreColor, int16_t bgColor, uint8_t scale, bool transparent)  {
  enableTextMode();
  clearText(x, y, _previous_text, bgColor);

  // Set Text coordinate
  setTextCoordinate(x, y);

  // Set Text color & Size
  bgColor = (bgColor == NULL) ? _window_color : bgColor;
  setTextColor(foreColor, bgColor, transparent);
  textEnlarge(scale);

  // Write Text
  uint16_t len = strlen(buffer);
  _previous_text = len;
  writeCommand(RA8875_MRWC_ADDR);
  for (int i=0; i<len; i++) {writeData(buffer[i]);}
}

/**********************************************************************************                       
                        Character Upload to CGRAM
    @param dataBuffer        Data to write
    @param cgramAddr         Address of CGRAM to write (0-255)
***********************************************************************************/
void TFT_RA8875::uploadCustomCharacter(const uint8_t dataBuffer[], uint8_t cgramAddr) {
  // Create the font or symbol by selecting CGRAM font and setting address in CGRAM 
  enableGraphicsMode();
  uint8_t mwcr1_temp = readReg(RA8875_MWCR1_ADDR);
  writeReg(RA8875_CGSR_ADDR, cgramAddr);
  writeReg(RA8875_FNCR0_ADDR, readReg(RA8875_FNCR0_ADDR) & 0x7F);
  writeReg(RA8875_MWCR1_ADDR, RA8875_MWCR1_DEST_CGRAM);

  writeCommand(RA8875_MRWC_ADDR);
  for (int i=0; i<16; i++)  {
    writeData(dataBuffer[i]);
  }

  // Restore Cursor Register
  writeReg(RA8875_MWCR1_ADDR, mwcr1_temp);
}

/**********************************************************************************                       
                        Character Print to Display
    @param x                  X-Axis coordinate
    @param y                  Y-Axis coordinate
    @param cgramAddr          Address of CGRAM to write (0-255)
    @param wide               Data length -> 0 = 8x16, > 0 = 8xArraySize
    @param foreColor          Character foreground color
    @param bgColor            Character background color
***********************************************************************************/
void TFT_RA8875::printCustomCharacter(int16_t x, int16_t y, uint8_t cgramAddr, uint8_t wide, int16_t foreColor, int16_t bgColor, bool transparent)  {
  // Write the font or symbol to display RAM
  enableTextMode();
  setTextCoordinate(x, y);
  setTextColor(foreColor, bgColor, transparent);
  uint8_t fncr0_temp = readReg(RA8875_FNCR0_ADDR);
  writeReg(RA8875_FNCR0_ADDR, fncr0_temp | RA8875_FNCR0_SELECT_CGRAM);
  writeReg(RA8875_MWCR1_ADDR, RA8875_MWCR1_DEST_LAYER_1_2);
  
  writeCommand(RA8875_MRWC_ADDR);
  if(wide > 1)  {
    for (int i=1; i<=wide; i++)  {
      writeData(cgramAddr + i);
    }
  } else {
    writeData(cgramAddr);
  }
  // Restore Font Register
  writeReg(RA8875_FNCR0_ADDR, fncr0_temp);
}

/**********************************************************************************                       
              Enable Touch Screen In Auto Mode & Interrupt Enable
***********************************************************************************/
void TFT_RA8875::enableTouch(void)  {
  // Enable Touch Panel
  pinMode(RA8875_INT, INPUT);
  digitalWrite(RA8875_INT, HIGH);
#if defined(RA8875_800x480)
  writeReg(RA8875_TPCR0_ADDR, (RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK | RA8875_TPCR0_WAKEENABLE | RA8875_TPCR0_ADCCLK_DIV16));
#else
  writeReg(RA8875_TPCR0_ADDR, (RA8875_TPCR0_ENABLE | RA8875_TPCR0_WAIT_4096CLK | RA8875_TPCR0_WAKEENABLE | RA8875_TPCR0_ADCCLK_DIV4));
#endif
  // Auto mode
  writeReg(RA8875_TPCR1_ADDR, RA8875_TPCR1_AUTO | RA8875_TPCR1_DEBOUNCE);
  // Enable Touch in interrupt mode
  writeReg(RA8875_INTC1_ADDR, readReg(RA8875_INTC1_ADDR) | RA8875_INTC1_TP);
}

/**********************************************************************************                       
                          Check Touch Event State
***********************************************************************************/
boolean TFT_RA8875::isTouched(void) {
  if (readReg(RA8875_INTC2_ADDR) & RA8875_INTC2_TP)
    return true;
  return false;
}

/**********************************************************************************                       
                        Read Touch Event Coordinate
    @param x    X-Axis touched coordinate
    @param y    Y-Axis touched coordinate
***********************************************************************************/
boolean TFT_RA8875::readTouch(uint16_t* x, uint16_t* y) {
  uint16_t txH, tyH; 
  uint8_t txyL;
  float xScale = 1024.0F/screen_width;
  float yScale = 1024.0F/screen_height;

  spi_speed = CLK_SPI_READ;
  // Read high & low byte (10 Bit ADC Resolution)
  txH   = readReg(RA8875_TPXH_ADDR);  // MSB
  tyH   = readReg(RA8875_TPYH_ADDR);  // MSB
  txyL  = readReg(RA8875_TPXYL_ADDR);
  // txH   = (txH << 2) | (txyL & 0x0C);  // X low bit in bit 2-3 
  // tyH   = (tyH << 2) | (txyL & 0x03);  // Y low bit in bit 0-1
  txH   = (txH << 2) | (txyL & 0x03);  // X low bit in bit 2-3 
  tyH   = (tyH << 2) | ((txyL >> 2) & 0x03);  // Y low bit in bit 0-1

  // Return parameter
  *x = txH/xScale;
  *y = tyH/yScale;

  // Clear Touch Interrupt
  spi_speed = CLK_SPI_WRITE;
  writeReg(RA8875_INTC2_ADDR, RA8875_INTC2_TP);
  return true;   // If success
}

/**********************************************************************************                       
                            Brush Draw Mode
    @param color  Brush color
***********************************************************************************/
void TFT_RA8875::brushDraw(int16_t color) {
  uint8_t touchAvailable = readReg(RA8875_TPCR0_ADDR);
  uint16_t tx, ty;

  if (touchAvailable & 0x80)  {
    if(isTouched()) {
      readTouch(&tx, &ty);
      drawPixel(tx, ty, color);
    }
  } else {
    drawFilledRectangle(screen_width/4, screen_height/4, screen_width/2, screen_height/2, RA8875_RED);
    writeText(screen_width/3, screen_height/2, "Touch is disabled!", RA8875_WHITE, RA8875_RED, 1, true);
    delay(1000);
  }
}

/**********************************************************************************                       
                            Pencil Draw Mode
    @param color      Pencil Color
    @param thickness  Pencil Thickness
***********************************************************************************/
void TFT_RA8875::pencilDraw(int16_t color, uint8_t thickness) {
  uint8_t touchAvailable = readReg(RA8875_TPCR0_ADDR);
  uint16_t tx, ty;

  if (touchAvailable & 0x80)  {
    if(isTouched()) {
      readTouch(&tx, &ty);
      drawFilledCircle(tx, ty, thickness, color);
    }
  } else {
    drawFilledRectangle(screen_width/4, screen_height/4, screen_width/2, screen_height/2, RA8875_RED);
    writeText(screen_width/3, screen_height/2, "Touch is disabled!", RA8875_WHITE, RA8875_RED, 1, true);
    delay(1000);
  }
}

/**********************************************************************************                       
                            BTE Source Helper
    @param source_hor    Horizontal source address
    @param source_ver    Vertical source address
***********************************************************************************/
void TFT_RA8875::BTE_sourceHelper(int16_t source_hor, int16_t source_ver) {
  // Setting horizontal source position
  writeReg(RA8875_HSBE0_ADDR, (source_hor & 0xFF));
  writeReg(RA8875_HSBE1_ADDR, (source_hor >> 8) & 0x03);
  
  // Setting vertical source position
  writeReg(RA8875_VSBE0_ADDR, (source_ver & 0xFF));
  writeReg(RA8875_VSBE1_ADDR, (source_ver >> 8) & 0x03);
}

/**********************************************************************************                       
                          BTE Destination Helper
    @param dest_hor    Horizontal destination address
    @param dest_ver    Vertical destination address
***********************************************************************************/
void TFT_RA8875::BTE_destinationHelper(int16_t dest_hor, int16_t dest_ver)  {
  // Setting horizontal destination position
  writeReg(RA8875_HDBE0_ADDR, (dest_hor & 0xFF));
  writeReg(RA8875_HDBE1_ADDR, (dest_hor >> 8) & 0x03);
  
  // Setting vertical destination position
  writeReg(RA8875_VDBE0_ADDR, (dest_ver & 0xFF));
  writeReg(RA8875_VDBE1_ADDR, (dest_ver >> 8) & 0x03);
}

/**********************************************************************************                       
                          BTE Size Helper
    @param BTE_width     BTE Width size
    @param BTE_height    BTE Height size
***********************************************************************************/
void TFT_RA8875::BTE_sizeHelper(uint16_t BTE_width, uint16_t BTE_height)  {
  // Setting BTE Width
  writeReg(RA8875_BEWR0_ADDR, (BTE_width & 0xFF));
  writeReg(RA8875_BEWR1_ADDR, (BTE_width >> 8) & 0x03);
  
  // Setting BTE Height
  writeReg(RA8875_BEHR0_ADDR, (BTE_height & 0xFF));
  writeReg(RA8875_BEHR1_ADDR, (BTE_height >> 8) & 0x03);
}

/**********************************************************************************                       
                    BTE Raster Operation Code Helper
    @param code     ROP Code refers to datasheet
***********************************************************************************/
void TFT_RA8875::BTE_ROP_codeHelper(uint8_t code) {
  writeReg(RA8875_BECR1_ADDR, code);
}

/**********************************************************************************                       
                          BTE Size Helper
    @param dest_hor    Horizontal destination address
    @param dest_ver    Vertical destination address
    @param BTE_width     BTE Width size
    @param BTE_height    BTE Height size
***********************************************************************************/
void TFT_RA8875::BTEWriteROP(int16_t dest_hor, int16_t dest_ver, uint16_t BTE_width, uint16_t BTE_height, uint16_t* img, uint32_t num)  {
  _waitBusy(0x40);                            // Check if another BTE is still in progress
  BTE_destinationHelper(dest_hor, dest_ver);  // Set destination address
  BTE_sizeHelper(BTE_width, BTE_height);      // BTE size
  BTE_ROP_codeHelper(RA8875_ROP_S_EQUAL_D);   // ROP Code
  writeReg(RA8875_BECR0_ADDR, (RA8875_BECR0_ENABLE | RA8875_BECR0_DESTINATION_BLOCK)); // Enable BTE

  // Pixels direction - LRTD: Default, on Rotation: RLTD
  uint8_t dir = RA8875_MWCR0_LRTD;
  if (_rotation == 2) {
    dir = RA8875_MWCR0_RLTD;
  }
  writeReg(RA8875_MWCR0_ADDR, (readReg(RA8875_MWCR0_ADDR) & ~RA8875_MWCR0_DIRMASK) | dir);

  // Draw Pixels
  writeCommand(RA8875_MRWC_ADDR);
  digitalWrite(RA8875_CS, LOW);
  fspi->transfer(RA8875_DATAWRITE);
  while(num--) {
    fspi->transfer16(*img++);
  }
  digitalWrite(RA8875_CS, HIGH);
  _waitBusy(0x40);                            // Wait until BTE finished the transfer
}

/**********************************************************************************                       
                              DMA Helper
    @param address    DMA source starting address
***********************************************************************************/
void TFT_RA8875::startAddressDMA(uint32_t addr) {
  // DMA Source starting address
  writeReg(RA8875_SSAR0_ADDR, addr & 0xFF);
  writeReg(RA8875_SSAR1_ADDR, (addr >> 8) & 0xFF);
  writeReg(RA8875_SSAR2_ADDR, (addr >> 16) & 0xFF);
}

void TFT_RA8875::enableDMA(void)  {
  // // Setting Flash/rom as DMA Mode, 5 Bus (Fast mode), and Single Latch Mode (Non-bi directional)
  // writeReg(RA8875_SROC_ADDR, (RA8875_SROC_DMA_MODE | RA8875_SROC_5_BUS));
  
  writeReg(RA8875_SFCLR_ADDR, 0x00);
  writeReg(RA8875_SROC_ADDR, 0x87);

  // // Enable DMA Mode
  // writeReg(RA8875_SACS_ADDR, RA8875_SACS_MODE_DMA);
}

void TFT_RA8875::enableDMAInt(void)  {
  writeReg(RA8875_INTC1_ADDR, readReg(RA8875_INTC1_ADDR) | RA8875_INTC1_DMA);
}

/**************************************************************************
	@param res:  0x80(for most operations),
	            0x40(BTE wait), 
	            0x01(DMA wait)
**************************************************************************/
void TFT_RA8875::_waitBusy(uint8_t res) 
{
	uint8_t temp; 	
	unsigned long start = millis();//M.Sandercock
	do {
		if (res == 0x01) writeCommand(RA8875_DMACR_ADDR);//dma
		temp = readStatus();
		if ((millis() - start) > 10) return;
	} while ((temp & res) == res);
}

/**********************************************************************************                       
                            Continuous DMA
    @param x            X-Axis active window
    @param y            Y-Axis active window
    @param addr         DMA source starting address
    @param transfNum    DMA Transfer number
***********************************************************************************/
void TFT_RA8875::continuousDMA(uint16_t x, uint16_t y, uint32_t addr, int32_t transfNum)  {
  enableDMA();
  x = applyRotationX(x);
  y = applyRotationY(y);

  // Memory Write Cursor position
  graphicSetCoordinate(x, y);
  setActiveWindow();

  // Source starting address
  startAddressDMA(addr);
  
  // DMA transfer number of 19 bit
  writeReg(RA8875_BWR0_ADDR,  transfNum & 0xFF);           // DMA Transfer Number Reg0 (bit 7:0)
  writeReg(RA8875_BHR0_ADDR,  (transfNum >> 8) & 0xFF);    // DMA Transfer Number Reg1 (bit 15:8)
  writeReg(RA8875_SPWR0_ADDR, (transfNum >> 16) & 0x03);   // DMA Transfer Number Reg2 (bit 18:16)

  // Enable DMA Start
  uint8_t _dmaBusy = readReg(RA8875_DMACR_ADDR);
  if (!(_dmaBusy & 0x01))  {
    // writeReg(RA8875_DMACR_ADDR, (_dmaBusy | 0x01));
    writeData(_dmaBusy | 0x03);
  }

  // Wait until DMA finish
  _waitBusy(0x01);  
}

/**********************************************************************************                       
                            Block DMA
    @param x            X-Axis active window
    @param y            Y-Axis active window
    @param w            Width block
    @param h            Height block
    @param addr         DMA source starting address
    @param SPWR         Source picture width
***********************************************************************************/
void TFT_RA8875::blockDMA(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t addr, int16_t SPWR) {
  enableDMA();
  x = applyRotationX(x);
  y = applyRotationY(y);
  w = applyRotationY(w);
  h = applyRotationY(h);

  // Memory Write Cursor position
  graphicSetCoordinate(x, y);
  setActiveWindow();

  // DMA Block Width (10 bit)
  writeReg(RA8875_BWR0_ADDR, (w & 0xFF));           // DMA Block Width [7:0]
  writeReg(RA8875_BWR1_ADDR, (w >> 8) & 0x03);      // DMA Block Width [9:8]

  // DMA Block Height (10 bit)
  writeReg(RA8875_BHR0_ADDR, (h & 0xFF));           // DMA Block Height [7:0]
  writeReg(RA8875_BHR1_ADDR, (h >> 8) & 0x03);      // DMA Block Height [9:8]

  // Source Picture Width
  writeReg(RA8875_SPWR0_ADDR, (SPWR & 0xFF));       // DMA Source Picture Width [7:0]
  writeReg(RA8875_SPWR1_ADDR, (SPWR >> 8) & 0x03);  // DMA Source Picture Width [9:8]
  
  // Source starting address
  startAddressDMA(addr);

  // Set DMA Block mode
  writeReg(RA8875_DMACR_ADDR, 0x02);

  // Enable DMA Start
  uint8_t _dmaBusy = readReg(RA8875_DMACR_ADDR);
  if (!(_dmaBusy & 0x01))  {
    // writeReg(RA8875_DMACR_ADDR, (_dmaBusy | 0x01));
    writeData(_dmaBusy | 0x03);
  }

  // Wait until DMA finish
  _waitBusy(0x01);
}