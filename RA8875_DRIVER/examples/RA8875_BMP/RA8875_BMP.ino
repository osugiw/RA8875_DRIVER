/*
  Author: Sugiarto Wibowo
  Email: osugiartow@gmail.com
  Github: https://github.com/osugiw
  File: RA8875_BMP.ino

  Some codes are implemented from https://github.com/adafruit/Adafruit_RA8875
  WARNING:
          - This program has successfully implemented on ESP32-C312F, with display resolution of 800x480
          - If your project plan to use SD Card module, please use separate BUS with RA8875's BUS, because the RA8875 driver doesn't support Tri-State 
          - Some of functions are still in progress or have not found the solution yet. Those functions are DMA, BTE, and FastDrawing
          - If you plan to use with LVGL please use example from my repository, but be aware that the drawing process is not optimized yet.
            As far I know, this because we could not use PIO or I2C in Adafruit_RA8875 Module. Actually the datasheet says I2C and PIO can be used,
            however you need to modify the hardware.
          - Change pins assignment of the RA8875 and SD card driver
  Usage:
          - Draw image from BMP Format
*/

// Only support 24 bit depth image
#include <SPI.h>
#include <RA8875_DRIVER.h>
#include <string.h>

// Pin Interface RA8875 (SPI Bus cannot be combined with other, refer to the product information.)
#define RA8875_WAIT    3    // Busy Pin
#define RA8875_SCLK    2    // RA8875 SCLK
#define RA8875_MISO    8    // RA8875 MISO
#define RA8875_MOSI    9    // RA8875 MOSI
#define RA8875_CS      10    // RA8875 Chip Selector
#define RA8875_INT     18   // RA8875 Touch Interrupt
#define RA8875_RST     19   // RA8875 Reset Panel
TFT_RA8875 tft(RA8875_WAIT, RA8875_SCLK, RA8875_MISO, RA8875_MOSI, RA8875_CS, RA8875_INT, RA8875_RST);

// SD Card SPI Bus
#define SD_SCLK        4     // SD Card clock bus
#define SD_MISO        5     // SD Card MISO
#define SD_MOSI        6    // SD Card MOSI
#define SD_CS          7     // SD Card selector

void setup() {
  Serial.begin(115200);
  delay(200);

  // Init display first - default screen is 800x480, change in the RA8875_SPI.h
  if (!tft.init()) {
    printf("RA8875 not found!\n");
    while(1);
  }
  printf("1. RA8875 initialized!\n");

  tft.displayOn();
  tft.enableGPIOX();
  tft.enablePWM1(RA8875_PWM_CLK_DIV4096);
  tft.setPWM1DutyCycle(255);
  tft.setBackgroundWindow(RA8875_YELLOW); 
  tft.enableTouch();

  if(!SD.begin(SD_CS))
  {
    printf("SD Card initialization failed!\n");
    while(1);
  }
  printf("2. SD Card initialized\n");
  tft.bmpDraw("/parrot.bmp", 0, tft.get_height()/3);
}

void loop() {
}
