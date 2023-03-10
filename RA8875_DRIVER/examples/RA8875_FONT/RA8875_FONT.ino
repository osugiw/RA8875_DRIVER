/*
  Author: Sugiarto Wibowo
  Email: osugiartow@gmail.com
  Github: https://github.com/osugiw
  File: RA8875_FONT.ino

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
          - Display font
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

  // Change window background
  tft.setBackgroundWindow(RA8875_CYAN);
  delay(500);

  // Display Text
  char msg[] = "Hello My Name is Sugiarto";
  tft.writeText(200, 100, msg, RA8875_RED, RA8875_BLUE, 1, true);
  tft.cursorBlink(32);

}

void loop() {
  String ser;
  // Wrtie data from Serial Monitor to RA8875 Display
  while (Serial.available() > 0)  {
    ser = Serial.readStringUntil('\n');
    printf("%s\n", ser);
    tft.writeText(300, 150, ser.c_str(), RA8875_RED, RA8875_CYAN, 1, true);
    ser = '\0';
    delay(10);
  }
}
