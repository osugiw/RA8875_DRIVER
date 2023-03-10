/*
  Author: Sugiarto Wibowo
  Email: osugiartow@gmail.com
  Github: https://github.com/osugiw
  File: RA8875_FREERTOS.ino

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
          - Implement FREERTOS for receiveing input from display, while displaying the input drawing to the display
          - Please be careful while configuring FREERTOS, otherwise you will experience failed boot
*/

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#include <SPI.h>
#include <RA8875_DRIVER.h>
#include <string.h>

// FreeRTOS Setting with Binary Semaphore
static SemaphoreHandle_t display_sem; 

// Timer
const unsigned long eventInterval = 1000;
unsigned long previousTime = 0;

// Pin Interface RA8875 (SPI Bus cannot be combined with other, refer to the product information.)
#define RA8875_WAIT    3    // Busy Pin
#define RA8875_SCLK    2    // RA8875 SCLK
#define RA8875_MISO    8    // RA8875 MISO
#define RA8875_MOSI    9    // RA8875 MOSI
#define RA8875_CS      10    // RA8875 Chip Selector
#define RA8875_INT     18   // RA8875 Touch Interrupt
#define RA8875_RST     19   // RA8875 Reset Panel
TFT_RA8875 tft(RA8875_WAIT, RA8875_SCLK, RA8875_MISO, RA8875_MOSI, RA8875_CS, RA8875_INT, RA8875_RST);

String dummyLevel;

void setup() {
  Serial.begin(115200);
  delay(200);
  
  // Init display first - default screen is 800x480, change in the RA8875_SPI.h
  if (!tft.init()) {
    printf("RA8875 not found!\n");
    while(1);
  }
  printf("RA8875 initialized!\n");

  tft.displayOn();
  tft.enableGPIOX();
  tft.enablePWM1(RA8875_PWM_CLK_DIV4096);
  tft.setPWM1DutyCycle(255);
  // tft.setScrollWindowRange(0, 0, 800, 480, RA8875_LTPR0_SCROLL_SIMUL);
  tft.setBackgroundWindow(RA8875_YELLOW); 
  tft.enableTouch();

  display_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(display_sem);
  BaseType_t taskDisplayBattery = xTaskCreatePinnedToCore(
                                  displayBattery,
                                  "Display Battery", 
                                  4096, 
                                  NULL, 
                                  1, 
                                  NULL, 
                                  app_cpu);
  BaseType_t taskDrawMode       = xTaskCreatePinnedToCore(
                                  drawBrush,
                                  "Draw Mode", 
                                  4096, 
                                  NULL, 
                                  1, 
                                  NULL, 
                                  app_cpu);
}

void loop() {
  // uint16_t tx, ty;
  // if(tft.isTouched()) {
  //   tft.readTouch(&tx, &ty);
  //   printf("x, y (%d, %d)\n", tx, ty);
  // }
  // tft.pencilDraw(RA8875_BLUE, 2);
  // tft.brushDraw(RA8875_BLACK);

  dummyLevel = String(15);
  delay(100);
  dummyLevel = String(25);
  delay(100);
}

void drawBrush(void* parameter) {
  for(;;) {
    unsigned long currentTime = millis();
    
    if((xSemaphoreTake(display_sem, portMAX_DELAY) == pdPASS) || (currentTime - previousTime >= eventInterval))  {
      tft.brushDraw(RA8875_BLACK);
      previousTime = currentTime;
      xSemaphoreGive(display_sem);
      // vTaskDelay(10/portTICK_PERIOD_MS);
    }
  }
}

void displayBattery(void* parameter)  {
  for(;;) {
    if (xSemaphoreTake(display_sem, portMAX_DELAY) == pdPASS){
      battery_25(tft.get_width()-90, 10, RA8875_RED, dummyLevel, RA8875_WHITE);
      xSemaphoreGive(display_sem);
      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
  }
}

void battery_100(int16_t x, int16_t y, int16_t color, String lev, int16_t bgColor)  {
  
  const uint8_t b1[16]  = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF};
  const uint8_t b2[16]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF};
  const uint8_t b3[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lev = lev + '%';

  tft.drawFilledRectangle(0, 0, tft.get_width(), 50, bgColor);
  tft.uploadCustomCharacter(b1, 1);
  tft.uploadCustomCharacter(b2, 2);
  tft.uploadCustomCharacter(b3, 3);
  tft.printCustomCharacter(x, y, 1, 1, color, NULL, true);        // Upper left  
  tft.printCustomCharacter(x+8, y, 2, 1, color, NULL, true);      // Upper right 
  tft.printCustomCharacter(x, y+16, 3, 1, color, NULL, true);     // Bottom left 
  tft.printCustomCharacter(x+8, y+16, 3, 1, color, NULL, true);   // Bottom right
  tft.writeText(x+20, y, lev.c_str(), RA8875_BLACK, bgColor, 1, true);
}

void battery_75(int16_t x, int16_t y, int16_t color, String lev, int16_t bgColor)  {
  const uint8_t b1[16]  = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF};
  const uint8_t b2[16]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF};
  const uint8_t b3[16]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lev = lev + '%';
  
  tft.drawFilledRectangle(0, 0, tft.get_width(), 50, bgColor);
  tft.uploadCustomCharacter(b1, 1);
  tft.uploadCustomCharacter(b2, 2);
  tft.uploadCustomCharacter(b3, 3);
  tft.printCustomCharacter(x, y, 1, 1, color, NULL, true);      // Upper left  
  tft.printCustomCharacter(x+8, y, 2, 1, color, NULL, true);    // Upper right 
  tft.printCustomCharacter(x, y+16, 3, 1, color, NULL, true);   // Bottom left 
  tft.printCustomCharacter(x+8, y+16, 3, 1, color, NULL, true); // Bottom right
  tft.writeText(x+20, y, lev.c_str(), RA8875_BLACK, bgColor, 1, true);
}

void battery_50(int16_t x, int16_t y, int16_t color, String lev, int16_t bgColor)  {
  const uint8_t b1[16]  = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0};
  const uint8_t b2[16]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
  const uint8_t b3[16]  = {0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t b4[16]  = {0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lev = lev + '%';
  
  tft.drawFilledRectangle(0, 0, tft.get_width(), 50, bgColor);
  tft.uploadCustomCharacter(b1, 1);
  tft.uploadCustomCharacter(b2, 2);
  tft.uploadCustomCharacter(b3, 3);
  tft.uploadCustomCharacter(b4, 4);
  tft.printCustomCharacter(x, y, 1, 1, color, NULL, true);      // Upper left  
  tft.printCustomCharacter(x+8, y, 2, 1, color, NULL, true);    // Upper right 
  tft.printCustomCharacter(x, y+16, 3, 1, color, NULL, true);   // Bottom left 
  tft.printCustomCharacter(x+8, y+16, 4, 1, color, NULL, true); // Bottom right
  tft.writeText(x+20, y, lev.c_str(), RA8875_BLACK, bgColor, 1, true);
}

void battery_25(int16_t x, int16_t y, int16_t color, String lev, int16_t bgColor)  {
  const uint8_t b1[16]  = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0};
  const uint8_t b2[16]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F};
  const uint8_t b3[16]  = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t b4[16]  = {0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  lev = lev + '%';
  
  tft.drawFilledRectangle(0, 0, tft.get_width(), 50, bgColor);
  tft.uploadCustomCharacter(b1, 1);
  tft.uploadCustomCharacter(b2, 2);
  tft.uploadCustomCharacter(b3, 3);
  tft.uploadCustomCharacter(b4, 4);
  tft.printCustomCharacter(x, y, 1, 1, color, NULL, true);      // Upper left  
  tft.printCustomCharacter(x+8, y, 2, 1, color, NULL, true);    // Upper right 
  tft.printCustomCharacter(x, y+16, 3, 1, color, NULL, true);   // Bottom left 
  tft.printCustomCharacter(x+8, y+16, 4, 1, color, NULL, true); // Bottom right
  tft.writeText(x+20, y, lev.c_str(), RA8875_BLACK, bgColor, 1, true);
}

void scrollEx()
{
  static int scroll = 0;
  static int dir = 1;

  tft.scrollWindowX(scroll);
  tft.scrollWindowY(scroll);
  scroll += dir;
  if (scroll >= 800) {
    dir -= 1;
  } else if (scroll <= 0) {
    dir = 1;
  }
  delay(10);
}
