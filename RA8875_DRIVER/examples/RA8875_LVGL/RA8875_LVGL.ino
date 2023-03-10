/*
  Author: Sugiarto Wibowo
  Email: osugiartow@gmail.com
  Github: https://github.com/osugiw
  File: RA8875_LVGL.ino

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
          - Draw drawing buffer from LVGL to RA8875 display
*/

#include <lvgl.h>
#include <lv_demo.h>
#include <RA8875_DRIVER.h>
#include "src/3D_PRINTER/ui.h"

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

/*Change to your screen resolution*/
#define screenWidth       800
#define screenHeight      480
#define DISPLAY_BUFFER    (screenWidth * 40)
static lv_disp_draw_buf_t draw_buf;
// static lv_color_t buf1[DISPLAY_BUFFER];
// static lv_color_t buf2[DISPLAY_BUFFER];
static lv_color_t* buf1 = (lv_color_t*)(heap_caps_malloc(DISPLAY_BUFFER * sizeof(lv_color_t), MALLOC_CAP_DMA));
static lv_color_t* buf2 = (lv_color_t*)(heap_caps_malloc(DISPLAY_BUFFER * sizeof(lv_color_t), MALLOC_CAP_DMA));

void setup() {
  Serial.begin(115200);
  delay(200);

  // Init display first - default screen is 800x480, change in the RA8875_SPI.h
  if (!tft.init()) {
    printf("RA8875 not found!\n");
    while(1);
  }  
  tft.displayOn();
  tft.enableGPIOX();
  tft.enablePWM1(RA8875_PWM_CLK_DIV4096);
  tft.setPWM1DutyCycle(255);
  tft.enableTouch();

   if(!SD.begin(SD_CS))
  {
    printf("SD Card initialization failed!\n");
    while(1);
  }
  printf("SD Card initialized\n");

  lv_init();
#if LV_USE_LOG != 0
  lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif
  lv_disp_draw_buf_init( &draw_buf, buf1, buf2, DISPLAY_BUFFER);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  // /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );

  ui_init();
  // lv_example_bmp();
  // lv_demo_widgets();
}

void loop() {
  lv_timer_handler();
  delay(5);
}

void lv_example_bmp(void)
{
  lv_obj_t * wp;
  wp = lv_img_create(lv_scr_act());
  lv_img_set_src(wp, "parrot.bmp");
}

/* Display buffer to TFT */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    int32_t row_count = (area->x2 - area->x1 + 1);
    uint16_t color_row[screenWidth];
 
    for (int y = area->y1; y <= area->y2; y++)  {
      for (int x=0; x<row_count; x++) {
        color_row[x] = color_p++->full;
      }
      tft.drawPixels(area->x1, y, color_row, row_count);
      // tft.BTEWriteROP(area->x1, y, screenWidth, screenHeight,  color_row, row_count);
    } 

    lv_disp_flush_ready( disp );
}

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;
    data->state = (tft.isTouched() == true) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

    if (!digitalRead(RA8875_INT))  {
      if(data->state = LV_INDEV_STATE_PR)
      {
        tft.readTouch(&touchX, &touchY);
        data->point.x = touchX;
        data->point.y = touchY;
        // printf( "Data (x, y) = (%d, %d)\n", touchX, touchY);
      }
    }
}

static void btn_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "Button: %d", cnt);
    }
}