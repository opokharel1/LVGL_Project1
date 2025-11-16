#include "shared.h"
#include "rs485.h"
#include "ui.h"
#include "fonts/lv_font_montserrat_78.h"

#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <GT911.h>

#include <hardwareserial.h>
#define SD_CS 5
#define TFT_HOR_RES 480  // LANDSCAPE: Width first
#define TFT_VER_RES 320  // LANDSCAPE: Height second

/* Touch pins */
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21
#define TOUCH_RST 25

// ===== Serial Configuration =====
#define SERIAL1_RX 16
#define SERIAL1_TX 17

GT911 ts = GT911();

/* Dashboard UI Elements - Global pointers to labels */
lv_obj_t *speed_label;
lv_obj_t *range_label;
lv_obj_t *avg_wkm_label;
lv_obj_t *trip_label;
lv_obj_t *odo_label;
lv_obj_t *avg_kmh_label;
lv_obj_t *motor_temp_label;
lv_obj_t *battery_temp_label;
lv_obj_t *mode_label;
lv_obj_t *status_label;

lv_obj_t *soc;
lv_obj_t *voltage;
lv_obj_t *current;

lv_obj_t *time_label;              // update in time
lv_obj_t *menu_btn = NULL;

/* Touch callback */
void my_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint8_t touches = ts.touched(GT911_MODE_POLLING);
  if (touches) {
    GTPoint *p = ts.getPoints();
    data->point.x = TFT_HOR_RES - p->y;
    data->point.y = p->x;
    data->state = LV_INDEV_STATE_PRESSED;
  } else { 
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

/* Create EV Dashboard UI */
void create_ev_dashboard_ui() {
  Serial.println("Creating EV dashboard UI...");

  lv_obj_t *scr = lv_scr_act();
  lv_obj_clean(scr);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0xe5e5e5), 0);

  /* Top bar */
  lv_obj_t *top_bar = lv_obj_create(scr);
  lv_obj_set_size(top_bar, TFT_HOR_RES, 55);
  lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(top_bar, lv_color_white(), 0);
  lv_obj_set_style_border_width(top_bar, 0, 0);
  lv_obj_set_style_radius(top_bar, 0, 0);
  lv_obj_set_style_pad_all(top_bar, 0, 0);

  time_label = lv_label_create(top_bar);
  lv_label_set_text(time_label, "9:41 AM");
  lv_obj_set_style_text_color(time_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_18, 0);
  lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 0);

  // Create menu button
  menu_btn = lv_btn_create(top_bar);
  lv_obj_set_size(menu_btn, 50, 45);
  lv_obj_align(menu_btn, LV_ALIGN_LEFT_MID, 0, 0);
  lv_obj_add_flag(menu_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(menu_btn, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
  lv_obj_set_style_bg_color(menu_btn, lv_color_hex(0x333333), 0);

  // Create menu symbol
  lv_obj_t *menu_label = lv_label_create(menu_btn);
  lv_label_set_text(menu_label, LV_SYMBOL_BARS);
  lv_obj_set_style_text_font(menu_label, &lv_font_montserrat_20, 0);
  lv_obj_center(menu_label);

  lv_obj_t *map_btn = lv_label_create(top_bar);
  lv_label_set_text(map_btn, "Map");
  lv_obj_set_style_text_font(map_btn, &lv_font_montserrat_16, 0);
  lv_obj_align(map_btn, LV_ALIGN_RIGHT_MID, -10, 0);

  /* Status badge */
  // lv_obj_t *status_badge = lv_obj_create(scr);
  // lv_obj_set_size(status_badge, 140, 49);
  // lv_obj_align(status_badge, LV_ALIGN_TOP_MID, 0, 60);
  // lv_obj_set_style_bg_color(status_badge, lv_color_hex(0x333333), 0);
  // lv_obj_set_style_radius(status_badge, 20, 0);
  // lv_obj_set_style_border_width(status_badge, 0, 0);

  // status_label = lv_label_create(status_badge);
  // lv_label_set_text(status_label, dashData.status.c_str());
  // lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
  // lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
  // lv_obj_center(status_label);

  /* Main speed display */
  speed_label = lv_label_create(scr);
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", dashData.speed);
  lv_label_set_text(speed_label, buf);
  lv_obj_set_style_text_color(speed_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_78, 0);
  lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -40);

  lv_obj_t *kmh_label = lv_label_create(scr);
  lv_label_set_text(kmh_label, "Km/h");
  lv_obj_set_style_text_color(kmh_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(kmh_label, &lv_font_montserrat_16, 0);
  lv_obj_align(kmh_label, LV_ALIGN_CENTER, 66, -34);

  /* Mode selector */
  lv_obj_t *mode_container = lv_obj_create(scr);
  lv_obj_set_size(mode_container, 100, 60);
  lv_obj_align(mode_container, LV_ALIGN_CENTER, 0, 45);
  lv_obj_set_style_bg_color(mode_container, lv_color_white(), 0);
  lv_obj_set_style_radius(mode_container, 10, 0);
  lv_obj_set_style_border_width(mode_container, 0, 0);

  // lv_obj_t *mode_text = lv_label_create(mode_container);
  // lv_label_set_text(mode_text, "Mode");
  // lv_obj_set_style_text_color(mode_text, lv_color_black(), 0);
  // lv_obj_set_style_text_font(mode_text, &lv_font_montserrat_16, 0);
  // lv_obj_align(mode_text, LV_ALIGN_TOP_MID, 0, 3);

  mode_label = lv_label_create(mode_container);
  lv_label_set_text(mode_label, dashData.mode.c_str());
  lv_obj_set_style_text_color(mode_label, lv_color_hex(0x00cc00), 0);
  lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_20, 0);
  lv_obj_align(mode_label, LV_ALIGN_CENTER, 0, 0);

  /* Left side info */
  range_label = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Range: %d km", dashData.range);
  lv_label_set_text(range_label, buf);
  lv_obj_set_style_text_color(range_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(range_label, &lv_font_montserrat_16, 0);
  lv_obj_align(range_label, LV_ALIGN_LEFT_MID, 10, -60);

  avg_wkm_label = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Avg. con: %d W/km", dashData.avg_wkm);
  lv_label_set_text(avg_wkm_label, buf);
  lv_obj_set_style_text_color(avg_wkm_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(avg_wkm_label, &lv_font_montserrat_16, 0);
  lv_obj_align(avg_wkm_label, LV_ALIGN_LEFT_MID, 10, -20);

  voltage = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Volt: %.2f V", dashData.voltage);
  lv_label_set_text(voltage, buf);
  lv_obj_set_style_text_color(voltage, lv_color_black(), 0);
  lv_obj_set_style_text_font(voltage, &lv_font_montserrat_16, 0);
  lv_obj_align(voltage, LV_ALIGN_LEFT_MID, 10, 60);

  current = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Current: %.2f A", dashData.current); 
  lv_label_set_text(current, buf);
  lv_obj_set_style_text_color(current, lv_color_black(), 0);
  lv_obj_set_style_text_font(current, &lv_font_montserrat_16, 0);
  lv_obj_align(current, LV_ALIGN_LEFT_MID, 10, 90);

  /* Right side info */
  motor_temp_label = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Motor: %d°C", dashData.motor_temp);
  lv_label_set_text(motor_temp_label, buf);
  lv_obj_set_style_text_color(motor_temp_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(motor_temp_label, &lv_font_montserrat_16, 0);
  lv_obj_align(motor_temp_label, LV_ALIGN_RIGHT_MID, -10, -60);

  battery_temp_label = lv_label_create(scr);
  snprintf(buf, sizeof(buf), "Battery: %d°C", dashData.battery_temp);
  lv_label_set_text(battery_temp_label, buf);
  lv_obj_set_style_text_color(battery_temp_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(battery_temp_label, &lv_font_montserrat_16, 0);
  lv_obj_align(battery_temp_label, LV_ALIGN_RIGHT_MID, -10, -20);

  soc =lv_label_create(scr);
  snprintf(buf, sizeof(buf), "SoC: %d%%", dashData.soc);
  lv_label_set_text(soc, buf);
  lv_obj_set_style_text_color(soc, lv_color_black(), 0);
  lv_obj_set_style_text_font(soc, &lv_font_montserrat_16, 0);
  lv_obj_align(soc, LV_ALIGN_RIGHT_MID, -10, 60);

  /* Bottom bar */
  lv_obj_t *bottom_bar = lv_obj_create(scr);
  lv_obj_set_size(bottom_bar, TFT_HOR_RES, 50);
  lv_obj_align(bottom_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(bottom_bar, lv_color_white(), 0);
  lv_obj_set_style_border_width(bottom_bar, 0, 0);
  lv_obj_set_style_radius(bottom_bar, 0, 0);

  trip_label = lv_label_create(bottom_bar);
  snprintf(buf, sizeof(buf), "TRIP: %d km", dashData.trip);
  lv_label_set_text(trip_label, buf);
  lv_obj_set_style_text_color(trip_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(trip_label, &lv_font_montserrat_14, 0);
  lv_obj_align(trip_label, LV_ALIGN_LEFT_MID, 5, 0);

  odo_label = lv_label_create(bottom_bar);
  snprintf(buf, sizeof(buf), "ODO: %d km", dashData.odo);
  lv_label_set_text(odo_label, buf);
  lv_obj_set_style_text_color(odo_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(odo_label, &lv_font_montserrat_14, 0);
  lv_obj_align(odo_label, LV_ALIGN_CENTER, 0, 0);

  avg_kmh_label = lv_label_create(bottom_bar);
  snprintf(buf, sizeof(buf), "Avg. SPEED: %d km/h", dashData.avg_kmh);
  lv_label_set_text(avg_kmh_label, buf);
  lv_obj_set_style_text_color(avg_kmh_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(avg_kmh_label, &lv_font_montserrat_14, 0);
  lv_obj_align(avg_kmh_label, LV_ALIGN_RIGHT_MID, -2, 0);

  Serial.println("EV dashboard UI created!");
}

void init_dashboard_data() {
  dashData.speed = 0;
  dashData.range = 10;
  dashData.avg_wkm = 30;
  dashData.trip = 110;
  dashData.odo = 10;
  dashData.avg_kmh = 10;
  dashData.motor_temp = 20;
  dashData.battery_temp = 10;
  dashData.mode = "Sports";
  dashData.status = "ARMED";
  dashData.soc = 25;
  dashData.voltage = 23.0;
  dashData.current = 0.0;
}


void setup() {
  Serial.begin(115200);
  delay(100);


  // Your typical frame is ~22 bytes, so trigger at 24 bytes
  Serial1.setRxFIFOFull(64);
  
  // Default is 256 bytes, increase to 1024 bytes
  Serial1.setRxBufferSize(1024);

  // Initialize RS485
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);



  Serial.println("\n=== EV Dashboard ===");

  // Initialize data structure
  init_dashboard_data();

  /* Initialize SD Card */
  Serial.println("Initializing SD Card...");
  SPIClass spi = SPIClass(VSPI);
  spi.begin(18, 19, 23, SD_CS);

  if (!SD.begin(SD_CS, spi)) {
    Serial.println("ERROR: SD Card mount failed!");
    while (1) delay(1000);
  }

  /* Load splash image */
  if (!load_image_to_ram("/lvgl/logo1.bin")) {
    Serial.println("ERROR: Failed to load image!");
    while (1) delay(1000);
  }

  SD.end();

  /* Initialize LVGL */
  lv_init();

  /* Initialize touch */
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  ts.begin(TOUCH_INT, TOUCH_RST);

  /* Allocate draw buffer */
  draw_buf = heap_caps_malloc(
      TFT_HOR_RES * 40 * (LV_COLOR_DEPTH / 8),
      MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);

  if (!draw_buf) {
    Serial.println("ERROR: Draw buffer allocation failed!");
    while (1) delay(1000);
  }

  /* Create display */
  disp = lv_tft_espi_create(
      TFT_HOR_RES, TFT_VER_RES, draw_buf,
      TFT_HOR_RES * 40 * (LV_COLOR_DEPTH / 8));

  TFT_eSPI().setRotation(3);

  /* Setup touch input */
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touch_read);

  /* Show splash screen */
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_white(), 0);

  lv_obj_t *label = lv_label_create(scr);
  lv_label_set_text(label, "Charge Into The Future");
  lv_obj_set_style_text_color(label, lv_color_black(), 0);
  lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, -64);

  static lv_image_dsc_t img_dsc;
  img_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
  img_dsc.header.w = 148;
  img_dsc.header.h = 148;
  img_dsc.data_size = image_size;
  img_dsc.data = image_data;

  lv_obj_t *img = lv_image_create(scr);
  lv_image_set_src(img, &img_dsc);
  lv_obj_align(img, LV_ALIGN_CENTER, 0, 4);

  lv_refr_now(disp);
  delay(3000);

  /* Cleanup splash */
  lv_obj_delete(img);
  lv_obj_delete(label);
  if (image_data) {
    free(image_data);
    image_data = NULL;
  }

  /* Create dashboard with initial values */
  create_ev_dashboard_ui();
  lv_refr_now(disp);

  Serial.println("\n=== Setup Complete ===");
  Serial.println("Waiting for RS485 data...");
}

unsigned long last_time_update = 0;

void loop() {
  lv_timer_handler();

  // Update time every second
  if (millis() - last_time_update > 1000) {
    update_time_display();
    last_time_update = millis();
  }

  // Process RS485 frames and auto-update UI
  read_rs485_frames();

  delay(5);
}