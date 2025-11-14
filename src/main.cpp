#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <GT911.h>

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

// ===== Dashboard Protocol Constants =====
#define STX1             0x5D  // Start of text1
#define STX2             0x47  // Start of text2
#define ETX              0x78  // End of text

// Data Identifiers
#define ID_SOC           0x85  // State of Charge (0-100%)
#define ID_VOLTAGE       0x83  // Total voltage (0.01V precision)
#define ID_CURRENT       0x84  // Current (0.01A precision)
#define ID_TEMP          0x80  // Battery Temperature (0.1°C precision)      battery_temp_label
#define ID_SPEED         0x82  // Vehicle speed (0.1 km/h precision)          speed_label
#define ID_MODE          0x86  // Driving mode (0=ECO, 1=CITY, 2=SPORT)        mode_label
#define ID_ARMED         0x87  // Armed status (0=DISARMED, 1=ARMED)            status_label
#define ID_RANGE         0x88  // Remaining range (0.1 km precision)             range_label
#define ID_CONSUMPTION   0x89  // Average consumption (0.1 W/km precision)       avg_wkm_label
#define ID_AMBIENT_TEMP  0x8A  // Ambient temperature (0.1°C precision)           motor_temp_label
#define ID_TRIP          0x8B  // Trip distance (0.1 km precision)               trip_label
#define ID_ODOMETER      0x8C  // Odometer (0.1 km precision)                    odo_label
#define ID_AVG_SPEED     0x8D  // Average speed (0.1 km/h precision)            avg_kmh_label

// Driving Modes
enum DrivingMode {
  MODE_ECO = 0,
  MODE_CITY = 1,
  MODE_SPORT = 2
};

// ===== Buffer for receiving data =====
uint8_t serialBuffer[332];
uint16_t bufferPos = 0;

GT911 ts = GT911();
void *draw_buf;
lv_display_t *disp;

/* Buffer to store image data in RAM */
uint8_t *image_data = NULL;
uint32_t image_size = 0;

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

/* Dashboard Data Structure */
struct DashboardData {
  int speed;
  int range;
  int avg_wkm;
  int trip;
  int odo;
  int avg_kmh;
  int motor_temp;
  int battery_temp;
  String mode;
  String status;
  int soc;
  float voltage;
  float current;
} dashData;

/* Initialize dashboard data with defaults */
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

// ===== CRC-16 Modbus Calculation =====
uint16_t calculateChecksum(const uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)data[pos];
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ===== Frame Validation =====
bool validateFrame(uint8_t* frame, uint16_t len) {
  if (len < 15 || frame[0] != STX1 || frame[1] != STX2) {
    return false;
  }

  uint16_t declaredLength = (frame[2] << 8) | frame[3];
  uint16_t expectedLength = declaredLength + 6;
  
  if (len != expectedLength) {
    return false;
  }

  uint16_t etxPos = 4 + declaredLength - 1;
  if (frame[etxPos] != ETX) {
    return false;
  }

  uint16_t calculatedChecksum = calculateChecksum(&frame[2], declaredLength + 2);
  uint16_t receivedChecksum = (frame[expectedLength-2] << 8) | frame[expectedLength-1];
  
  if (receivedChecksum != calculatedChecksum) {
    return false;
  }

  return true;
}

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

/* Load image from SD card into RAM */
bool load_image_to_ram(const char *path) {
  Serial.printf("Loading image: %s\n", path);

  File file = SD.open(path);
  if (!file) {
    Serial.println("ERROR: Failed to open image file!");
    return false;
  }

  image_size = file.size();
  image_data = (uint8_t *)malloc(image_size);
  if (!image_data) {
    Serial.println("ERROR: Failed to allocate memory for image!");
    file.close();
    return false;
  }

  size_t bytes_read = file.read(image_data, image_size);
  file.close();

  if (bytes_read != image_size) {
    free(image_data);
    image_data = NULL;
    return false;
  }

  Serial.println("Image loaded into RAM successfully!");
  return true;
}

/* Update time display */
void update_time_display() {
  unsigned long now = millis() / 1000;
  int hours = (now / 3600) % 24;
  int minutes = (now / 60) % 60;

  char time_str[16];
  snprintf(time_str, sizeof(time_str), "%d:%02d AM", hours == 0 ? 12 : hours, minutes);
  lv_label_set_text(time_label, time_str);
}

/* Update specific UI element based on ID */
void update_ui_element(uint8_t id) {
  char buf[32];
  
  switch(id) {
    case ID_SPEED:
      snprintf(buf, sizeof(buf), "%d", dashData.speed);
      lv_label_set_text(speed_label, buf);
      break;
      
    case ID_RANGE:
      snprintf(buf, sizeof(buf), "Range %d km", dashData.range);
      lv_label_set_text(range_label, buf);
      break;
      
    case ID_CONSUMPTION:
      snprintf(buf, sizeof(buf), "Avg. %d W/km", dashData.avg_wkm);
      lv_label_set_text(avg_wkm_label, buf);
      break;
      
    case ID_TRIP:
      snprintf(buf, sizeof(buf), "TRIP %d km", dashData.trip);
      lv_label_set_text(trip_label, buf);
      break;
      
    case ID_ODOMETER:
      snprintf(buf, sizeof(buf), "ODO %d km", dashData.odo);
      lv_label_set_text(odo_label, buf);
      break;
      
    case ID_AVG_SPEED:
      snprintf(buf, sizeof(buf), "AVG. %d km/h", dashData.avg_kmh);
      lv_label_set_text(avg_kmh_label, buf);
      break;
      
    case ID_TEMP:
      snprintf(buf, sizeof(buf), "Battery %d°C", dashData.battery_temp);
      lv_label_set_text(battery_temp_label, buf);
      break;
      
    case ID_AMBIENT_TEMP:
      snprintf(buf, sizeof(buf), "Motor %d°C", dashData.motor_temp);
      lv_label_set_text(motor_temp_label, buf);
      break;
      
    case ID_MODE:
      lv_label_set_text(mode_label, dashData.mode.c_str());
      // Update color based on mode
      if (dashData.mode == "Eco") {
        lv_obj_set_style_text_color(mode_label, lv_color_hex(0x00cc00), 0);
      } else if (dashData.mode == "City") {
        lv_obj_set_style_text_color(mode_label, lv_color_hex(0x0088ff), 0);
      } else if (dashData.mode == "Sport") {
        lv_obj_set_style_text_color(mode_label, lv_color_hex(0xff0000), 0);
      }
      break;
      
    case ID_ARMED:
      lv_label_set_text(status_label, dashData.status.c_str());
      break;

      case ID_SOC:
      snprintf(buf, sizeof(buf), "SoC: %d%%", dashData.soc);
      lv_label_set_text(soc, buf);
      break;

    case ID_VOLTAGE:
      snprintf(buf, sizeof(buf), "Volt: %.2f V", dashData.voltage);
      lv_label_set_text(voltage, buf);
      break;

    case ID_CURRENT:
      snprintf(buf, sizeof(buf), "Curr: %.2f A", dashData.current);
      lv_label_set_text(current, buf);
      break;
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

  lv_obj_t *menu_btn = lv_label_create(top_bar);
  lv_label_set_text(menu_btn, "Menu");
  lv_obj_set_style_text_font(menu_btn, &lv_font_montserrat_16, 0);
  lv_obj_align(menu_btn, LV_ALIGN_LEFT_MID, 10, 0);

  lv_obj_t *map_btn = lv_label_create(top_bar);
  lv_label_set_text(map_btn, "Map");
  lv_obj_set_style_text_font(map_btn, &lv_font_montserrat_16, 0);
  lv_obj_align(map_btn, LV_ALIGN_RIGHT_MID, -10, 0);

  /* Status badge */
  lv_obj_t *status_badge = lv_obj_create(scr);
  lv_obj_set_size(status_badge, 140, 49);
  lv_obj_align(status_badge, LV_ALIGN_TOP_MID, 0, 60);
  lv_obj_set_style_bg_color(status_badge, lv_color_hex(0x333333), 0);
  lv_obj_set_style_radius(status_badge, 20, 0);
  lv_obj_set_style_border_width(status_badge, 0, 0);

  status_label = lv_label_create(status_badge);
  lv_label_set_text(status_label, dashData.status.c_str());
  lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
  lv_obj_set_style_text_font(status_label, &lv_font_montserrat_16, 0);
  lv_obj_center(status_label);

  /* Main speed display */
  speed_label = lv_label_create(scr);
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", dashData.speed);
  lv_label_set_text(speed_label, buf);
  lv_obj_set_style_text_color(speed_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
  lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -20);

  lv_obj_t *kmh_label = lv_label_create(scr);
  lv_label_set_text(kmh_label, "Km/h");
  lv_obj_set_style_text_color(kmh_label, lv_color_black(), 0);
  lv_obj_set_style_text_font(kmh_label, &lv_font_montserrat_16, 0);
  lv_obj_align(kmh_label, LV_ALIGN_CENTER, 0, 20);

  /* Mode selector */
  lv_obj_t *mode_container = lv_obj_create(scr);
  lv_obj_set_size(mode_container, 200, 90);
  lv_obj_align(mode_container, LV_ALIGN_CENTER, 0, 80);
  lv_obj_set_style_bg_color(mode_container, lv_color_white(), 0);
  lv_obj_set_style_radius(mode_container, 10, 0);
  lv_obj_set_style_border_width(mode_container, 0, 0);

  lv_obj_t *mode_text = lv_label_create(mode_container);
  lv_label_set_text(mode_text, "Mode");
  lv_obj_set_style_text_color(mode_text, lv_color_black(), 0);
  lv_obj_set_style_text_font(mode_text, &lv_font_montserrat_16, 0);
  lv_obj_align(mode_text, LV_ALIGN_TOP_MID, 0, 3);

  mode_label = lv_label_create(mode_container);
  lv_label_set_text(mode_label, dashData.mode.c_str());
  lv_obj_set_style_text_color(mode_label, lv_color_hex(0x00cc00), 0);
  lv_obj_set_style_text_font(mode_label, &lv_font_montserrat_20, 0);
  lv_obj_align(mode_label, LV_ALIGN_CENTER, 0, 15);

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

/* Process RS485 frames and update UI */
// void read_rs485_frames() {
//   // Read available bytes from Serial1
//   while (Serial1.available()) {
  
//     memmove(serialBuffer, serialBuffer+1, sizeof(serialBuffer)-1);
//     bufferPos--;
//     serialBuffer[bufferPos++] = Serial1.read();
    
//     char temp[sizeof(serialBuffer) + 1];
//     memcpy(temp, serialBuffer, bufferPos);
//     temp[bufferPos] = '\0';

//     Serial.println(temp);
//     yield();
//   }
  
//   // Process complete frames
//   bool frameFound = true;
//   while (frameFound && bufferPos >= 6) {
//     frameFound = false;
    
//     for (uint16_t i = 0; i < bufferPos-1; i++) {
//       if (serialBuffer[i] == STX1 && serialBuffer[i+1] == STX2) {
//         if (i+3 >= bufferPos) break;
        
//         uint16_t declaredLength = (serialBuffer[i+2] << 8) | serialBuffer[i+3];
//         uint16_t frameLength = declaredLength + 6;
        
//         if (i + frameLength <= bufferPos) {
//           if (validateFrame(&serialBuffer[i], frameLength)) {
//             Serial.println("\n[RS485] Valid frame received");
            
//             uint8_t infoEnd = i + 4 + declaredLength - 5;
//             uint8_t updatedIDs[20];
//             uint8_t updateCount = 0;
            
//             // Parse all data fields
//             for (uint8_t j = i+11; j < infoEnd;) {
//               uint8_t id = serialBuffer[j++];
              
//               switch (id) {
//                 case ID_SOC:
//                   dashData.soc = serialBuffer[j++];
//                   Serial.printf("  SOC: %d%%\n", dashData.soc);
//                   updatedIDs[updateCount++] = id;
//                   break;
                  
//                 case ID_VOLTAGE: {
//                   uint16_t v = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.voltage = v * 0.01f;
//                   j += 2;
//                   Serial.printf("  Voltage: %.2f V\n", dashData.voltage);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_CURRENT: {
//                   uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.current = (c & 0x8000) ? -(c & 0x7FFF) * 0.01f : c * 0.01f;
//                   j += 2;
//                   Serial.printf("  Current: %.2f A\n", dashData.current);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_TEMP: {
//                   uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.battery_temp = (int)(t * 0.1f);
//                   j += 2;
//                   Serial.printf("  Battery Temp: %d°C\n", dashData.battery_temp);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_SPEED: {
//                   uint16_t s = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.speed = (int)(s * 0.1f);
//                   j += 2;
//                   Serial.printf("  Speed: %d km/h\n", dashData.speed);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_MODE: {
//                   uint8_t m = serialBuffer[j++];
//                   if (m == MODE_ECO) dashData.mode = "Eco";
//                   else if (m == MODE_CITY) dashData.mode = "City";
//                   else if (m == MODE_SPORT) dashData.mode = "Sport";
//                   Serial.printf("  Mode: %s\n", dashData.mode.c_str());
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_ARMED: {
//                   uint8_t a = serialBuffer[j++];
//                   dashData.status = a ? "ARMED" : "DISARMED";
//                   Serial.printf("  Status: %s\n", dashData.status.c_str());
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_RANGE: {
//                   uint16_t r = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.range = (int)(r * 0.1f);
//                   j += 2;
//                   Serial.printf("  Range: %d km\n", dashData.range);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_CONSUMPTION: {
//                   uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.avg_wkm = (int)(c * 0.1f);
//                   j += 2;
//                   Serial.printf("  Consumption: %d W/km\n", dashData.avg_wkm);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_AMBIENT_TEMP: {
//                   uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.motor_temp = (int)(t * 0.1f);
//                   j += 2;
//                   Serial.printf("  Motor Temp: %d°C\n", dashData.motor_temp);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_TRIP: {
//                   uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.trip = (int)(t * 0.1f);
//                   j += 2;
//                   Serial.printf("  Trip: %d km\n", dashData.trip);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_ODOMETER: {
//                   uint32_t o = (serialBuffer[j] << 24) | (serialBuffer[j+1] << 16) | 
//                               (serialBuffer[j+2] << 8) | serialBuffer[j+3];
//                   dashData.odo = (int)(o * 0.1f);
//                   j += 4;
//                   Serial.printf("  Odometer: %d km\n", dashData.odo);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }
                
//                 case ID_AVG_SPEED: {
//                   uint16_t as = (serialBuffer[j] << 8) | serialBuffer[j+1];
//                   dashData.avg_kmh = (int)(as * 0.1f);
//                   j += 2;
//                   Serial.printf("  Avg Speed: %d km/h\n", dashData.avg_kmh);
//                   updatedIDs[updateCount++] = id;
//                   break;
//                 }

//                 default:
//                   Serial.printf("  Unknown ID: 0x%02X\n", id);
//                   if (id == ID_ODOMETER) j += 4;
//                   else if (id >= 0x80 && id <= 0x8F) j += 2;
//                   else j++;
//                   break;
//               }
//             }
            
//             // Update only changed UI elements
//             Serial.printf("[UI] Updating %d elements...\n", updateCount);
//             for (uint8_t k = 0; k < updateCount; k++) {
//               update_ui_element(updatedIDs[k]);
//             }
            
//             // Single display refresh
//             lv_refr_now(disp);
//             Serial.println("[UI] Display updated\n");
            
//             // Remove processed frame
//             memmove(serialBuffer, &serialBuffer[i + frameLength], 
//                     bufferPos - (i + frameLength));
//             bufferPos -= (i + frameLength);
//             frameFound = true;
//             break;
//           } else {
//             Serial.println("Invalid Frame Received");
//             i++; 
//           }
//         } else {
//           break;
//         }

//       }
//     }
    
//     if (!frameFound && bufferPos > 200) {
//       Serial.println("[WARNING] Buffer full, clearing");
//       bufferPos = 0;
//     }
//   }
// }

/* Process RS485 frames and update UI */
void read_rs485_frames() {
  // Read available bytes from Serial1
  while (Serial1.available()) {
    uint8_t incomingByte = Serial1.read();
    
    // Check if we should start capturing (5D 47)
    if (bufferPos == 0) {
      // Wait for STX1 (5D)
      if (incomingByte == STX1) {
        serialBuffer[bufferPos++] = incomingByte;
      }
      // Discard any other byte
    } 
    else if (bufferPos == 1) {
      // Check for STX2 (47)
      if (incomingByte == STX2) {
        serialBuffer[bufferPos++] = incomingByte;
        Serial.println("[FRAME] Start detected (5D 47)");
      } else {
        // False start, reset and check if this byte is STX1
        bufferPos = 0;
        if (incomingByte == STX1) {
          serialBuffer[bufferPos++] = incomingByte;
        }
      }
    }
    else {
      // Frame started, continue capturing
      if (bufferPos < sizeof(serialBuffer)) {
        serialBuffer[bufferPos++] = incomingByte;
        
        // Check if we have enough bytes to read length field
        if (bufferPos == 4) {
          uint16_t declaredLength = (serialBuffer[2] << 8) | serialBuffer[3];
          uint16_t expectedFrameLength = declaredLength + 6;
          Serial.printf("[FRAME] Expected total length: %d bytes\n", expectedFrameLength);
          
          // Sanity check on frame length
          if (expectedFrameLength > sizeof(serialBuffer) || expectedFrameLength < 15) {
            Serial.println("[ERROR] Invalid frame length, resetting");
            bufferPos = 0;
          }
        }
        
        // Check if we might have a complete frame
        if (bufferPos >= 15) { // Minimum valid frame size
          uint16_t declaredLength = (serialBuffer[2] << 8) | serialBuffer[3];
          uint16_t expectedFrameLength = declaredLength + 6;
          
          // Check if we have received the complete frame
          if (bufferPos >= expectedFrameLength) {
            uint16_t etxPos = expectedFrameLength - 3;
            
            // Verify ETX at correct position
            if (serialBuffer[etxPos] == ETX) {
              Serial.println("[FRAME] Complete frame captured!");
              Serial.print("Frame: ");
              for (uint16_t i = 0; i < bufferPos; i++) {
                Serial.printf("%02X ", serialBuffer[i]);
              }
              Serial.println();
              
              // // Process the frame
              // if (validateFrame(serialBuffer, expectedFrameLength)) {
              //   processCompleteFrame();
              // } else {
              //   Serial.println("[ERROR] Frame validation failed");
              // }
            } else {
              Serial.printf("[ERROR] ETX not found at position %d\n", etxPos);
            }
            
            // Reset buffer for next frame
            bufferPos = 0;
          }
        }
      } else {
        // Buffer overflow
        Serial.println("[ERROR] Buffer overflow, resetting");
        bufferPos = 0;
      }
    }
    
    yield();
  }
}

/* Process validated complete frame */
void processCompleteFrame() {
  Serial.println("\n[RS485] Processing valid frame");
  
  uint16_t declaredLength = (serialBuffer[2] << 8) | serialBuffer[3];
  uint8_t infoEnd = 4 + declaredLength - 5;
  uint8_t updatedIDs[20];
  uint8_t updateCount = 0;
  
  // Parse all data fields (starting at position 11 after 7-byte header)
  for (uint8_t j = 11; j < infoEnd;) {
    uint8_t id = serialBuffer[j++];
    
    switch (id) {
      case ID_SOC:
        dashData.soc = serialBuffer[j++];
        Serial.printf("  SOC: %d%%\n", dashData.soc);
        updatedIDs[updateCount++] = id;
        break;
        
      case ID_VOLTAGE: {
        uint16_t v = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.voltage = v * 0.01f;
        j += 2;
        Serial.printf("  Voltage: %.2f V\n", dashData.voltage);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_CURRENT: {
        uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.current = (c & 0x8000) ? -(c & 0x7FFF) * 0.01f : c * 0.01f;
        j += 2;
        Serial.printf("  Current: %.2f A\n", dashData.current);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_TEMP: {
        uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.battery_temp = (int)(t * 0.1f);
        j += 2;
        Serial.printf("  Battery Temp: %d°C\n", dashData.battery_temp);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_SPEED: {
        uint16_t s = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.speed = (int)(s * 0.1f);
        j += 2;
        Serial.printf("  Speed: %d km/h\n", dashData.speed);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_MODE: {
        uint8_t m = serialBuffer[j++];
        if (m == MODE_ECO) dashData.mode = "Eco";
        else if (m == MODE_CITY) dashData.mode = "City";
        else if (m == MODE_SPORT) dashData.mode = "Sport";
        Serial.printf("  Mode: %s\n", dashData.mode.c_str());
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_ARMED: {
        uint8_t a = serialBuffer[j++];
        dashData.status = a ? "ARMED" : "DISARMED";
        Serial.printf("  Status: %s\n", dashData.status.c_str());
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_RANGE: {
        uint16_t r = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.range = (int)(r * 0.1f);
        j += 2;
        Serial.printf("  Range: %d km\n", dashData.range);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_CONSUMPTION: {
        uint16_t c = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.avg_wkm = (int)(c * 0.1f);
        j += 2;
        Serial.printf("  Consumption: %d W/km\n", dashData.avg_wkm);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_AMBIENT_TEMP: {
        uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.motor_temp = (int)(t * 0.1f);
        j += 2;
        Serial.printf("  Motor Temp: %d°C\n", dashData.motor_temp);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_TRIP: {
        uint16_t t = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.trip = (int)(t * 0.1f);
        j += 2;
        Serial.printf("  Trip: %d km\n", dashData.trip);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_ODOMETER: {
        uint32_t o = (serialBuffer[j] << 24) | (serialBuffer[j+1] << 16) | 
                    (serialBuffer[j+2] << 8) | serialBuffer[j+3];
        dashData.odo = (int)(o * 0.1f);
        j += 4;
        Serial.printf("  Odometer: %d km\n", dashData.odo);
        updatedIDs[updateCount++] = id;
        break;
      }
      
      case ID_AVG_SPEED: {
        uint16_t as = (serialBuffer[j] << 8) | serialBuffer[j+1];
        dashData.avg_kmh = (int)(as * 0.1f);
        j += 2;
        Serial.printf("  Avg Speed: %d km/h\n", dashData.avg_kmh);
        updatedIDs[updateCount++] = id;
        break;
      }

      default:
        Serial.printf("  Unknown ID: 0x%02X\n", id);
        if (id == ID_ODOMETER) j += 4;
        else if (id >= 0x80 && id <= 0x8F) j += 2;
        else j++;
        break;
    }
  }
  
  // Update only changed UI elements
  Serial.printf("[UI] Updating %d elements...\n", updateCount);
  for (uint8_t k = 0; k < updateCount; k++) {
    update_ui_element(updatedIDs[k]);
  }
  
  // Single display refresh
  lv_refr_now(disp);
  Serial.println("[UI] Display updated\n");
}

void setup() {
  Serial.begin(115200);
  delay(100);

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