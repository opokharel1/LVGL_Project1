#include "ui.h"

DashboardData dashData;
lv_display_t *disp;
void *draw_buf;
uint8_t *image_data;
uint32_t image_size;

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