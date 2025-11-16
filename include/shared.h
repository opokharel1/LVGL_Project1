#pragma once
// shared.h - central place for protocol constants, shared types and externs

#include <Arduino.h>
#include <lvgl.h>

// ===== Protocol constants =====
#define STX1             0x5D  // Start of text1
#define STX2             0x47  // Start of text2
#define ETX              0x78  // End of text

// Data Identifiers
#define ID_SOC           0x85
#define ID_VOLTAGE       0x83
#define ID_CURRENT       0x84
#define ID_TEMP          0x80
#define ID_SPEED         0x82
#define ID_MODE          0x86
#define ID_ARMED         0x87
#define ID_RANGE         0x88
#define ID_CONSUMPTION   0x89
#define ID_AMBIENT_TEMP  0x8A
#define ID_TRIP          0x8B
#define ID_ODOMETER      0x8C
#define ID_AVG_SPEED     0x8D

// Driving Modes enum
enum DrivingMode {
  MODE_ECO = 0,
  MODE_CITY = 1,
  MODE_SPORT = 2
};

// ===== Dashboard Data Structure =====
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
};

// ===== Extern globals (defined in ONE .cpp only) =====
// Define these in main.cpp (or a dedicated shared.cpp). Here we only declare them.
extern DashboardData dashData;
extern lv_display_t *disp;
extern void *draw_buf;
extern uint8_t *image_data;
extern uint32_t image_size;

// UI object pointers (created in UI module)
extern lv_obj_t *speed_label;
extern lv_obj_t *range_label;
extern lv_obj_t *avg_wkm_label;
extern lv_obj_t *trip_label;
extern lv_obj_t *odo_label;
extern lv_obj_t *avg_kmh_label;
extern lv_obj_t *motor_temp_label;
extern lv_obj_t *battery_temp_label;
extern lv_obj_t *mode_label;
extern lv_obj_t *status_label;
extern lv_obj_t *soc;
extern lv_obj_t *voltage;
extern lv_obj_t *current;
extern lv_obj_t *time_label;

// ===== Shared function prototypes =====

// // CRC & frame helpers (implemented in main.cpp or a helper cpp)

// uint16_t calculateChecksum(const uint8_t *data, uint16_t length);
// bool validateFrame(uint8_t* frame, uint16_t len);

// // UI helpers (implemented in ui.cpp)

// void update_ui_element(uint8_t id);
// void update_time_display();
// void create_ev_dashboard_ui();
// bool load_image_to_ram(const char *path);

// NOTE: serialBuffer & bufferPos are intentionally NOT externed here.
// Keep them local to rs485.cpp for encapsulation (recommended).
