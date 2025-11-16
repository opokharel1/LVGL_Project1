#pragma once
// ui.h - UI/API exposed by the UI module

#include "shared.h"
#include <FS.h>
#include <SD.h>

#ifdef __cplusplus
extern "C" {
#endif

// void create_ev_dashboard_ui(void);
void update_ui_element(uint8_t id);
void update_time_display(void);
bool load_image_to_ram(const char *path);

#ifdef __cplusplus
}
#endif
