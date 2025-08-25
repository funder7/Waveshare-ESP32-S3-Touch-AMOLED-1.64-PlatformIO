#ifndef DISPLAY_BSP_H
#define DISPLAY_BSP_H

#include "Arduino.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_co5300.h"
#include "lcd_config.h"

#ifdef USE_LVGL_DEMO
#include "lvgl.h"
#include "demos/lv_demos.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif 

// Basic display initialization - extracted from working display_test.cpp
esp_err_t display_bsp_init(esp_lcd_panel_handle_t *panel_handle, esp_lcd_panel_io_handle_t *io_handle);

// Display operations  
esp_err_t display_bsp_draw_bitmap(esp_lcd_panel_handle_t panel_handle, int x_start, int y_start, int x_end, int y_end, const void *color_data);
esp_err_t display_bsp_fill_screen(esp_lcd_panel_handle_t panel_handle, uint16_t color);

// Display cleanup
esp_err_t display_bsp_deinit(esp_lcd_panel_handle_t panel_handle, esp_lcd_panel_io_handle_t io_handle);

#ifdef __cplusplus
}
#endif

#endif