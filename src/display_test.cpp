#include "Arduino.h"
#include "esp_log.h"
#include "display_bsp.h"

static const char* TAG = "display_test_co5300";

static void dcs_read_and_log(esp_lcd_panel_io_handle_t io, uint8_t cmd, size_t len, const char* name)
{
  uint8_t buf[8] = {0};
  if (len > sizeof(buf)) len = sizeof(buf);
  uint32_t field = (0x03 << 24) | cmd; // QSPI Read opcode + cmd (0x03 for compatibility)
  esp_err_t r = esp_lcd_panel_io_rx_param(io, field, buf, len);
  if (r == ESP_OK) {
    if (len == 1) {
      ESP_LOGI(TAG, "DCS %s (0x%02X): %02X", name, cmd, buf[0]);
    } else if (len == 3) {
      ESP_LOGI(TAG, "DCS %s (0x%02X): %02X %02X %02X", name, cmd, buf[0], buf[1], buf[2]);
    } else {
      ESP_LOGI(TAG, "DCS %s (0x%02X): len=%d OK", name, cmd, (int)len);
    }
  } else {
    ESP_LOGE(TAG, "DCS %s (0x%02X) read failed: %s", name, cmd, esp_err_to_name(r));
  }
}

#define LCD_HOST    SPI2_HOST

esp_lcd_panel_handle_t panel_handle = NULL;

void test_display_basic()
{
  ESP_LOGI(TAG, "=== Basic Display Test Started ===");
  ESP_LOGI(TAG, "Display resolution: %dx%d", EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
  ESP_LOGI(TAG, "Pin configuration - CLK:%d, D0:%d, D1:%d, D2:%d, D3:%d, CS:%d, RST:%d", 
    EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0, EXAMPLE_PIN_NUM_LCD_DATA1,
    EXAMPLE_PIN_NUM_LCD_DATA2, EXAMPLE_PIN_NUM_LCD_DATA3, EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_RST);

  // Initialize display using shared display_bsp library
  ESP_LOGI(TAG, "Initializing display using display_bsp...");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_err_t ret = display_bsp_init(&panel_handle, &io_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Display initialization failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Display initialized successfully");

  // Probe DCS reads after init
  dcs_read_and_log(io_handle, 0x0B, 1, "RDMADCTL");
  dcs_read_and_log(io_handle, 0x0C, 1, "RDDCOLMOD");
  dcs_read_and_log(io_handle, 0x0A, 1, "STATUS");

  // Quick small-area draw test (8x8 red) to rule out transfer issues
  {
    const int tile_w = 8;
    const int tile_h = 8;
    const int x0 = (EXAMPLE_LCD_H_RES - tile_w) / 2;
    const int y0 = (EXAMPLE_LCD_V_RES - tile_h) / 2;
    uint16_t* tile = (uint16_t*)malloc(tile_w * tile_h * sizeof(uint16_t));
    if (tile) {
      for (int i = 0; i < tile_w * tile_h; ++i) tile[i] = 0xF800; // red
      esp_err_t r = display_bsp_draw_bitmap(panel_handle, x0, y0, x0 + tile_w, y0 + tile_h, tile);
      ESP_LOGI(TAG, "Small tile draw at (%d,%d) ret=%s", x0, y0, esp_err_to_name(r));
      free(tile);
      delay(1000);
    } else {
      ESP_LOGE(TAG, "Failed to allocate tile buffer");
    }
  }

  // Test corner positions to check alignment
  ESP_LOGI(TAG, "Testing corner positions...");
  {
    const int tile_w = 16;
    const int tile_h = 16;
    uint16_t* tile = (uint16_t*)malloc(tile_w * tile_h * sizeof(uint16_t));
    if (tile) {
      // Fill with white
      for (int i = 0; i < tile_w * tile_h; ++i) tile[i] = 0xFFFF;
      
      // Top-left corner
      display_bsp_draw_bitmap(panel_handle, 0, 0, tile_w, tile_h, tile);
      ESP_LOGI(TAG, "Top-left corner (0,0) drawn");
      delay(500);
      
      // Top-right corner  
      display_bsp_draw_bitmap(panel_handle, EXAMPLE_LCD_H_RES-tile_w, 0, EXAMPLE_LCD_H_RES, tile_h, tile);
      ESP_LOGI(TAG, "Top-right corner (%d,0) drawn", EXAMPLE_LCD_H_RES-tile_w);
      delay(500);
      
      // Bottom-left corner
      display_bsp_draw_bitmap(panel_handle, 0, EXAMPLE_LCD_V_RES-tile_h, tile_w, EXAMPLE_LCD_V_RES, tile);
      ESP_LOGI(TAG, "Bottom-left corner (0,%d) drawn", EXAMPLE_LCD_V_RES-tile_h);
      delay(500);
      
      // Bottom-right corner
      display_bsp_draw_bitmap(panel_handle, EXAMPLE_LCD_H_RES-tile_w, EXAMPLE_LCD_V_RES-tile_h, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, tile);
      ESP_LOGI(TAG, "Bottom-right corner (%d,%d) drawn", EXAMPLE_LCD_H_RES-tile_w, EXAMPLE_LCD_V_RES-tile_h);
      
      free(tile);
      delay(2000);
    }
  }

  ESP_LOGI(TAG, "=== Basic Display Test Completed ===");
}

void test_display_alignment()
{
  ESP_LOGI(TAG, "=== Testing Display Alignment ===");
  
  if (!panel_handle) {
    ESP_LOGE(TAG, "Panel not initialized!");
    return;
  }

  // Test if display has any offset by drawing borders
  size_t total_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
  uint16_t* test_buffer = (uint16_t*)malloc(total_pixels * sizeof(uint16_t));
  
  if (!test_buffer) {
    ESP_LOGE(TAG, "Failed to allocate test buffer");
    return;
  }

  // Fill entire screen with dark blue
  for (size_t i = 0; i < total_pixels; i++) {
    test_buffer[i] = 0x0010; // dark blue
  }

  // Draw white borders to check alignment
  // Top border
  for (int x = 0; x < EXAMPLE_LCD_H_RES; x++) {
    test_buffer[x] = 0xFFFF; // white
  }
  // Bottom border  
  for (int x = 0; x < EXAMPLE_LCD_H_RES; x++) {
    test_buffer[(EXAMPLE_LCD_V_RES-1) * EXAMPLE_LCD_H_RES + x] = 0xFFFF; // white
  }
  // Left border
  for (int y = 0; y < EXAMPLE_LCD_V_RES; y++) {
    test_buffer[y * EXAMPLE_LCD_H_RES] = 0xFFFF; // white
  }
  // Right border
  for (int y = 0; y < EXAMPLE_LCD_V_RES; y++) {
    test_buffer[y * EXAMPLE_LCD_H_RES + (EXAMPLE_LCD_H_RES-1)] = 0xFFFF; // white
  }

  ESP_LOGI(TAG, "Drawing border test pattern...");
  esp_err_t ret = display_bsp_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, test_buffer);
  ESP_LOGI(TAG, "Border test ret=%s", esp_err_to_name(ret));
  
  free(test_buffer);
  delay(3000);
  ESP_LOGI(TAG, "=== Display Alignment Test Completed ===");
}

void test_color_patterns()
{
  ESP_LOGI(TAG, "=== Testing Color Patterns ===");
  
  if (!panel_handle) {
    ESP_LOGE(TAG, "Panel not initialized!");
    return;
  }

  // Test with different solid colors
  uint16_t colors[] = {
    0xF800,  // Red
    0x07E0,  // Green  
    0x001F,  // Blue
    0xFFE0,  // Yellow
    0xF81F,  // Magenta
    0x07FF,  // Cyan
    0xFFFF,  // White
    0x0000   // Black
  };
  
  const char* color_names[] = {
    "Red", "Green", "Blue", "Yellow", "Magenta", "Cyan", "White", "Black"
  };

  size_t total_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
  uint16_t* color_buffer = (uint16_t*)malloc(total_pixels * sizeof(uint16_t));
  
  if (!color_buffer) {
    ESP_LOGE(TAG, "Failed to allocate color buffer");
    return;
  }

  for (int color_idx = 0; color_idx < 8; color_idx++) {
    ESP_LOGI(TAG, "Testing %s color (0x%04X)", color_names[color_idx], colors[color_idx]);
    
    // Fill buffer with current color
    for (size_t i = 0; i < total_pixels; i++) {
      color_buffer[i] = colors[color_idx];
    }
    
    // Send to display - ensure we cover the entire screen
    esp_err_t ret = display_bsp_draw_bitmap(
      panel_handle, 
      0, 0, 
      EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, 
      color_buffer
    );
    
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to draw %s: %s", color_names[color_idx], esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "%s pattern sent successfully", color_names[color_idx]);
    }
    
    // Wait 2 seconds between colors
    delay(2000);
  }
  
  free(color_buffer);
  ESP_LOGI(TAG, "=== Color Pattern Test Completed ===");
}
