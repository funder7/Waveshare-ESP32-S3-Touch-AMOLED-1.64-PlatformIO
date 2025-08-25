#include "Arduino.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_sh8601.h"
#include "lcd_config.h"

static const char* TAG = "display_test_sh8601";

static void dcs_read_and_log(esp_lcd_panel_io_handle_t io, uint8_t cmd, size_t len, const char* name)
{
  uint8_t buf[8] = {0};
  if (len > sizeof(buf)) len = sizeof(buf);
  
  // Try different read methods for CO5300
  ESP_LOGI(TAG, "Attempting to read %s (0x%02X)...", name, cmd);
  
  // Method 1: Standard QSPI read with Fast Read command
  uint32_t field1 = (0x0B << 24) | (cmd << 16); // Fast Read + cmd as address
  esp_err_t r1 = esp_lcd_panel_io_rx_param(io, field1, buf, len);
  
  // Method 2: Simple command read
  uint32_t field2 = (0x03 << 24) | cmd; 
  esp_err_t r2 = esp_lcd_panel_io_rx_param(io, field2, buf, len);
  
  // Method 3: Direct command without QSPI encoding
  esp_err_t r3 = esp_lcd_panel_io_rx_param(io, cmd, buf, len);
  
  if (r1 == ESP_OK && buf[0] != 0xFF) {
    ESP_LOGI(TAG, "DCS %s (0x%02X) [Fast Read]: %02X", name, cmd, buf[0]);
  } else if (r2 == ESP_OK && buf[0] != 0xFF) {
    ESP_LOGI(TAG, "DCS %s (0x%02X) [Standard]: %02X", name, cmd, buf[0]);
  } else if (r3 == ESP_OK && buf[0] != 0xFF) {
    ESP_LOGI(TAG, "DCS %s (0x%02X) [Direct]: %02X", name, cmd, buf[0]);
  } else {
    ESP_LOGW(TAG, "DCS %s (0x%02X) read failed or returned 0xFF", name, cmd);
  }
}

#define LCD_HOST    SPI2_HOST

// CO5300 AMOLED initialization sequence - copied from working Arduino demo
static const sh8601_lcd_init_cmd_t amoled_init_cmds[] = 
{
  {0x11, (uint8_t []){0x00}, 0, 80},  // SLPOUT - Exit sleep mode, wait 80ms
  {0xC4, (uint8_t []){0x80}, 1, 0},   // CO5300 specific command
  {0x35, (uint8_t []){0x00}, 1, 0},   // Tearing Effect Line ON
  {0x53, (uint8_t []){0x20}, 1, 1},   // Write Control Display
  {0x63, (uint8_t []){0xFF}, 1, 1},   // CO5300 specific command
  {0x51, (uint8_t []){0x00}, 1, 1},   // Set brightness to 0 initially
  {0x29, (uint8_t []){0x00}, 0, 10},  // DISPON - Display on, wait 10ms
  {0x51, (uint8_t []){0xFF}, 1, 0},   // Set brightness to maximum
};

esp_lcd_panel_handle_t panel_handle = NULL;

void test_display_basic()
{
  ESP_LOGI(TAG, "=== Basic Display Test Started ===");

  // Step 1: Initialize SPI bus
  ESP_LOGI(TAG, "1. Initializing SPI bus...");
  const spi_bus_config_t buscfg = SH8601_PANEL_BUS_QSPI_CONFIG(
    EXAMPLE_PIN_NUM_LCD_PCLK,
    EXAMPLE_PIN_NUM_LCD_DATA0,
    EXAMPLE_PIN_NUM_LCD_DATA1,
    EXAMPLE_PIN_NUM_LCD_DATA2,
    EXAMPLE_PIN_NUM_LCD_DATA3,
    EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8  // Match Arduino exactly
  );
  
  esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "SPI bus initialized successfully");

  // Step 2: Create panel I/O
  ESP_LOGI(TAG, "2. Creating panel I/O...");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  // Use exact Arduino configuration
  const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(
    EXAMPLE_PIN_NUM_LCD_CS,
    NULL,  // No callback for this test
    NULL
  );
  
  ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Panel I/O creation failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Panel I/O created successfully");

  // Test basic communication first
  ESP_LOGI(TAG, "Testing basic QSPI communication...");
  
  // Try reading some basic registers
  dcs_read_and_log(io_handle, 0x04, 1, "ID");
  vTaskDelay(pdMS_TO_TICKS(10));
  dcs_read_and_log(io_handle, 0x0A, 1, "STATUS");
  vTaskDelay(pdMS_TO_TICKS(10));

  // Step 3: Create SH8601 panel
  ESP_LOGI(TAG, "3. Creating SH8601 panel...");
  sh8601_vendor_config_t vendor_config = {
    .init_cmds = amoled_init_cmds,
    .init_cmds_size = sizeof(amoled_init_cmds) / sizeof(amoled_init_cmds[0]),
    .flags = {
      .use_qspi_interface = 1,
    },
  };

  const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .bits_per_pixel = 16,  // RGB565  
    .vendor_config = &vendor_config,
  };

  ret = esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SH8601 panel creation failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "SH8601 panel created successfully");

  // Step 4: Reset panel
  ESP_LOGI(TAG, "4. Resetting panel...");
  ret = esp_lcd_panel_reset(panel_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Panel reset failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Panel reset completed");

  // Step 5: Initialize panel
  ESP_LOGI(TAG, "5. Initializing panel...");
  ret = esp_lcd_panel_init(panel_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Panel initialization failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Panel initialization completed");

  // Probe DCS reads after init
  dcs_read_and_log(io_handle, 0x0B, 1, "RDMADCTL");
  dcs_read_and_log(io_handle, 0x0C, 1, "RDDCOLMOD");
  dcs_read_and_log(io_handle, 0x0A, 1, "STATUS");

  // Step 6: Turn on display
  ESP_LOGI(TAG, "6. Turning on display...");
  ret = esp_lcd_panel_disp_off(panel_handle, false);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Display enable failed: %s", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(TAG, "Display enabled successfully");

  // Probe DCS reads after DISPON
  dcs_read_and_log(io_handle, 0x0A, 1, "STATUS");

  // Quick small-area draw test (40x40 red) to rule out transfer issues
  {
    const int tile_w = 40;
    const int tile_h = 40;
    const int x0 = (EXAMPLE_LCD_H_RES - tile_w) / 2;
    const int y0 = (EXAMPLE_LCD_V_RES - tile_h) / 2;
    uint16_t* tile = (uint16_t*)malloc(tile_w * tile_h * sizeof(uint16_t));
    if (tile) {
      for (int i = 0; i < tile_w * tile_h; ++i) tile[i] = 0xF800; // red
      esp_err_t r = esp_lcd_panel_draw_bitmap(panel_handle, x0, y0, x0 + tile_w, y0 + tile_h, tile);
      ESP_LOGI(TAG, "Small tile draw ret=%s", esp_err_to_name(r));
      free(tile);
      delay(200);
    } else {
      ESP_LOGE(TAG, "Failed to allocate tile buffer");
    }
  }

  ESP_LOGI(TAG, "=== Basic Display Test Completed ===");
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
    
    // Send to display
    esp_err_t ret = esp_lcd_panel_draw_bitmap(
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

void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== AMOLED Display Direct Test ===");
  ESP_LOGI(TAG, "Starting AMOLED display direct test");
  ESP_LOGI(TAG, "Display resolution: %dx%d", EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
  ESP_LOGI(TAG, "Pin configuration - CLK:%d, D0:%d, D1:%d, D2:%d, D3:%d, CS:%d, RST:%d", 
    EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0, EXAMPLE_PIN_NUM_LCD_DATA1,
    EXAMPLE_PIN_NUM_LCD_DATA2, EXAMPLE_PIN_NUM_LCD_DATA3, EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_RST);

  // Test basic display functionality
  test_display_basic();
  
  // Wait a moment, then test color patterns
  // delay(1000);
  // test_color_patterns();
  
  Serial.println("=== Setup Complete ===");
}

void loop()
{
  Serial.println("Test completed. Display should show color patterns.");
  delay(5000);
}