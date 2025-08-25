#include "Arduino.h"

// Include headers for display test
#ifdef USE_DISPLAY_TEST
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_co5300.h"
#include "lcd_config.h"

extern void test_display_basic();
extern void test_display_alignment(); 
extern void test_color_patterns();
#endif

// Include headers for LVGL demo
#ifdef USE_LVGL_DEMO
#include "FT3168.h"

extern void lvgl_demo_setup();
extern void lvgl_demo_loop();
extern void set_amoled_backlight(uint8_t brightness);
#endif

void setup() {
    Serial.begin(115200);
    delay(1000);
    
#ifdef USE_DISPLAY_TEST
    Serial.println("=== Starting Display Test Mode ===");
    delay(2000);
    
    Serial.println("=== AMOLED Display Direct Test ===");
    ESP_LOGI("display_test_co5300", "Starting AMOLED display direct test");
    ESP_LOGI("display_test_co5300", "Display resolution: %dx%d", EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    ESP_LOGI("display_test_co5300", "Pin configuration - CLK:%d, D0:%d, D1:%d, D2:%d, D3:%d, CS:%d, RST:%d", 
        EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0, EXAMPLE_PIN_NUM_LCD_DATA1,
        EXAMPLE_PIN_NUM_LCD_DATA2, EXAMPLE_PIN_NUM_LCD_DATA3, EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_RST);
    
    // Test basic display functionality
    test_display_basic();
    
    // Test display alignment
    delay(2000);
    test_display_alignment();
    
    // Wait a moment, then test color patterns
    delay(2000);
    test_color_patterns();
    
    Serial.println("=== Display Test Complete ===");
    
#elif defined(USE_LVGL_DEMO)
    Serial.println("=== Starting LVGL Demo Mode ===");
    
    lvgl_demo_setup();
    delay(2000);
    Serial.println("Setup done - LVGL demo initialized");
    
#else
    Serial.println("=== No Mode Selected ===");
    Serial.println("Please build with either 'display-test' or 'lvgl-demo' environment");
    Serial.println("Example: pio run -e display-test");
#endif
}

void loop() {
#ifdef USE_DISPLAY_TEST
    // Display test runs once in setup, just idle here
    delay(5000);
    Serial.println("Display test completed. Looping...");
    
#elif defined(USE_LVGL_DEMO)
    lvgl_demo_loop();
    
#else
    delay(5000);
    Serial.println("No valid environment selected...");
#endif
}