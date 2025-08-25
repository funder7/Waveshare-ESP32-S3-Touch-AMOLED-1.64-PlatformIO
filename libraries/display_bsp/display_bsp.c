#include "display_bsp.h"

#define LCD_HOST SPI2_HOST

esp_err_t display_bsp_init(esp_lcd_panel_handle_t *panel_handle, esp_lcd_panel_io_handle_t *io_handle)
{
    // This is the EXACT working code from display_test.cpp - DO NOT MODIFY
    
    // Step 1: Initialize SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_PCLK;
    buscfg.data0_io_num = EXAMPLE_PIN_NUM_LCD_DATA0;
    buscfg.data1_io_num = EXAMPLE_PIN_NUM_LCD_DATA1;
    buscfg.data2_io_num = EXAMPLE_PIN_NUM_LCD_DATA2;
    buscfg.data3_io_num = EXAMPLE_PIN_NUM_LCD_DATA3;
    buscfg.max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 2;  // 2 bytes per pixel for RGB565
    
    esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 2: Create panel I/O
    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
    io_config.dc_gpio_num = -1;
    io_config.spi_mode = 0;
    io_config.pclk_hz = 40 * 1000 * 1000;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = NULL;  // No callback for basic init
    io_config.user_ctx = NULL;
    io_config.lcd_cmd_bits = 32;
    io_config.lcd_param_bits = 8;
    io_config.flags.quad_mode = true;
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, io_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Skip DCS reads before init to avoid disturbing the panel state
    vTaskDelay(pdMS_TO_TICKS(10));

    // Step 3: Create CO5300 panel
    co5300_vendor_config_t vendor_config = {};
    vendor_config.init_cmds = NULL;
    vendor_config.init_cmds_size = 0;
    vendor_config.flags.use_qspi_interface = 1;

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST;
    panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR;  // RGB order like original code
    panel_config.bits_per_pixel = 16;  // RGB565
    panel_config.vendor_config = &vendor_config;

    ret = esp_lcd_new_panel_co5300(*io_handle, &panel_config, panel_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 4: Reset panel
    vTaskDelay(pdMS_TO_TICKS(300)); // extra power-settle delay at boot
    ret = esp_lcd_panel_reset(*panel_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    // Step 5: Initialize panel
    ret = esp_lcd_panel_init(*panel_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // allow panel to settle after init

    // Set display offset to compensate for CO5300 memory alignment
    ret = esp_lcd_panel_set_gap(*panel_handle, LCD_OFFSET_GAP_X, LCD_OFFSET_GAP_Y);
    if (ret != ESP_OK) {
        // Non-fatal error, continue
    }

    // Step 6: Turn on display
    ret = esp_lcd_panel_disp_on_off(*panel_handle, true);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t display_bsp_draw_bitmap(esp_lcd_panel_handle_t panel_handle, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    return esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_end, y_end, color_data);
}

esp_err_t display_bsp_fill_screen(esp_lcd_panel_handle_t panel_handle, uint16_t color)
{
    size_t total_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
    uint16_t* color_buffer = (uint16_t*)malloc(total_pixels * sizeof(uint16_t));
    
    if (!color_buffer) {
        return ESP_ERR_NO_MEM;
    }
    
    // Fill buffer with color
    for (size_t i = 0; i < total_pixels; i++) {
        color_buffer[i] = color;
    }
    
    esp_err_t ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, color_buffer);
    free(color_buffer);
    
    return ret;
}

esp_err_t display_bsp_deinit(esp_lcd_panel_handle_t panel_handle, esp_lcd_panel_io_handle_t io_handle)
{
    esp_err_t ret = ESP_OK;
    
    if (panel_handle) {
        esp_lcd_panel_del(panel_handle);
    }
    
    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
    }
    
    ret = spi_bus_free(LCD_HOST);
    
    return ret;
}