/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <sys/cdefs.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_log.h"

#include "esp_lcd_co5300_my.h"

#define LCD_OPCODE_WRITE_CMD        (0x02ULL)
#define LCD_OPCODE_READ_CMD         (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR      (0x32ULL)

static const char *TAG = "CO5300";

static esp_err_t panel_co5300_del(esp_lcd_panel_t *panel);
static esp_err_t panel_co5300_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_co5300_init(esp_lcd_panel_t *panel);
static esp_err_t panel_co5300_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_co5300_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_co5300_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_co5300_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_co5300_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_co5300_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    const co5300_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    struct {
        unsigned int use_qspi_interface: 1;
        unsigned int reset_level: 1;
    } flags;
} co5300_panel_t;

esp_err_t esp_lcd_new_panel_co5300(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

    ESP_LOGI(TAG, "Creating new CO5300 panel");
    ESP_LOGI(TAG, "Reset GPIO: %d, BPP: %d", panel_dev_config->reset_gpio_num, panel_dev_config->bits_per_pixel);

    esp_err_t ret = ESP_OK;
    co5300_panel_t *co5300 = NULL;
    co5300 = calloc(1, sizeof(co5300_panel_t));
    ESP_GOTO_ON_FALSE(co5300, ESP_ERR_NO_MEM, err, TAG, "no mem for co5300 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    // Set default color order (RGB)
    co5300->madctl_val = 0;

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        co5300->colmod_val = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        co5300->colmod_val = 0x66;
        fb_bits_per_pixel = 18;
        break;
    case 24: // RGB888
        co5300->colmod_val = 0x77;
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    co5300->io = io;
    co5300->fb_bits_per_pixel = fb_bits_per_pixel;
    co5300->reset_gpio_num = panel_dev_config->reset_gpio_num;
    co5300->flags.reset_level = panel_dev_config->flags.reset_active_high;

    // Save init commands
    if (panel_dev_config->vendor_config) {
        co5300_vendor_config_t *vendor_config = (co5300_vendor_config_t *)panel_dev_config->vendor_config;
        co5300->init_cmds = vendor_config->init_cmds;
        co5300->init_cmds_size = vendor_config->init_cmds_size;
        co5300->flags.use_qspi_interface = vendor_config->flags.use_qspi_interface;
    }

    co5300->base.del = panel_co5300_del;
    co5300->base.reset = panel_co5300_reset;
    co5300->base.init = panel_co5300_init;
    co5300->base.draw_bitmap = panel_co5300_draw_bitmap;
    co5300->base.invert_color = panel_co5300_invert_color;
    co5300->base.set_gap = panel_co5300_set_gap;
    co5300->base.mirror = panel_co5300_mirror;
    co5300->base.swap_xy = panel_co5300_swap_xy;
    co5300->base.disp_off = panel_co5300_disp_on_off;

    *ret_panel = &(co5300->base);
    ESP_LOGD(TAG, "new co5300 panel @%p", co5300);

    return ESP_OK;

err:
    if (co5300) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(co5300);
    }
    return ret;
}

static esp_err_t panel_co5300_del(esp_lcd_panel_t *panel)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);

    if (co5300->reset_gpio_num >= 0) {
        gpio_reset_pin(co5300->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del co5300 panel @%p", co5300);
    free(co5300);
    return ESP_OK;
}

static esp_err_t panel_co5300_reset(esp_lcd_panel_t *panel)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;

    ESP_LOGI(TAG, "Starting CO5300 reset sequence");
    
    // Perform hardware reset
    if (co5300->reset_gpio_num >= 0) {
        ESP_LOGI(TAG, "Performing hardware reset on GPIO %d", co5300->reset_gpio_num);
        gpio_set_level(co5300->reset_gpio_num, co5300->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(co5300->reset_gpio_num, !co5300->flags.reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOGI(TAG, "Hardware reset completed");
    } else {
        ESP_LOGI(TAG, "Performing software reset");
        // Perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_LOGI(TAG, "Software reset completed");
    }

    return ESP_OK;
}

static esp_err_t panel_co5300_init(esp_lcd_panel_t *panel)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;

    ESP_LOGI(TAG, "Starting CO5300 initialization");

    // Send custom initialization commands
    if (co5300->init_cmds && co5300->init_cmds_size) {
        ESP_LOGI(TAG, "Using custom initialization commands (%d commands)", co5300->init_cmds_size);
        for (int i = 0; i < co5300->init_cmds_size; i++) {
            const co5300_lcd_init_cmd_t *cmd = &co5300->init_cmds[i];
            ESP_LOGI(TAG, "Sending command[%d]: 0x%02X, data_bytes: %d", i, cmd->cmd, cmd->data_bytes);
            
            esp_err_t ret = esp_lcd_panel_io_tx_param(io, cmd->cmd, cmd->data, cmd->data_bytes);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Command[%d] 0x%02X failed: %s", i, cmd->cmd, esp_err_to_name(ret));
                return ret;
            }
            ESP_LOGI(TAG, "Command[%d] 0x%02X sent successfully", i, cmd->cmd);
            
            if (cmd->delay_ms > 0) {
                ESP_LOGI(TAG, "Waiting %d ms after command[%d]", cmd->delay_ms, i);
                vTaskDelay(pdMS_TO_TICKS(cmd->delay_ms));
            }
        }
    } else {
        ESP_LOGI(TAG, "Using default CO5300 initialization sequence");
        // Default CO5300 initialization sequence
        ESP_LOGI(TAG, "Sending SLPOUT command");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(120));
        
        ESP_LOGI(TAG, "Sending MADCTL command (value: 0x%02X)", co5300->madctl_val);
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {co5300->madctl_val}, 1), TAG, "send command failed");
        
        ESP_LOGI(TAG, "Sending COLMOD command (value: 0x%02X)", co5300->colmod_val);
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {co5300->colmod_val}, 1), TAG, "send command failed");
        
        ESP_LOGI(TAG, "Sending DISPON command");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_DISPON, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "CO5300 initialization completed successfully");
    return ESP_OK;
}

static esp_err_t panel_co5300_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = co5300->io;

    static int draw_count = 0;
    draw_count++;

    x_start += co5300->x_gap;
    x_end += co5300->x_gap;
    y_start += co5300->y_gap;
    y_end += co5300->y_gap;

    // Log first few draw operations
    if (draw_count <= 5) {
        ESP_LOGI(TAG, "DRAW[%d]: (%d,%d)->(%d,%d), gap(%d,%d), QSPI=%d", 
                 draw_count, x_start, y_start, x_end, y_end, 
                 co5300->x_gap, co5300->y_gap, co5300->flags.use_qspi_interface);
    }

    // Set column address
    esp_err_t ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start >> 8) & 0xFF, x_start & 0xFF, (x_end >> 8) & 0xFF, x_end & 0xFF,
    }, 4);
    if (ret != ESP_OK && draw_count <= 3) {
        ESP_LOGE(TAG, "DRAW[%d] CASET failed: %s", draw_count, esp_err_to_name(ret));
        return ret;
    }

    // Set row address
    ret = esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
        (y_start >> 8) & 0xFF, y_start & 0xFF, (y_end >> 8) & 0xFF, y_end & 0xFF,
    }, 4);
    if (ret != ESP_OK && draw_count <= 3) {
        ESP_LOGE(TAG, "DRAW[%d] RASET failed: %s", draw_count, esp_err_to_name(ret));
        return ret;
    }

    // Send pixel data
    size_t len = (x_end - x_start) * (y_end - y_start) * co5300->fb_bits_per_pixel / 8;
    
    if (draw_count <= 3) {
        ESP_LOGI(TAG, "DRAW[%d]: Sending %d bytes of pixel data", draw_count, len);
    }
    
    if (co5300->flags.use_qspi_interface) {
        ret = esp_lcd_panel_io_tx_color(io, LCD_OPCODE_WRITE_COLOR << 24 | LCD_CMD_RAMWR, color_data, len);
        if (ret != ESP_OK && draw_count <= 3) {
            ESP_LOGE(TAG, "DRAW[%d] QSPI color tx failed: %s", draw_count, esp_err_to_name(ret));
        }
    } else {
        ret = esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);
        if (ret != ESP_OK && draw_count <= 3) {
            ESP_LOGE(TAG, "DRAW[%d] SPI color tx failed: %s", draw_count, esp_err_to_name(ret));
        }
    }

    return ret;
}

static esp_err_t panel_co5300_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_co5300_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;
    if (mirror_x) {
        co5300->madctl_val |= LCD_CMD_MX_BIT;
    } else {
        co5300->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y) {
        co5300->madctl_val |= LCD_CMD_MY_BIT;
    } else {
        co5300->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {co5300->madctl_val}, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_co5300_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;
    if (swap_axes) {
        co5300->madctl_val |= LCD_CMD_MV_BIT;
    } else {
        co5300->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {co5300->madctl_val}, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_co5300_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    co5300->x_gap = x_gap;
    co5300->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_co5300_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    co5300_panel_t *co5300 = __containerof(panel, co5300_panel_t, base);
    esp_lcd_panel_io_handle_t io = co5300->io;
    int command = 0;

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}