#include "lcd_bsp.h"
#include "esp_lcd_co5300_my.h"
#include "lcd_config.h"
#include "FT3168.h"
#include "esp_log.h"

static const char *TAG_LCD = "lcd_bsp";
static SemaphoreHandle_t lvgl_mux = NULL; // mutex semaphores
#define LCD_HOST SPI2_HOST

// #define EXAMPLE_Rotate_90
#define SH8601_ID 0x86
#define CO5300_ID 0xff

static esp_lcd_panel_io_handle_t amoled_panel_io_handle = NULL;

// CO5300 initialization commands - simplified generic LCD sequence
static const co5300_lcd_init_cmd_t lcd_init_cmds[] =
    {
        {0x11, NULL, 0, 120},            // SLPOUT - Exit sleep mode, wait 120ms
        {0x3A, (uint8_t[]){0x55}, 1, 0}, // COLMOD - Set pixel format to 16-bit (RGB565)
        {0x36, (uint8_t[]){0x00}, 1, 0}, // MADCTL - Memory access control (normal)
        {0x29, NULL, 0, 10},             // DISPON - Display on, wait 10ms
        {0x51, (uint8_t[]){0xFF}, 1, 0}, // WRDISBV - Set brightness to maximum
};

void lcd_lvgl_Init(void)
{
  ESP_LOGI(TAG_LCD, "=== Starting LCD LVGL Initialization ===");

  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions

  ESP_LOGI(TAG_LCD, "Display resolution: %dx%d, BPP: %d", EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, LCD_BIT_PER_PIXEL);
  ESP_LOGI(TAG_LCD, "SPI pins - CLK:%d, D0:%d, D1:%d, D2:%d, D3:%d, CS:%d, RST:%d",
           EXAMPLE_PIN_NUM_LCD_PCLK, EXAMPLE_PIN_NUM_LCD_DATA0, EXAMPLE_PIN_NUM_LCD_DATA1,
           EXAMPLE_PIN_NUM_LCD_DATA2, EXAMPLE_PIN_NUM_LCD_DATA3, EXAMPLE_PIN_NUM_LCD_CS, EXAMPLE_PIN_NUM_LCD_RST);

  const spi_bus_config_t buscfg = CO5300_PANEL_BUS_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_PCLK,
                                                               EXAMPLE_PIN_NUM_LCD_DATA0,
                                                               EXAMPLE_PIN_NUM_LCD_DATA1,
                                                               EXAMPLE_PIN_NUM_LCD_DATA2,
                                                               EXAMPLE_PIN_NUM_LCD_DATA3,
                                                               EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
  ESP_LOGI(TAG_LCD, "Initializing SPI bus...");
  esp_err_t spi_result = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (spi_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "SPI bus initialization failed: %s", esp_err_to_name(spi_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "SPI bus initialized successfully");
  esp_lcd_panel_io_handle_t io_handle = NULL;

  const esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(EXAMPLE_PIN_NUM_LCD_CS,
                                                                              example_notify_lvgl_flush_ready,
                                                                              &disp_drv);

  co5300_vendor_config_t vendor_config =
      {
          .init_cmds = lcd_init_cmds,
          .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
          .flags =
              {
                  .use_qspi_interface = 1,
              },
      };
  ESP_LOGI(TAG_LCD, "Creating panel I/O interface...");
  esp_err_t io_result = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);
  if (io_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "Panel I/O creation failed: %s", esp_err_to_name(io_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "Panel I/O interface created successfully");

  amoled_panel_io_handle = io_handle;
  esp_lcd_panel_handle_t panel_handle = NULL;
  const esp_lcd_panel_dev_config_t panel_config =
      {
          .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
          // Commented out for ESP-IDF compatibility - rgb_ele_order not available in this version
          //.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
          .bits_per_pixel = LCD_BIT_PER_PIXEL,
          .vendor_config = &vendor_config,
      };
  ESP_LOGI(TAG_LCD, "Creating CO5300 panel...");
  esp_err_t panel_result = esp_lcd_new_panel_co5300(io_handle, &panel_config, &panel_handle);
  if (panel_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "CO5300 panel creation failed: %s", esp_err_to_name(panel_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "CO5300 panel created successfully");

  ESP_LOGI(TAG_LCD, "Resetting panel...");
  esp_err_t reset_result = esp_lcd_panel_reset(panel_handle);
  if (reset_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "Panel reset failed: %s", esp_err_to_name(reset_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "Panel reset completed");

  ESP_LOGI(TAG_LCD, "Initializing panel...");
  esp_err_t init_result = esp_lcd_panel_init(panel_handle);
  if (init_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "Panel initialization failed: %s", esp_err_to_name(init_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "Panel initialization completed");

  ESP_LOGI(TAG_LCD, "Enabling display...");
  esp_err_t disp_result = esp_lcd_panel_disp_off(panel_handle, false);
  if (disp_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "Display enable failed: %s", esp_err_to_name(disp_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "Display enabled successfully");

  ESP_LOGI(TAG_LCD, "Initializing LVGL...");
  lv_init();

  ESP_LOGI(TAG_LCD, "Allocating LVGL buffers - Buffer size: %dx%d pixels", EXAMPLE_LCD_H_RES, EXAMPLE_LVGL_BUF_HEIGHT);
  lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  if (!buf1)
  {
    ESP_LOGE(TAG_LCD, "Failed to allocate LVGL buffer 1");
    return;
  }
  ESP_LOGI(TAG_LCD, "LVGL buffer 1 allocated successfully");

  lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  if (!buf2)
  {
    ESP_LOGE(TAG_LCD, "Failed to allocate LVGL buffer 2");
    return;
  }
  ESP_LOGI(TAG_LCD, "LVGL buffer 2 allocated successfully");
  ESP_LOGI(TAG_LCD, "Configuring LVGL display driver...");
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.rounder_cb = example_lvgl_rounder_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
#ifdef EXAMPLE_Rotate_90
  ESP_LOGI(TAG_LCD, "Enabling 270 degree rotation");
  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_270;
#endif
  ESP_LOGI(TAG_LCD, "Registering LVGL display driver...");
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
  if (!disp)
  {
    ESP_LOGE(TAG_LCD, "Failed to register LVGL display driver");
    return;
  }
  ESP_LOGI(TAG_LCD, "LVGL display driver registered successfully");

  // Touch input is disabled for now
  /*
  static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.disp = disp;
  indev_drv.read_cb = example_lvgl_touch_cb;
  lv_indev_drv_register(&indev_drv);
  */

  ESP_LOGI(TAG_LCD, "Creating LVGL timer...");
  const esp_timer_create_args_t lvgl_tick_timer_args =
      {
          .callback = &example_increase_lvgl_tick,
          .name = "lvgl_tick"};
  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_err_t timer_result = esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  if (timer_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "LVGL timer creation failed: %s", esp_err_to_name(timer_result));
    return;
  }

  timer_result = esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);
  if (timer_result != ESP_OK)
  {
    ESP_LOGE(TAG_LCD, "LVGL timer start failed: %s", esp_err_to_name(timer_result));
    return;
  }
  ESP_LOGI(TAG_LCD, "LVGL timer started successfully");

  ESP_LOGI(TAG_LCD, "Creating LVGL task...");
  lvgl_mux = xSemaphoreCreateMutex(); // mutex semaphores
  if (!lvgl_mux)
  {
    ESP_LOGE(TAG_LCD, "Failed to create LVGL mutex");
    return;
  }

  BaseType_t task_result = xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
  if (task_result != pdPASS)
  {
    ESP_LOGE(TAG_LCD, "Failed to create LVGL task");
    return;
  }
  ESP_LOGI(TAG_LCD, "LVGL task created successfully");

  // Test basic display functionality before starting LVGL demo
  ESP_LOGI(TAG_LCD, "Testing basic display functionality...");

  // Try to fill display with a simple color pattern
  uint16_t test_color = 0xF800; // Red color in RGB565
  size_t total_pixels = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
  uint16_t *test_buffer = malloc(total_pixels * sizeof(uint16_t));
  if (test_buffer)
  {
    // Fill buffer with red color
    for (size_t i = 0; i < total_pixels; i++)
    {
      test_buffer[i] = test_color;
    }

    ESP_LOGI(TAG_LCD, "Sending red test pattern to display...");
    esp_err_t test_result = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, test_buffer);
    if (test_result != ESP_OK)
    {
      ESP_LOGE(TAG_LCD, "Test pattern draw failed: %s", esp_err_to_name(test_result));
    }
    else
    {
      ESP_LOGI(TAG_LCD, "Test pattern sent successfully");
    }
    free(test_buffer);

    // Wait 2 seconds to see the test pattern
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  else
  {
    ESP_LOGE(TAG_LCD, "Failed to allocate test buffer");
  }

  ESP_LOGI(TAG_LCD, "Starting LVGL demo...");
  if (example_lvgl_lock(-1))
  {
    ESP_LOGI(TAG_LCD, "Starting lv_demo_widgets()");
    lv_demo_widgets(); /* A widgets example */
    // lv_demo_music();        /* A modern, smartphone-like music player demo. */
    // lv_demo_stress();       /* A stress test for LVGL. */
    // lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */

    // Release the mutex
    example_lvgl_unlock();
    ESP_LOGI(TAG_LCD, "LVGL demo started successfully");
  }
  else
  {
    ESP_LOGE(TAG_LCD, "Failed to acquire LVGL mutex for demo start");
  }

  ESP_LOGI(TAG_LCD, "=== LCD LVGL Initialization Complete ===");
}

static bool example_lvgl_lock(int timeout_ms)
{
  assert(lvgl_mux && "bsp_display_start must be called first");

  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
  assert(lvgl_mux && "bsp_display_start must be called first");
  xSemaphoreGive(lvgl_mux);
}
static void example_lvgl_port_task(void *arg)
{
  uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
  for (;;)
  {
    if (example_lvgl_lock(-1))
    {
      task_delay_ms = lv_timer_handler();

      example_lvgl_unlock();
    }
    if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
    {
      task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    }
    else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
    {
      task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
}
static void example_increase_lvgl_tick(void *arg)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  static int flush_count = 0;
  flush_count++;

  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
  const int offsetx1 = area->x1; // + 0x14;  // Removed offset for CO5300
  const int offsetx2 = area->x2; // + 0x14;  // Removed offset for CO5300
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;

  // Log first few flush operations for debugging
  if (flush_count <= 10)
  {
    ESP_LOGI(TAG_LCD, "FLUSH[%d]: area(%d,%d)->(%d,%d), size=%dx%d",
             flush_count, offsetx1, offsety1, offsetx2, offsety2,
             offsetx2 - offsetx1 + 1, offsety2 - offsety1 + 1);
  }
  else if (flush_count == 11)
  {
    ESP_LOGI(TAG_LCD, "FLUSH: (further flush operations will not be logged)");
  }

  esp_err_t draw_result = esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
  if (draw_result != ESP_OK && flush_count <= 5)
  {
    ESP_LOGE(TAG_LCD, "FLUSH[%d] ERROR: %s", flush_count, esp_err_to_name(draw_result));
  }
}
void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
  uint16_t x1 = area->x1;
  uint16_t x2 = area->x2;

  uint16_t y1 = area->y1;
  uint16_t y2 = area->y2;

  // round the start of coordinate down to the nearest 2M number
  area->x1 = (x1 >> 1) << 1;
  area->y1 = (y1 >> 1) << 1;
  // round the end of coordinate up to the nearest 2N+1 number
  area->x2 = ((x2 >> 1) << 1) + 1;
  area->y2 = ((y2 >> 1) << 1) + 1;
}
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  uint16_t tp_x, tp_y;
  uint8_t win = getTouch(&tp_x, &tp_y);
  if (win)
  {
    data->point.x = tp_x;
    data->point.y = tp_y;
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

esp_err_t set_amoled_backlight(uint8_t brig)
{
  // Use QSPI write-command opcode (0x02) + command (0x51) in the low 8 bits
  uint32_t lcd_cmd = (0x02U << 24) | 0x51U;
  return esp_lcd_panel_io_tx_param(amoled_panel_io_handle, lcd_cmd, &brig, 1);
}