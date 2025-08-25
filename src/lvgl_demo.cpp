#ifdef USE_LVGL_DEMO

#include "display_bsp.h"
#include "FT3168.h"
#include "lvgl.h"
#include "demos/lv_demos.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static esp_lcd_panel_handle_t panel_handle = NULL;
static SemaphoreHandle_t lvgl_mux = NULL;

// LVGL callback functions (extracted from arduino-demo-lvgl/lcd_bsp.c)
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area);
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data);
static void example_increase_lvgl_tick(void *arg);
static void example_lvgl_port_task(void *arg);
static bool example_lvgl_lock(int timeout_ms);
static void example_lvgl_unlock(void);
static void lvgl_init_with_display(void);

// These functions are now called from main.cpp when USE_LVGL_DEMO is defined
void lvgl_demo_setup()
{
  Serial.println("Starting display and LVGL initialization...");
  
  // Initialize display hardware using display_bsp
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_err_t ret = display_bsp_init(&panel_handle, &io_handle);
  if (ret != ESP_OK) {
    Serial.printf("Display initialization failed: %s\n", esp_err_to_name(ret));
    return;
  }
  Serial.println("Display hardware initialized successfully");
  
  // Initialize touch
  Serial.println("Initializing touch...");
  Touch_Init();
  Serial.println("Touch initialized successfully");
  
  // Initialize LVGL
  Serial.println("Initializing LVGL...");
  lvgl_init_with_display();
  Serial.println("LVGL initialized successfully");
  
  Serial.println("LVGL demo setup complete");
}

void lvgl_demo_loop()
{
  Serial.println("LVGL demo looping");
  delay(2000);
  //set_amoled_backlight(0xff);
  //delay(1000);
  //set_amoled_backlight(200);
  //delay(1000);
  //set_amoled_backlight(150);
  //delay(1000);
  //set_amoled_backlight(100);
  //delay(1000);
  //set_amoled_backlight(50);
  //delay(1000);
  //set_amoled_backlight(0);
}

// LVGL initialization function (extracted LVGL parts from arduino-demo-lvgl/lcd_bsp.c)
static void lvgl_init_with_display(void)
{
  static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
  static lv_disp_drv_t disp_drv;      // contains callback functions

  // Initialize LVGL
  lv_init();
  
  // Allocate LVGL buffers
  lv_color_t *buf1 = (lv_color_t*)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1);
  lv_color_t *buf2 = (lv_color_t*)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf2);
  
  // Initialize display buffer
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LVGL_BUF_HEIGHT);
  
  // Initialize display driver
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.rounder_cb = example_lvgl_rounder_cb;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  
  // Register display driver
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  // Initialize touch input device
  static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.disp = disp;
  indev_drv.read_cb = example_lvgl_touch_cb;
  lv_indev_drv_register(&indev_drv);

  // Create and start LVGL timer
  const esp_timer_create_args_t lvgl_tick_timer_args = {
    .callback = &example_increase_lvgl_tick,
    .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

  // Create mutex and task for LVGL
  lvgl_mux = xSemaphoreCreateMutex();
  assert(lvgl_mux);
  xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
  
  // Start LVGL demo
  if (example_lvgl_lock(-1)) {   
    lv_demo_widgets();      /* A widgets example */
    //lv_demo_music();        /* A modern, smartphone-like music player demo. */
    //lv_demo_stress();       /* A stress test for LVGL. */
    //lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */

    // Release the mutex
    example_lvgl_unlock();
  }
}

// LVGL callback implementations (extracted from arduino-demo-lvgl/lcd_bsp.c)
static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
  lv_disp_flush_ready(disp_driver);
  return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
  esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

  display_bsp_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
  lv_disp_flush_ready(drv);
}

static void example_lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
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
  if (win) {
    data->point.x = tp_x;
    data->point.y = tp_y;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void example_increase_lvgl_tick(void *arg)
{
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
  uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
  for (;;) {
    if (example_lvgl_lock(-1)) {
      task_delay_ms = lv_timer_handler();
      example_lvgl_unlock();
    }
    if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
      task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
      task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
    }
    vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
  }
}

static bool example_lvgl_lock(int timeout_ms)
{
  assert(lvgl_mux && "LVGL mutex must be created first");
  const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
  return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

static void example_lvgl_unlock(void)
{
  assert(lvgl_mux && "LVGL mutex must be created first");
  xSemaphoreGive(lvgl_mux);
}

#endif