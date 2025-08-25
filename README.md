# Waveshare ESP32-S3 Touch AMOLED 1.64
## PlatformIO Demo + Test

This is a demo project built from the original Demo provided by Waveshare for Arduino.
The Demo project was using a SH8601 display driver library with a custom initialization sequence, specific for the CO5300 driver.
It contains the board configuration and the minimum setup to run a LVGL demo.

## Env description

Select the right PlatformIO env depending on what you need to do:

- `display-test`: basic display tests
- `lvgl-demo`: load LVGL demo widgets as in the original Waveshare demo project

### What has been done

1. A new board has been configured, using the same settings as Arduino IDE ([waveshare_esp32_s3_touch_amoled_164.json](boards/waveshare_esp32_s3_touch_amoled_164.json))
2. PlatformIO doesn't use the same core libraries as Arduino IDE, so I switched to pioarduino in order to use
   the stable platform-espressif32.
3. I've developed a simple test program to verify basic communication with the display. After many attempts, the display finally worked (pioarduino was the key).
4. The CO5300 driver for esp-idf [available on Github](https://github.com/espressif/esp-iot-solution/tree/master/components/display/lcd/esp_lcd_co5300) has been imported and [used in the project as-is](libraries\esp_lcd_co5300), replacing the SH8601+Custom init sequence.
5. More tests have been added to setup the display alignment correctly: an offset gap has been configured in order to center the displayed image.
6. Configure two PlatformIO environments to select which program to run: LVGL libraries will be compiled only when needed.
7. Update Waveshare LVGL demo in order to set display offest correctly (already set in [display_bsp.c](libraries/display_bsp/display_bsp.c))