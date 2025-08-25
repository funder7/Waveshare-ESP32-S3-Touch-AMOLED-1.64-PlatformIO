# Waveshare ESP32-S3 Touch AMOLED 1.64 Demo - PlatformIO

This is a demo project built from the original Demo provided by Waveshare for Arduino.
The Demo project was using a SH8601 display driver library with a custom initialization sequence, specific for the CO5300 driver.
It contains the board configuration and the minimum setup to run a LVGL demo.

## What has been done

1. PlatformIO doesn't use the same core libraries as Arduino IDE, so I switched to pioarduino in order to use
   the stable platform-espressif32.
2. I developed a simple test program to verify basic communication with the display. After many attempts, the display finally worked (pioarduino was the key).
3. The CO5300 driver for esp-idf [available on Github](https://github.com/espressif/esp-iot-solution/tree/master/components/display/lcd/esp_lcd_co5300) has been imported and used in the project as-is, replacing the SH8601+Custom init sequence.
4. More tests have been added to setup the display alignment correctly: an offset gap has been configured in order to center the displayed image.

