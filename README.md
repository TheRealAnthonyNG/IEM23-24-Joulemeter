# IEM23-24-Joulemeter

Writes to SD card and i2c display.

Made with platformio, can be run on arduino IDE once correct libraries installed.

Not a programmer, a hardware engineer. 

ESP32 board: https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654

## Update for FreeRTOS code:
Connecting SD card causes the ESP32 watchdog to restart the ESP32, so it doesn't actually log data.  
