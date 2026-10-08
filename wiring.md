# Wiring — ESP32-2432S028R CYD + Ai-Thinker RD-03D

## Radar-to-controller connections

| CYD connection | RD-03D connection | Function |
|---|---|---|
| 5 V | VCC | Power to the RD-03D |
| GND | GND | Common power and signal ground |
| GPIO22 (ESP32 RX) | TX | Radar serial data to CYD |
| GPIO27 (ESP32 TX) | RX | CYD serial transmit to radar |

The firmware initializes ESP32 `Serial2` at **256000 baud** (`SERIAL_8N1`), sends the multi-target-mode command at startup and decodes up to three targets per valid radar frame.

## Other CYD connections

| CYD GPIO | Purpose |
|---|---|
| GPIO26 | PWM output for alert/proximity audio |
| GPIO21 | TFT backlight |

Match the GPIO26 output stage and speaker/buzzer power requirements to the actual hardware; do not assume a bare ESP32 GPIO can safely drive an arbitrary speaker load. Verify the radar UART's I/O voltage levels before connecting.

## Display configuration

This build targets the **ESP32-2432S028R CYD with ILI9341 TFT**. Its included [TFT_eSPI `User_Setup.h`](config/TFT_eSPI/User_Setup.h) is the tested HSPI pin configuration:

| TFT signal | ESP32 GPIO |
|---|---|
| MISO | 12 |
| MOSI | 13 |
| SCLK | 14 |
| CS | 15 |
| DC | 2 |
| RESET | -1 (connected to ESP32 reset) |
| Backlight | 21 |

**Before overwriting** your local TFT_eSPI `User_Setup.h`, back up any configuration for another display, especially ST7735. Keep a second copy of the included CYD config outside the library folder because Library Manager updates can overwrite it. See [README.md](README.md) for the complete Arduino IDE procedure and on-screen color test.

## Physical installation

Mount the radar securely, facing forward and aligned with the vehicle centerline. The firmware filters **horizontal azimuth** to ±20° for warnings; it does not implement vertical-angle rejection. Supply the radar with its required 5 V power and share ground with the ESP32. Keep all wiring secure and protected from shorts, moisture and vibration.
