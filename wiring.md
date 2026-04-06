# Wiring

## CYD to RD03D

| CYD Pin | RD03D Pin | Function |
|---|---|---|
| 5V | VCC | Power |
| GND | GND | Common ground |
| GPIO22 | TX | Radar to ESP32 receive |
| GPIO27 | RX | ESP32 transmit to radar |

## Other CYD pins used

| CYD Pin | Function |
|---|---|
| GPIO26 | Speaker |
| GPIO21 | TFT backlight |

## Notes

- RD03D baud rate is 256000
- Use hardware serial
- Keep ground common
- Mount radar rigidly and keep its face aimed straight ahead
- For tighter vertical rejection, physical shielding is needed
