# RD03D CYD Collision Alert

ESP32 based forward collision warning display using the Cheap Yellow Display (ESP32 2432S028R) and the Ai Thinker RD03D 24 GHz radar module.

This project reads RD03D radar targets over high speed UART, filters them to a narrowed forward cone, shows approaching speed in MPH on the display, and triggers visual and audible alerts when a closing target meets defined thresholds.

## Features

- Narrowed forward detection cone using azimuth filtering
- MPH display for approaching targets
- Warning mode with flashing stripe screen
- Alert mode with faster flashing and klaxon
- High speed far range alert condition
- Multi target RD03D frame decoding
- Designed for the CYD display

## Hardware

- ESP32 2432S028R Cheap Yellow Display
- Ai Thinker RD03D radar
- 5V power for RD03D
- Common ground between CYD and RD03D

## Wiring

### RD03D to CYD
- RD03D VCC to 5V
- RD03D GND to GND
- RD03D TX to GPIO22 on CYD
- RD03D RX to GPIO27 on CYD

### Other pins used
- Speaker pin: GPIO26
- Backlight pin: GPIO21

## Important Notes

- RD03D default baud rate is 256000
- Hardware Serial is used
- RD03D requires 5V supply
- Vertical narrowing is not done in code. Vertical beam control would require physical masking or enclosure shaping

## Detection Logic

The sketch narrows the active detection field by calculating target azimuth from radar coordinates.

- `x_mm` = left or right offset
- `y_mm` = forward distance
- `azimuth = atan2(x_mm, y_mm)`

Only targets within the configured azimuth window are used for:
- MPH display
- warning logic
- alert logic
- high speed far range logic

Current narrow field setting:
- plus or minus 20 degrees from centerline

## Alert Behavior

### Normal
- White screen
- MPH shown for approaching target in the narrowed cone

### Warning
- White and yellow flashing stripe screen
- No sound

### Alert
- White and red flashing stripe screen
- Alternating klaxon tone

### High Speed Far
- Orange stripe alert pattern
- Staccato tone

## Current Thresholds

- Alert distance: 4572 mm
- Warning distance: 7620 mm
- High speed far max distance: 8534 mm
- TTC warning: 1.5 s
- TTC alert: 1.0 s
- Minimum approach speed used for TTC: -200 cm/s
- High speed far trigger speed: -1878 cm/s
- Narrow azimuth filter: plus or minus 20 degrees

## Installation

1. Install Arduino IDE or PlatformIO
2. Install ESP32 board support
3. Install `TFT_eSPI`
4. Configure `TFT_eSPI` for your CYD display
5. Copy the sketch into `src/rd03d_cyd_collision_alert.ino`
6. Flash the CYD
7. Power the RD03D and CYD with common ground

## Usage

Mount the RD03D facing forward on the vehicle and align it close to the vehicle centerline. The software only reacts to targets inside the narrowed forward cone, which reduces unwanted side detections compared to using the full radar field.

## Repo Topics

`esp32`, `arduino`, `radar`, `rd03d`, `collision-warning`, `driver-assist`, `cheap-yellow-display`, `cyd`, `tft-espi`, `vehicle-electronics`

## Safety

> Warning
> This project is experimental and not certified for road safety use. Do not rely on it to avoid collisions.

## Media

Add photos, wiring images, and a short demo GIF in the `docs` folder.

## License

MIT
