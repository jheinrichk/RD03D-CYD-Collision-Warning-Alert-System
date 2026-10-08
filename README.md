# RD03D + CYD Collision Warning Alert System

Experimental ESP32 forward-collision **warning prototype** combining an Ai-Thinker RD-03D 24 GHz mmWave radar with the ESP32-2432S028R "Cheap Yellow Display" (CYD). The firmware reads up to three radar targets over UART, filters a ±20° forward cone, displays closing speed and issues prioritized visual and audible warnings.

**Current firmware:** black nighttime-oriented user interface, live radar dots with fading trails, target approach/recede audio, an on-screen radar frame-age indicator and a boot-time color-bar test. This is a **warning-only** device: it does not steer, brake or otherwise control a vehicle.

## Features

- **Narrow forward cone:** software azimuth filter, ±20° around the radar centerline, for speed displays and audible/visual warnings.
- **Idle radar display:** black background; dim cone outlines; green targets inside the cone; gray targets outside the cone; fading dots over approximately 2 seconds.
- **Link-health status:** bottom-left frame age in milliseconds; red `RD03D: NO DATA` or `RD03D LOST …ms` when valid frames are missing.
- **Closing-speed view:** approaching target speed in MPH, subject to the forward-cone filter.
- **Alert patterns:** red striped warning/alert views with different flash rates and an orange high-speed/far-range mode.
- **Proximity audio:** 1,200 Hz rhythmic approaching-target beeps, using LEDC PWM for distance-scaled volume.
- **Receding-target audio:** a distinct 2,000 Hz triple-beep pattern.
- **Self-test:** labeled color bars for two seconds, then a flashing `SYSTEM CHECK` display and startup tone.

## Hardware

| Component | Description |
|---|---|
| Controller/display | ESP32-2432S028R CYD with **ILI9341** TFT, 320 × 240 |
| Radar | Ai-Thinker RD-03D 24 GHz multi-target mmWave radar |
| Audio | Speaker/buzzer connected to GPIO26 (check the electrical requirements of the actual device) |
| Power | 5 V supply for RD-03D; share ground with the CYD |

The included `User_Setup.h` is specifically for the supported **ILI9341** CYD. Do **not** apply it to a different CYD display revision or a separate ST7735-based display.

## Wiring

| Radar / peripheral | CYD pin | Direction / purpose |
|---|---|---|
| RD-03D VCC | 5 V | Radar power |
| RD-03D GND | GND | Common ground |
| RD-03D TX | GPIO22 | Radar data to ESP32 RX |
| RD-03D RX | GPIO27 | ESP32 TX to radar |
| Audio output | GPIO26 | PWM audio |
| TFT backlight | GPIO21 | TFT BL |

RD-03D data uses `Serial2` at **256000 baud**. Verify power and I/O voltage compatibility for your exact radar/display hardware. See [wiring.md](wiring.md).

## Flashing with Arduino IDE

1. Install the **ESP32** board package using **Tools → Board → Boards Manager**.
2. Install **TFT_eSPI** using **Sketch → Include Library → Manage Libraries**.
3. **Back up the existing** `Documents/Arduino/libraries/TFT_eSPI/User_Setup.h`. In particular, preserve an existing **ST7735** configuration if another display uses it.
4. Replace that library file with [`config/TFT_eSPI/User_Setup.h`](config/TFT_eSPI/User_Setup.h) from this repository. This config selects the **ILI9341** driver and the CYD HSPI pins.
5. Create an Arduino sketch directory named `rd03d_cyd_collision_alert`. Copy [`src/rd03d_cyd_collision_alert.ino`](src/rd03d_cyd_collision_alert.ino) into that directory, keeping the filename unchanged, and open it in Arduino IDE.
6. Set **Tools → Board → ESP32 Dev Module**. Leave the partition scheme and other board options at their defaults.
7. Select the **CYD's COM port** under **Tools → Port** and upload.
8. Connect the RD-03D as shown above, with its required supply voltage and common ground.

**Keep another backup of the CYD `User_Setup.h` outside the `TFT_eSPI` library directory.** A Library Manager update may overwrite that file and restore an incompatible setup, resulting in an unresponsive or white display.

### Expected boot sequence

1. Five labeled BLACK, RED, GREEN, BLUE and WHITE vertical bars appear for **2 seconds**. The BLACK bar must be black and the RED bar must be red.
2. `SYSTEM CHECK` appears with a flashing red pattern and an audio test tone.
3. The normal idle screen is black with two dim cone lines, fading radar dots and the radar-frame age at the bottom-left.

The sketch ships with `DISPLAY_SELF_TEST = 1` and `PANEL_INVERTED = 0`. If the test bars display incorrect/inverted colors, check the exact ILI9341 board revision and library setup before changing `PANEL_INVERTED`. Once the color test has been confirmed, `DISPLAY_SELF_TEST` may be set to `0` to skip it.

### Basic troubleshooting

| Symptom | Check |
|---|---|
| Solid white / unresponsive display | Reinstall the included ILI9341 `User_Setup.h`; inspect driver selection and HSPI pin definitions; check whether TFT_eSPI updated |
| Reversed colors | Check color bars and `PANEL_INVERTED` after validating the board and driver |
| `RD03D: NO DATA` | Radar 5 V supply, shared GND, crossed TX/RX wiring and 256000-baud interface |
| `RD03D LOST …ms` | Intermittent radar serial frames, power and wiring |
| No buzzer sound | GPIO26 output circuit, speaker/buzzer compatibility and LEDC setup |
| Arduino compile problems | Selected **ESP32 Dev Module** core, `TFT_eSPI` installation and matching library configuration |

## Detection and alert logic

The software computes `atan2(x_mm, y_mm)` and admits targets within **±20°** of centerline for the alert decision and MPH display. The idle screen also plots out-of-cone targets in gray for context. The configured plot range is **9 m**.

| Mode | Screen | Sound |
|---|---|---|
| Idle, no closing target | Black live radar plot, cone and link health | Silent unless a qualifying receding target is present |
| Closing speed, normal level | MPH with black background | Rhythmic proximity beep when qualifying target is within 8 m |
| Warning (TTC under 1.5 s) | Red stripes, 250 ms flash interval | Proximity beeper can continue; no dedicated warning klaxon |
| Alert (TTC under 1.0 s at 4.572 m or nearer) | Red stripes, 125 ms flash interval | Alternating 500/1,000 Hz klaxon |
| High-speed/far | Orange stripes, 180 ms flash interval | Staccato 800 Hz alert |
| Receding target (normal level) | Idle radar screen | Triple 2,000 Hz beep pattern (when no higher-priority sound) |

The code evaluates the **high-speed/far** mode for a target in the 4.572–8.534 m band closing at least 1,878 cm/s (about 42 mph). TTC alert evaluation uses targets closing at least 200 cm/s. A separate proximity audio deadband is **−30 cm/s**, and receding audio requires at least **+200 cm/s**.

**Audio priority:** alert/high-speed-far sound → approaching-target proximity beeper → receding-target triple beep. Proximity beeps start at 8 m, with nominal LEDC duty-scaled level increasing from 20% at 8 m to 100% at 2 m and closer. Percentages refer to programmed PWM duty scaling, **not calibrated acoustic sound-pressure levels**.

Tuning constants are near the top of the sketch. The existing `WARNING_DIST_MM` constant is present for reference but is not currently referenced by the warning decision; the TTC and speed logic above describes implemented behavior.

## Repository contents

- [`src/rd03d_cyd_collision_alert.ino`](src/rd03d_cyd_collision_alert.ino) — Arduino firmware
- [`config/TFT_eSPI/User_Setup.h`](config/TFT_eSPI/User_Setup.h) — tested ILI9341 CYD library configuration
- [`wiring.md`](wiring.md) — wiring and pin reference
- `rd03d_cyd_alert_screen_mockup.png` — earlier design concept, **not a screenshot of the current firmware**
- `LICENSE` — MIT license

## Limitations and safety

**Experimental project only.** Not a certified automotive safety system and not suitable as a substitute for driver attention or vehicle-certified collision-warning/automatic-braking equipment. Radar returns and closing-velocity estimates may be unreliable in particular vehicle, roadside, rain, reflection or multipath conditions. The ±20° filter is **horizontal**; it does not constrain vertical radar sensitivity in software. Mount rigidly and do not obscure the driver's view.

## License

[MIT](LICENSE)
