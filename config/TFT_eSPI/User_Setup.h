// TFT_eSPI User_Setup.h for the CYD (ESP32-2432S028R)
// Restores the known-good configuration for the RD03D collision
// alert project: ILI9341 on ESP32 HSPI pins.
//
// Install: replace the User_Setup.h in your TFT_eSPI library folder
// (typically Documents/Arduino/libraries/TFT_eSPI/User_Setup.h).
// Keep a backup copy outside the library folder: Library Manager
// updates overwrite this file with the ESP8266 default, which is
// exactly what produces the unresponsive solid white screen.

#define USER_SETUP_INFO "CYD ESP32-2432S028R ILI9341"

// ---------- Driver ----------
#define ILI9341_DRIVER

// ---------- ESP32 pins (CYD) ----------
#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1   // display reset tied to ESP32 reset

#define TFT_BL   21   // backlight control
#define TFT_BACKLIGHT_ON HIGH

// ---------- Fonts ----------
#define LOAD_GLCD    // Font 1: original Adafruit 8 pixel font
#define LOAD_FONT2   // Font 2: small 16 pixel font
#define LOAD_FONT4   // Font 4: medium 26 pixel font
#define LOAD_FONT6   // Font 6: large 48 pixel font (numbers)
#define LOAD_FONT7   // Font 7: 7 segment 48 pixel font
#define LOAD_FONT8   // Font 8: large 75 pixel font (numbers)
#define LOAD_GFXFF   // FreeFonts
#define SMOOTH_FONT

// ---------- SPI ----------
#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000
