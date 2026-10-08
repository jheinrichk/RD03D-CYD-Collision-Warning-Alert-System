/*
  CYD (ESP32-2432S028R "Cheap Yellow Display") + Ai-Thinker RD-03D
  Automotive Collision Avoidance Alert

  Updated:
  Added narrow azimuth filtering so alerts and MPH display only use
  targets within a forward cone of +/-20 degrees from centerline.

  Updated:
  1. Proximity beeper: rhythmic beep for approaching targets. Volume
     scales with distance: 20% at 8 m rising linearly to 100% at 2 m.
     Gated by its own approach deadband (PROX_MIN_APPROACH) so radar
     sign jitter on a stationary target does not beep. Runs in
     addition to the existing WARNING/ALERT/HIGH_SPEED_FAR thresholds
     and triggers, which are unchanged and take priority on the
     speaker.
  2. Receding targets are now distinguished from approaching ones.
     A target moving away produces a rapid beep-beep-beep pattern.
     Existing thresholds and triggers are unchanged.
  3. Audio now runs through LEDC PWM so duty cycle can control
     volume. tone()/noTone() calls were replaced with buzzer helpers.
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>

// ---------- Pins & UART ----------
static const int RD03D_RX_PIN = 22;
static const int RD03D_TX_PIN = 27;
static const uint32_t RD03D_BAUD = 256000;

#define SPEAKER_PIN 26
#define TFT_BL 21

// ---------- Display ----------
// Panel inversion. Evidence so far: TFT_WHITE fills displayed black
// on this unit and TFT_BLACK displays white, so the library setup is
// already inverting (TFT_INVERSION_ON in User_Setup.h). 0 counters
// that. If colors ever come out reversed after a library change, flip
// this to 1.
#define PANEL_INVERTED 0

// Shows labeled color bars for 2 s at boot so the correct
// PANEL_INVERTED value can be confirmed on sight: the bar labeled
// BLACK must display black and RED must display red. Set to 0 to
// skip once verified.
#define DISPLAY_SELF_TEST 1

TFT_eSPI tft;
static const int16_t SCR_W = 320;
static const int16_t SCR_H = 240;

// ---------- Theme ----------
// Single place to set the background. Black is preferred for night
// driving so the screen does not wash out the windshield.
static const uint16_t UI_BG = TFT_BLACK;  // screen background
static const uint16_t UI_FG = TFT_WHITE;  // text on the background

// ---------- Idle radar view ----------
// Shown only on the black idle screen: no alert, no MPH, no startup.
static const uint16_t PLOT_RANGE_MM    = 9000;   // forward range drawn on screen
static const uint32_t PLOT_UPDATE_MS   = 100;    // dot refresh rate
static const uint32_t STATUS_UPDATE_MS = 250;    // frame-age text refresh rate
static const int      DOT_RADIUS       = 3;
static const uint16_t COLOR_CONE       = 0x2104; // very dark grey cone outline
static const uint16_t COLOR_DOT_IN     = TFT_GREEN;   // target inside +/-20 deg
static const uint16_t COLOR_DOT_OUT    = 0x7BEF;      // target outside the cone
static const uint16_t COLOR_STATUS     = 0x7BEF;      // dim grey status text
static const uint32_t FRAME_TIMEOUT_MS = 600;    // radar considered dead past this
static const uint32_t TRAIL_FADE_MS    = 2000;   // dot persistence, full to black
static const uint8_t  TRAIL_MAX        = 64;     // 3 targets x 20 ticks + margin

// ---------- Calibration ----------
static const uint16_t ALERT_DIST_MM                = 4572;   // 15 ft
static const uint16_t WARNING_DIST_MM              = 7620;   // 25 ft
static const uint16_t HIGH_SPEED_FAR_DIST_MAX_MM   = 8534;   // 28 ft
static const int      HIGH_SPEED_FAR_MIN_SPEED_CMS = -1878;  // >42 mph

static const float TTC_WARNING = 1.5f;
static const float TTC_ALERT   = 1.0f;

static const int MIN_APPROACH_SPEED = -200;
static const uint16_t MIN_DIST_MM = 150;

// ---------- Narrow field filter ----------
static const float MAX_AZIMUTH_DEG = 20.0f;  // +/-20 degrees from straight ahead

// ---------- Flash behavior ----------
static const uint32_t WARNING_FLASH_MS = 250;
static const uint32_t ALERT_FLASH_MS   = 125;

// Diagonal stripe look
static const int STRIPE_THICKNESS = 18;
static const int STRIPE_SPACING   = 30;

// ---------- Klaxon ----------
static const int KLAXON_LOW_FREQ  = 500;
static const int KLAXON_HIGH_FREQ = 1000;
static const uint32_t KLAXON_STEP_MS = 250;
static const uint32_t MIN_ALERT_TONE_MS = 1000;

// ---------- Proximity beeper (approaching targets) ----------
static const uint16_t PROX_FAR_MM   = 8000;  // beeper engages at 8 m, 20% volume
static const uint16_t PROX_NEAR_MM  = 2000;  // 100% volume at 2 m and closer
static const uint8_t  PROX_VOL_FAR  = 20;    // percent
static const uint8_t  PROX_VOL_NEAR = 100;   // percent
static const int      PROX_BEEP_FREQ    = 1200;
static const uint32_t PROX_BEEP_ON_MS   = 100;
static const uint32_t PROX_BEEP_PERIOD_MS = 350; // on + off
// Deadband so radar sign jitter on a stationary target does not beep.
// Set below MIN_APPROACH_SPEED so a slow roll still triggers the beeper.
static const int      PROX_MIN_APPROACH = -30;  // cm/s, about 0.7 mph

// ---------- Receding beeper (targets moving away) ----------
static const int      RECEDE_MIN_SPEED  = 200;   // cm/s away; mirrors MIN_APPROACH_SPEED
static const int      RECEDE_BEEP_FREQ  = 2000;
static const uint32_t RECEDE_ON_MS      = 60;
static const uint32_t RECEDE_GAP_MS     = 60;
static const uint32_t RECEDE_CYCLE_MS   = 760;   // 3x(on+gap) + rest

// ---------- LEDC buzzer with volume control ----------
// tone() has no volume control. The speaker is driven with LEDC PWM;
// duty cycle sets loudness (50% duty = full volume for a square wave).
static const int      BUZZ_LEDC_CH   = 4;   // avoid channels TFT_eSPI may use
static const int      BUZZ_LEDC_RES  = 10;  // bits
static const uint32_t BUZZ_DUTY_MAX  = 512; // 50% of 1023

static int currentBuzzFreq = 0;
static uint8_t currentBuzzVol = 0;

static void buzzerInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttachChannel(SPEAKER_PIN, 2000, BUZZ_LEDC_RES, BUZZ_LEDC_CH);
  ledcWrite(SPEAKER_PIN, 0);
#else
  ledcSetup(BUZZ_LEDC_CH, 2000, BUZZ_LEDC_RES);
  ledcAttachPin(SPEAKER_PIN, BUZZ_LEDC_CH);
  ledcWrite(BUZZ_LEDC_CH, 0);
#endif
}

// volPct: 1-100. Duty is scaled from 0 up to 50% duty.
static void buzzerTone(int freq, uint8_t volPct) {
  if (freq <= 0 || volPct == 0) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(SPEAKER_PIN, 0);
#else
    ledcWrite(BUZZ_LEDC_CH, 0);
#endif
    currentBuzzFreq = 0;
    currentBuzzVol = 0;
    return;
  }
  if (volPct > 100) volPct = 100;
  uint32_t duty = (BUZZ_DUTY_MAX * (uint32_t)volPct) / 100;
  if (duty == 0) duty = 1;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  if (freq != currentBuzzFreq) ledcWriteTone(SPEAKER_PIN, freq);
  ledcWrite(SPEAKER_PIN, duty);
#else
  if (freq != currentBuzzFreq) ledcWriteTone(BUZZ_LEDC_CH, freq);
  ledcWrite(BUZZ_LEDC_CH, duty);
#endif
  currentBuzzFreq = freq;
  currentBuzzVol = volPct;
}

static void buzzerOff() {
  buzzerTone(0, 0);
}

// Linear volume map: 20% at PROX_FAR_MM to 100% at PROX_NEAR_MM.
static uint8_t proximityVolume(uint16_t dist_mm) {
  if (dist_mm >= PROX_FAR_MM)  return PROX_VOL_FAR;
  if (dist_mm <= PROX_NEAR_MM) return PROX_VOL_NEAR;
  uint32_t span   = PROX_FAR_MM - PROX_NEAR_MM;              // 6000
  uint32_t travel = PROX_FAR_MM - dist_mm;                   // 0..6000
  return (uint8_t)(PROX_VOL_FAR +
         (travel * (uint32_t)(PROX_VOL_NEAR - PROX_VOL_FAR)) / span);
}

// ---------- Target struct ----------
struct RD03DTarget {
  bool     valid     = false;
  int16_t  x_mm      = 0;
  int16_t  y_mm      = 0;
  int16_t  speed_cms = 0;
  uint16_t dist_mm   = 0;
};

const uint8_t MULTI_TARGET_CMD[12] = {
  0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01
};

// ---------- FULL RD03DReader class ----------
class RD03DReader {
public:
  bool begin(HardwareSerial& s) {
    ser = &s;
    ser->begin(RD03D_BAUD, SERIAL_8N1, RD03D_RX_PIN, RD03D_TX_PIN);
    reset();
    return true;
  }

  void initMultiTarget() {
    if (!ser) return;
    ser->flush();
    ser->write(MULTI_TARGET_CMD, sizeof(MULTI_TARGET_CMD));
    delay(200);
  }

  bool poll() {
    if (!ser) return false;
    while (ser->available() > 0) {
      uint8_t b = (uint8_t)ser->read();
      if (consumeByte(b)) {
        decodeFrame();
        lastFrameMs = millis();
        return true;
      }
    }
    return false;
  }

  const RD03DTarget& target(uint8_t i) const { return tg[i]; }
  uint32_t getFrameCount() const { return frameCount; }
  uint32_t getLastFrameMs() const { return lastFrameMs; }

private:
  HardwareSerial* ser = nullptr;
  static constexpr uint8_t FRAME_LEN = 30;
  uint8_t buf[FRAME_LEN];
  uint8_t idx = 0;
  RD03DTarget tg[3];
  uint32_t frameCount = 0;
  uint32_t lastFrameMs = 0;

  void reset() {
    idx = 0;
    memset(buf, 0, sizeof(buf));
  }

  static int16_t decodeSigned15(uint16_t raw) {
    int16_t mag = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? mag : -mag;
  }

  bool consumeByte(uint8_t b) {
    switch (idx) {
      case 0:
        if (b == 0xAA) { buf[idx++] = b; }
        return false;

      case 1:
        if (b == 0xFF) { buf[idx++] = b; }
        else {
          idx = (b == 0xAA) ? 1 : 0;
          if (idx) buf[0] = 0xAA;
        }
        return false;

      case 2:
        if (b == 0x03) { buf[idx++] = b; }
        else { idx = 0; }
        return false;

      case 3:
        if (b == 0x00) { buf[idx++] = b; }
        else { idx = 0; }
        return false;

      default:
        buf[idx++] = b;
        if (idx >= FRAME_LEN) {
          bool ok = (buf[FRAME_LEN - 2] == 0x55) && (buf[FRAME_LEN - 1] == 0xCC);
          if (ok) frameCount++;
          idx = 0;
          return ok;
        }
        return false;
    }
  }

  void decodeFrame() {
    for (int i = 0; i < 3; i++) {
      int base = 4 + i * 8;
      uint16_t x_raw = buf[base + 0] | (buf[base + 1] << 8);
      uint16_t y_raw = buf[base + 2] | (buf[base + 3] << 8);
      uint16_t s_raw = buf[base + 4] | (buf[base + 5] << 8);
      uint16_t d_raw = buf[base + 6] | (buf[base + 7] << 8);

      RD03DTarget& t = tg[i];
      t.x_mm      = decodeSigned15(x_raw);
      t.y_mm      = decodeSigned15(y_raw);
      t.speed_cms = decodeSigned15(s_raw);
      t.dist_mm   = d_raw;
      t.valid     = (d_raw != 0) || (x_raw != 0) || (y_raw != 0) || (s_raw != 0);
    }
  }
};

RD03DReader rd;

// ---------- Alert levels ----------
enum AlertLevel { NORMAL, WARNING, ALERT, HIGH_SPEED_FAR };

// ---------- Narrow field helper ----------
static bool targetInNarrowField(const RD03DTarget& t, float* azimuthOut = nullptr) {
  if (!t.valid) return false;
  if (t.y_mm <= 0) return false;

  float azimuthDeg = atan2((float)t.x_mm, (float)t.y_mm) * 180.0f / PI;
  if (azimuthOut) *azimuthOut = azimuthDeg;

  return fabsf(azimuthDeg) <= MAX_AZIMUTH_DEG;
}

// Draw diagonal thick stripes
static void drawDiagonalStripes(uint16_t baseColor, uint16_t stripeColor) {
  tft.fillScreen(baseColor);
  for (int k = -SCR_H; k < SCR_W; k += STRIPE_SPACING) {
    for (int w = 0; w < STRIPE_THICKNESS; w++) {
      int k2 = k + w;
      int x0 = 0, y0 = 0, x1 = 0, y1 = 0;

      if (k2 >= 0 && k2 < SCR_H) {
        x0 = 0; y0 = k2;
      } else if (-k2 >= 0 && -k2 < SCR_W) {
        x0 = -k2; y0 = 0;
      } else {
        continue;
      }

      int yRight  = (SCR_W - 1) + k2;
      int xBottom = (SCR_H - 1) - k2;

      if (yRight >= 0 && yRight < SCR_H) {
        x1 = SCR_W - 1; y1 = yRight;
      } else if (xBottom >= 0 && xBottom < SCR_W) {
        x1 = xBottom; y1 = SCR_H - 1;
      } else {
        continue;
      }

      tft.drawLine(x0, y0, x1, y1, stripeColor);
    }
  }
}

// Dynamic MPH display
static float lastDrawnMPH = -1.0f;
static void invalidateMPH() { lastDrawnMPH = -999.0f; }

static void drawMPH(int16_t speed_cms) {
  float mph = -speed_cms * 0.0223694f;
  if (fabsf(mph - lastDrawnMPH) >= 0.5f) {
    int fontSize = 7 + (abs(speed_cms) / 100);
    if (fontSize > 8) fontSize = 8;

    tft.fillScreen(UI_BG);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(UI_FG, UI_BG);

    for (int dx = -1; dx <= 1; dx++) {
      for (int dy = -1; dy <= 1; dy++) {
        if (dx == 0 && dy == 0) continue;
        tft.drawFloat(mph, 1, SCR_W / 2 + dx, 110 + dy, fontSize);
      }
    }
    tft.drawFloat(mph, 1, SCR_W / 2, 110, fontSize);

    tft.drawString("APPROACHING", SCR_W / 2, 40, 2);
    tft.drawString("MPH", SCR_W / 2, 190, 4);

    lastDrawnMPH = mph;
  }
}

// ---------- Idle radar view ----------
// Top-down plot of raw RD03D targets. Sensor sits at bottom center,
// forward is up. Dots: green inside the +/-20 deg cone, grey outside.
// A dim status line at the bottom edge shows ms since the last good
// frame so a dead or disconnected sensor is obvious.
static const int16_t PLOT_ORIGIN_X = SCR_W / 2;
static const int16_t PLOT_ORIGIN_Y = SCR_H - 14;  // keep clear of status text
static const int16_t PLOT_TOP_Y    = 6;
static const int16_t PLOT_SPAN_PX  = PLOT_ORIGIN_Y - PLOT_TOP_Y;  // 220 px

// ---------- Fading trails ----------
// Phosphor-style persistence: every plotted dot stays on screen and
// dims to black over TRAIL_FADE_MS. New dots draw at full brightness
// on top, so target motion reads as a comet trail.
struct TrailDot {
  bool     active = false;
  int16_t  x = 0;
  int16_t  y = 0;
  uint32_t bornMs = 0;
  bool     inCone = false;
};
static TrailDot trail[TRAIL_MAX];
static int8_t   slotTrailIdx[3] = { -1, -1, -1 };  // newest trail entry per slot

// Scale an RGB565 color by num/den toward black
static uint16_t dimColor(uint16_t c, uint32_t num, uint32_t den) {
  uint32_t r = (c >> 11) & 0x1F;
  uint32_t g = (c >> 5)  & 0x3F;
  uint32_t b =  c        & 0x1F;
  r = (r * num) / den;
  g = (g * num) / den;
  b = (b * num) / den;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

static void trailReset() {
  for (int i = 0; i < TRAIL_MAX; i++) trail[i].active = false;
  for (int i = 0; i < 3; i++) slotTrailIdx[i] = -1;
}

// Add a dot for slot s, or refresh the timestamp if the target has
// not moved so a stationary object stays at full brightness instead
// of stacking duplicate entries.
static void trailAdd(int s, int16_t x, int16_t y, bool inCone) {
  int8_t li = slotTrailIdx[s];
  if (li >= 0 && trail[li].active && trail[li].x == x && trail[li].y == y) {
    trail[li].bornMs = millis();
    trail[li].inCone = inCone;
    return;
  }
  // find a free entry, else steal the oldest
  int idx = -1;
  uint32_t oldest = 0xFFFFFFFF;
  int oldestIdx = 0;
  for (int i = 0; i < TRAIL_MAX; i++) {
    if (!trail[i].active) { idx = i; break; }
    if (trail[i].bornMs < oldest) { oldest = trail[i].bornMs; oldestIdx = i; }
  }
  if (idx < 0) {
    idx = oldestIdx;
    tft.fillCircle(trail[idx].x, trail[idx].y, DOT_RADIUS, UI_BG);
  }
  trail[idx].active = true;
  trail[idx].x = x;
  trail[idx].y = y;
  trail[idx].bornMs = millis();
  trail[idx].inCone = inCone;
  slotTrailIdx[s] = (int8_t)idx;
}

// Repaint all trail dots at their current fade level. Expired dots
// are blacked out and freed. Painted oldest first so fresh dots sit
// on top where trails overlap.
static void trailPaint() {
  uint32_t now = millis();

  int order[TRAIL_MAX];
  int n = 0;
  for (int i = 0; i < TRAIL_MAX; i++) {
    if (!trail[i].active) continue;
    int j = n++;
    while (j > 0 && trail[order[j - 1]].bornMs > trail[i].bornMs) {
      order[j] = order[j - 1];
      j--;
    }
    order[j] = i;
  }

  for (int k = 0; k < n; k++) {
    TrailDot& d = trail[order[k]];
    uint32_t age = now - d.bornMs;
    if (age >= TRAIL_FADE_MS) {
      tft.fillCircle(d.x, d.y, DOT_RADIUS, UI_BG);
      d.active = false;
      continue;
    }
    uint16_t base = d.inCone ? COLOR_DOT_IN : COLOR_DOT_OUT;
    uint16_t c = dimColor(base, TRAIL_FADE_MS - age, TRAIL_FADE_MS);
    tft.fillCircle(d.x, d.y, DOT_RADIUS, c);
  }
}

static void drawIdleCone() {
  // tan(20 deg) = 0.36397; horizontal reach at full span
  int16_t dx = (int16_t)(0.36397f * PLOT_SPAN_PX);  // ~80 px
  tft.drawLine(PLOT_ORIGIN_X, PLOT_ORIGIN_Y, PLOT_ORIGIN_X - dx, PLOT_TOP_Y, COLOR_CONE);
  tft.drawLine(PLOT_ORIGIN_X, PLOT_ORIGIN_Y, PLOT_ORIGIN_X + dx, PLOT_TOP_Y, COLOR_CONE);
}

// Map target slot i to screen. Same mm-per-pixel on both axes so the
// plot keeps true aspect. Returns false if it cannot be plotted.
// Takes the slot index, not the struct, so the Arduino-generated
// prototype does not reference RD03DTarget before its definition.
static bool plotPos(int i, int16_t* px, int16_t* py) {
  const RD03DTarget& t = rd.target(i);
  if (!t.valid || t.y_mm <= 0) return false;
  if (t.y_mm > (int32_t)PLOT_RANGE_MM) return false;

  int32_t x = PLOT_ORIGIN_X + ((int32_t)t.x_mm * PLOT_SPAN_PX) / (int32_t)PLOT_RANGE_MM;
  int32_t y = PLOT_ORIGIN_Y - ((int32_t)t.y_mm * PLOT_SPAN_PX) / (int32_t)PLOT_RANGE_MM;

  if (x < DOT_RADIUS || x > SCR_W - 1 - DOT_RADIUS) return false;
  if (y < PLOT_TOP_Y || y > PLOT_ORIGIN_Y) return false;

  *px = (int16_t)x;
  *py = (int16_t)y;
  return true;
}

static void drawIdleStatus() {
  uint32_t lastFrame = rd.getLastFrameMs();
  char buf[28];
  uint16_t color;

  if (lastFrame == 0) {
    color = TFT_RED;
    snprintf(buf, sizeof(buf), "RD03D: NO DATA");
  } else {
    uint32_t age = millis() - lastFrame;
    if (age >= FRAME_TIMEOUT_MS) {
      color = TFT_RED;
      snprintf(buf, sizeof(buf), "RD03D LOST %lums", (unsigned long)age);
    } else {
      color = COLOR_STATUS;
      snprintf(buf, sizeof(buf), "RD03D %lums", (unsigned long)age);
    }
  }

  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(color, UI_BG);
  tft.setTextPadding(150);  // clears leftover characters when text shortens
  tft.drawString(buf, 2, SCR_H - 1, 1);
  tft.setTextPadding(0);
}

// Full redraw on entering the idle view
static void drawIdleStatic() {
  tft.fillScreen(UI_BG);
  trailReset();
  drawIdleCone();
  drawIdleStatus();
}

// Incremental update while the idle view is showing
static void updateIdleView() {
  static uint32_t lastPlotMs = 0;
  static uint32_t lastStatusMs = 0;

  if (millis() - lastPlotMs >= PLOT_UPDATE_MS) {
    lastPlotMs = millis();

    // Register current targets as fresh trail dots
    for (int i = 0; i < 3; i++) {
      int16_t x, y;
      if (!plotPos(i, &x, &y)) continue;
      trailAdd(i, x, y, targetInNarrowField(rd.target(i)));
    }

    // Repaint every dot at its fade level; expired dots black out
    trailPaint();

    // Restore cone lines where fading or expiring dots crossed them
    drawIdleCone();
  }

  if (millis() - lastStatusMs >= STATUS_UPDATE_MS) {
    lastStatusMs = millis();
    drawIdleStatus();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);

  pinMode(SPEAKER_PIN, OUTPUT);
  noTone(SPEAKER_PIN);

  rd.begin(Serial2);
  rd.initMultiTarget();

  uint32_t startupStart = millis();
  bool audioTestDone = false;
  while (millis() - startupStart < 1000) {
    uint32_t elapsed = millis() - startupStart;
    bool flash = (elapsed / 120) % 2 == 0;

    if (flash) drawDiagonalStripes(TFT_WHITE, TFT_RED);
    else tft.fillScreen(TFT_WHITE);

    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawString("SYSTEM", SCR_W / 2, 80, 4);
    tft.drawString("CHECK", SCR_W / 2, 120, 4);

    if (!audioTestDone && elapsed > 200) {
      int freq = 600 + (elapsed - 200) / 2;
      tone(SPEAKER_PIN, freq);
      audioTestDone = true;
    }
    delay(40);
  }

  noTone(SPEAKER_PIN);
  tft.fillScreen(TFT_WHITE);

  Serial.println("RD-03D collision alert ready");
  Serial.println("Narrow field filter active: +/-20 deg azimuth");
}

// ---------- Loop ----------
void loop() {
  static AlertLevel lastLevel = NORMAL;
  static uint32_t lastFlashToggleMs = 0;
  static bool flashOn = false;

  static uint32_t lastKlaxonStepMs = 0;
  static bool klaxonHigh = false;
  static uint32_t alertToneStartMs = 0;

  static uint32_t lastMPHMs = 0;

  rd.poll();

  const uint32_t FRAME_TIMEOUT_MS = 600;
  bool radarAlive = (millis() - rd.getLastFrameMs()) < FRAME_TIMEOUT_MS;

  AlertLevel currentLevel = NORMAL;
  float minTTC = 999999.0f;
  uint16_t bestDist = 0;
  int16_t bestSpeed = 0;
  bool hasAnyApproachingTarget = false;
  int16_t mphSpeed = 0;

  if (radarAlive) {
    for (int i = 0; i < 3; i++) {
      const auto& t = rd.target(i);
      if (!t.valid) continue;
      if (t.dist_mm <= MIN_DIST_MM) continue;

      float azimuthDeg = 0.0f;
      if (!targetInNarrowField(t, &azimuthDeg)) {
        continue;
      }

      if (t.speed_cms < 0) {
        hasAnyApproachingTarget = true;

        if (mphSpeed == 0 || t.dist_mm < bestDist || bestDist == 0) {
          mphSpeed = t.speed_cms;
        }
      }

      if (t.dist_mm >= ALERT_DIST_MM &&
          t.dist_mm <= HIGH_SPEED_FAR_DIST_MAX_MM &&
          t.speed_cms <= HIGH_SPEED_FAR_MIN_SPEED_CMS) {
        currentLevel = HIGH_SPEED_FAR;
      }

      if (t.speed_cms <= MIN_APPROACH_SPEED) {
        float ttc = (float)t.dist_mm / (float)(-t.speed_cms * 10.0f);
        if (ttc < minTTC) {
          minTTC = ttc;
          bestDist = t.dist_mm;
          bestSpeed = t.speed_cms;
        }
      }
    }

    if (currentLevel != HIGH_SPEED_FAR) {
      if (bestDist > 0 && bestDist <= ALERT_DIST_MM && minTTC < TTC_ALERT) currentLevel = ALERT;
      else if (bestDist > 0 && minTTC < TTC_WARNING) currentLevel = WARNING;
      else currentLevel = NORMAL;
    }
  } else {
    currentLevel = NORMAL;
  }

  if (currentLevel == NORMAL) {
    if (hasAnyApproachingTarget && mphSpeed < 0) {
      drawMPH(mphSpeed);
      lastMPHMs = millis();
    } else if (millis() - lastMPHMs > 800) {
      tft.fillScreen(TFT_WHITE);
      lastMPHMs = 0;
    }
  } else {
    uint32_t flashPeriod = (currentLevel == WARNING) ? WARNING_FLASH_MS : ALERT_FLASH_MS;
    uint16_t stripeColor = TFT_RED;

    if (currentLevel == HIGH_SPEED_FAR) {
      flashPeriod = 180;
      stripeColor = 0xFD20; // ORANGE
    }

    bool needsRedraw = false;
    if (millis() - lastFlashToggleMs >= flashPeriod) {
      lastFlashToggleMs = millis();
      flashOn = !flashOn;
      needsRedraw = true;
    }
    if (currentLevel != lastLevel) {
      needsRedraw = true;
      lastFlashToggleMs = 0;
      flashOn = true;
    }
    if (needsRedraw) {
      if (!flashOn) tft.fillScreen(TFT_WHITE);
      else drawDiagonalStripes(TFT_WHITE, stripeColor);
    }
  }

  if (currentLevel == ALERT || currentLevel == HIGH_SPEED_FAR) {
    if (lastLevel != currentLevel) alertToneStartMs = millis();

    if (currentLevel == HIGH_SPEED_FAR) {
      uint32_t cycle = millis() % 300;
      if (cycle < 120) tone(SPEAKER_PIN, 800);
      else noTone(SPEAKER_PIN);
    } else {
      if (millis() - lastKlaxonStepMs >= KLAXON_STEP_MS) {
        lastKlaxonStepMs = millis();
        klaxonHigh = !klaxonHigh;
        tone(SPEAKER_PIN, klaxonHigh ? KLAXON_HIGH_FREQ : KLAXON_LOW_FREQ);
      }
    }
  } else {
    if (millis() - alertToneStartMs >= MIN_ALERT_TONE_MS) {
      noTone(SPEAKER_PIN);
      klaxonHigh = false;
      lastKlaxonStepMs = millis();
    }
  }

  lastLevel = currentLevel;

  static uint32_t lastDbg = 0;
  if (millis() - lastDbg > 500) {
    lastDbg = millis();
    Serial.printf(
      "Level=%d bestDist=%u mm bestSpeed=%d cm/s minTTC=%.2f frames=%u maxAz=%.1f\n",
      (int)currentLevel, bestDist, bestSpeed, minTTC, rd.getFrameCount(), MAX_AZIMUTH_DEG
    );
  }

  delay(5);
}
