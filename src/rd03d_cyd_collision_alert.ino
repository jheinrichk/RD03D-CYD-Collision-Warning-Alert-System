/*
  CYD (ESP32-2432S028R "Cheap Yellow Display") + Ai-Thinker RD-03D
  Automotive Collision Avoidance Alert

  Updated:
  Added narrow azimuth filtering so alerts and MPH display only use
  targets within a forward cone of +/-20 degrees from centerline.
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
TFT_eSPI tft;
static const int16_t SCR_W = 320;
static const int16_t SCR_H = 240;

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
static void drawMPH(int16_t speed_cms) {
  static float lastDrawnMPH = -1.0f;

  float mph = -speed_cms * 0.0223694f;
  if (fabsf(mph - lastDrawnMPH) >= 0.5f) {
    int fontSize = 7 + (abs(speed_cms) / 100);
    if (fontSize > 8) fontSize = 8;

    tft.fillScreen(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);

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
