/*
  AC Infinity Fan Controller for ESP32-S2 (Adafruit Reverse TFT Feather style pinout)

  Wiring based on WireDiagram.txt:
  - GPIO12 -> BS170 gate (controls fan D+ sink through MOSFET)
  - GPIO13 <- fan tach signal (D- / FG)
  - Board powered from 5V output of buck converter

  Serial commands (115200 baud):
  - speed <0-100>
  - +  / inc
  - -  / dec
  - status
  - help

  Hardware buttons:
  - D0: -10%
  - D2: +10%
  - D1: set 50%
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#if !defined(TFT_CS)
#define TFT_CS 7
#endif
#if !defined(TFT_DC)
#define TFT_DC 39
#endif
#if !defined(TFT_RST)
#define TFT_RST 40
#endif
#if !defined(TFT_BACKLITE)
#define TFT_BACKLITE 45
#endif

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// -----------------------------
// Pin and control configuration
// -----------------------------
static const int FAN_PWM_PIN = 12;      // Known-good PWM pin
static const int FAN_TACH_PIN = 13;     // Known-good tach pin

#if defined(D0)
static const int BTN_MINUS_PIN = D0;
#else
static const int BTN_MINUS_PIN = 0;
#endif

#if defined(D1)
static const int BTN_PRESET_PIN = D1;
#else
static const int BTN_PRESET_PIN = 1;
#endif

#if defined(D2)
static const int BTN_PLUS_PIN = D2;
#else
static const int BTN_PLUS_PIN = 2;
#endif

static const uint32_t BUTTON_DEBOUNCE_MS = 35;
static const bool BTN_MINUS_ACTIVE_LOW = true;   // D0
static const bool BTN_PRESET_ACTIVE_LOW = false; // D1
static const bool BTN_PLUS_ACTIVE_LOW = false;   // D2

// Match known-working ESPHome setup (~4.9 kHz on D+ line).
static const int PWM_FREQ_HZ = 4882;
static const int PWM_CHANNEL = 0;
static const int PWM_RES_BITS = 10;     // 10-bit: 0..1023

// With your MOSFET sink circuit, HIGH on GPIO sinks fan D+ to GND.
// Many 4-wire style control lines are active-high at the fan side,
// so sink duty can need to be inverted. Leave true first; if behavior
// is backwards, set to false.
static const bool PWM_ACTIVE_LOW_AT_FAN_LINE = true;

// Keep this false for AC Infinity D+ control to preserve true 0..100 mapping.
static const bool KEEP_PWM_PULSING_AT_MAX = false;
// When true, any non-zero speed request is forced to a pulsing duty (not solid high/low).
static const bool AVOID_SOLID_PWM_FOR_NONZERO_REQUEST = true;

// Calibrated endpoints for inverted PWM mode on this fan/controller pair.
// Lower effective % is faster. Keep top-end away from the shutdown dead-zone.
static const int INVERTED_EFFECTIVE_AT_MAX = 12; // user 100%
static const int INVERTED_EFFECTIVE_AT_MIN = 85; // user MIN_RUN_PERCENT

// Below this point the fan may stall. Keep the output above a small floor
// whenever the user asks for a nonzero speed.
static const int MIN_RUN_PERCENT = 15;

// Tach input: 3 pulses per fan revolution.
static const float TACH_PULSES_PER_REV = 3.0f;
// Reject very fast edges so noise does not look like real tach pulses.
static const uint32_t TACH_MIN_EDGE_US = 5000;
static const float MAX_REASONABLE_RPM = 4000.0;
static const uint32_t RPM_SAMPLE_MS = 5000;
static const uint32_t RPM_NO_PULSE_TIMEOUT_MS = 3000;
static const bool USE_INTERNAL_TACH_PULLUP = false;

static const uint32_t UART_SCAN_BAUDS[] = {9600, 10000, 10400, 11000, 11520, 19200, 38400, 57600, 115200};
static const bool UART_SCAN_INCLUDE_INVERTED = false;

volatile uint32_t tachPulseCount = 0;
volatile uint32_t tachLastCountedRiseUs = 0;
uint32_t lastRpmSampleMs = 0;
uint32_t lastPulseSnapshot = 0;
uint32_t lastPulseSeenMs = 0;
uint32_t lastDeltaPulses = 0;
float lastRawRpm = 0.0f;
float currentRpm = 0.0f;
int targetSpeedPercent = 40;
bool rawDutyOverrideEnabled = false;
uint32_t rawDutyOverride = 0;

uint32_t lastUiDrawMs = 0;
int lastUiSpeed = -1;
int lastUiRpm = -1;
uint32_t lastUiTachCount = 0;

bool minusPressedPrev = false;
bool presetPressedPrev = false;
bool plusPressedPrev = false;
uint32_t lastButtonEdgeMs = 0;

bool setupFanPwm() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  return ledcAttachChannel(FAN_PWM_PIN, PWM_FREQ_HZ, PWM_RES_BITS, PWM_CHANNEL);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL);
  return true;
#endif
}

void writeFanDuty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(FAN_PWM_PIN, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

void IRAM_ATTR onTachPulse() {
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - tachLastCountedRiseUs) < TACH_MIN_EDGE_US) {
    return;
  }

  tachLastCountedRiseUs = nowUs;
  tachPulseCount++;
}

int userPercentToEffectivePercent(int percent);

uint32_t percentToDuty(int percent) {
  const int effectivePercent = userPercentToEffectivePercent(percent);

  const uint32_t maxDuty = (1u << PWM_RES_BITS) - 1u;
  uint32_t duty = (uint32_t)((effectivePercent * (int)maxDuty) / 100);

  // Keep PWM toggling for any non-zero request; solid levels can stop some fans.
  if (AVOID_SOLID_PWM_FOR_NONZERO_REQUEST && percent > 0) {
    if (duty == 0u) {
      duty = 1u;
    } else if (duty >= maxDuty) {
      duty = maxDuty - 1u;
    }
  }

  return duty;
}

int userPercentToEffectivePercent(int percent) {
  percent = constrain(percent, 0, 100);

  int effectivePercent = percent;
  if (PWM_ACTIVE_LOW_AT_FAN_LINE) {
    if (percent <= 0) {
      effectivePercent = 100;
    } else {
      int request = percent;
      if (request < MIN_RUN_PERCENT) {
        request = MIN_RUN_PERCENT;
      }

      const float spanReq = (float)(100 - MIN_RUN_PERCENT);
      const float t = spanReq > 0.0f ? (float)(request - MIN_RUN_PERCENT) / spanReq : 1.0f;
      const float eff = (float)INVERTED_EFFECTIVE_AT_MIN +
                        ((float)(INVERTED_EFFECTIVE_AT_MAX - INVERTED_EFFECTIVE_AT_MIN) * t);
      effectivePercent = (int)(eff + 0.5f);
    }
  } else if (KEEP_PWM_PULSING_AT_MAX && percent >= 100) {
    effectivePercent = 99;
  } else {
    if (percent > 0 && percent < MIN_RUN_PERCENT) {
      percent = MIN_RUN_PERCENT;
    }
    effectivePercent = percent;
  }

  return constrain(effectivePercent, 0, 100);
}

void applyFanSpeed(int percent) {
  rawDutyOverrideEnabled = false;
  targetSpeedPercent = constrain(percent, 0, 100);
  const int effectivePercent = userPercentToEffectivePercent(targetSpeedPercent);
  const uint32_t duty = percentToDuty(targetSpeedPercent);
  writeFanDuty(duty);

  Serial.print("PWM map: request ");
  Serial.print(targetSpeedPercent);
  Serial.print("% -> effective ");
  Serial.print(effectivePercent);
  Serial.print("% -> duty ");
  Serial.println(duty);
}

void applyRawDuty(uint32_t duty) {
  const uint32_t maxDuty = (1u << PWM_RES_BITS) - 1u;
  rawDutyOverride = constrain(duty, 0u, maxDuty);
  rawDutyOverrideEnabled = true;
  writeFanDuty(rawDutyOverride);

  Serial.print("PWM raw override: duty ");
  Serial.print(rawDutyOverride);
  Serial.print("/");
  Serial.println(maxDuty);
}

void updateRpm() {
  const uint32_t now = millis();
  const uint32_t intervalMs = now - lastRpmSampleMs;

  if (intervalMs < RPM_SAMPLE_MS) {
    return;
  }

  uint32_t pulseSnapshot;
  noInterrupts();
  pulseSnapshot = tachPulseCount;
  interrupts();

  const uint32_t deltaPulses = pulseSnapshot - lastPulseSnapshot;
  lastDeltaPulses = deltaPulses;
  lastPulseSnapshot = pulseSnapshot;
  lastRpmSampleMs = now;

  if (deltaPulses == 0) {
    if ((now - lastPulseSeenMs) >= RPM_NO_PULSE_TIMEOUT_MS) {
      currentRpm = 0.0f;
    }
    return;
  }

  lastPulseSeenMs = now;

  if (intervalMs > 0) {
    // With a 5-second window and 3 pulses/rev, RPM = pulseCount * 4.
    float rpmCalc = deltaPulses * 4.0f;
    if (rpmCalc > MAX_REASONABLE_RPM) {
      rpmCalc = MAX_REASONABLE_RPM;
    }
    lastRawRpm = rpmCalc;
    currentRpm = rpmCalc;
  }
}

void printStatus() {
  Serial.println();
  Serial.println("--- AC Infinity Fan Status ---");
  Serial.print("Target speed (%): ");
  Serial.println(targetSpeedPercent);
  Serial.print("Measured RPM: ");
  Serial.println((int)currentRpm);
  Serial.print("PWM duty raw: ");
  Serial.println(percentToDuty(targetSpeedPercent));
  Serial.print("Raw duty override: ");
  if (rawDutyOverrideEnabled) {
    Serial.println(rawDutyOverride);
  } else {
    Serial.println("off");
  }
  Serial.println("------------------------------");
}

void dumpRawTach(uint32_t durationMs) {
  durationMs = constrain(durationMs, 200u, 10000u);

  Serial.println();
  Serial.print("Raw tach capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");

  const uint32_t startUs = micros();
  uint32_t lastEdgeUs = startUs;
  uint32_t nowUs = startUs;

  int lastState = digitalRead(FAN_TACH_PIN);
  uint32_t highUs = 0;
  uint32_t lowUs = 0;
  uint32_t transitions = 0;
  uint32_t risingEdges = 0;
  uint32_t fallingEdges = 0;
  uint32_t minSegmentUs = 0xFFFFFFFFu;
  uint32_t maxSegmentUs = 0;

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();

    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;
      if (lastState == HIGH) {
        highUs += segUs;
      } else {
        lowUs += segUs;
      }

      if (segUs < minSegmentUs) {
        minSegmentUs = segUs;
      }
      if (segUs > maxSegmentUs) {
        maxSegmentUs = segUs;
      }

      transitions++;
      if (state == HIGH) {
        risingEdges++;
      } else {
        fallingEdges++;
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  nowUs = micros();
  const uint32_t tailUs = nowUs - lastEdgeUs;
  if (lastState == HIGH) {
    highUs += tailUs;
  } else {
    lowUs += tailUs;
  }

  const uint32_t totalUs = nowUs - startUs;
  const float highPct = totalUs > 0 ? (100.0f * (float)highUs / (float)totalUs) : 0.0f;
  const float lowPct = 100.0f - highPct;
  const float risingHz = totalUs > 0 ? ((float)risingEdges * 1000000.0f / (float)totalUs) : 0.0f;
  const float estRpm = (risingHz * 60.0f) / TACH_PULSES_PER_REV;

  Serial.print("--- Raw Tach Capture (pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.println(") ---");
  Serial.print("Transitions: ");
  Serial.println(transitions);
  Serial.print("Rising edges: ");
  Serial.println(risingEdges);
  Serial.print("Falling edges: ");
  Serial.println(fallingEdges);
  Serial.print("High time (%): ");
  Serial.println(highPct, 2);
  Serial.print("Low time (%): ");
  Serial.println(lowPct, 2);

  if (transitions > 0) {
    Serial.print("Min segment (us): ");
    Serial.println(minSegmentUs);
    Serial.print("Max segment (us): ");
    Serial.println(maxSegmentUs);
  } else {
    Serial.println("No edges detected.");
  }

  Serial.print("Edge freq (Hz, rising): ");
  Serial.println(risingHz, 2);
  Serial.print("Est RPM from raw edges: ");
  Serial.println(estRpm, 1);
  Serial.println("-------------------------");
}

void dumpEdgeTimings(uint32_t durationMs) {
  durationMs = constrain(durationMs, 200u, 5000u);

  Serial.println();
  Serial.print("Edge timing capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  uint32_t transitions = 0;

  Serial.print("Start state: ");
  Serial.println(lastState == HIGH ? "HIGH" : "LOW");
  Serial.println("Segments (state:duration_us), first 120 transitions:");

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;
      Serial.print(lastState == HIGH ? "H:" : "L:");
      Serial.print(segUs);
      Serial.print(' ');

      transitions++;
      if ((transitions % 12u) == 0u) {
        Serial.println();
      }
      if (transitions >= 120u) {
        Serial.println();
        Serial.println("(truncated)");
        break;
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  nowUs = micros();
  const uint32_t tailUs = nowUs - lastEdgeUs;
  Serial.print("Tail ");
  Serial.print(lastState == HIGH ? "H:" : "L:");
  Serial.println(tailUs);

  Serial.print("Transitions captured: ");
  Serial.println(transitions);

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);
  Serial.println("Edge timing capture complete. Tach interrupt restored.");
}

void dumpEdgeHistogram(uint32_t durationMs) {
  durationMs = constrain(durationMs, 200u, 5000u);

  static const uint32_t BIN_US = 10;
  static const uint32_t MAX_BIN_US = 4000;
  static const uint32_t BIN_COUNT = (MAX_BIN_US / BIN_US) + 1;
  uint16_t bins[BIN_COUNT] = {0};
  uint32_t longSegments = 0;
  uint32_t glitchSegments = 0;

  Serial.println();
  Serial.print("Edge histogram on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  uint32_t transitions = 0;

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;
      if (segUs < 5u) {
        glitchSegments++;
      } else if (segUs > MAX_BIN_US) {
        longSegments++;
      } else {
        const uint32_t bin = segUs / BIN_US;
        if (bin < BIN_COUNT && bins[bin] < 65535u) {
          bins[bin]++;
        }
      }

      transitions++;
      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  uint32_t topBin[6] = {0, 0, 0, 0, 0, 0};
  uint16_t topCount[6] = {0, 0, 0, 0, 0, 0};

  for (uint32_t b = 0; b < BIN_COUNT; ++b) {
    const uint16_t c = bins[b];
    if (c == 0) {
      continue;
    }
    for (int i = 0; i < 6; ++i) {
      if (c > topCount[i]) {
        for (int j = 5; j > i; --j) {
          topCount[j] = topCount[j - 1];
          topBin[j] = topBin[j - 1];
        }
        topCount[i] = c;
        topBin[i] = b;
        break;
      }
    }
  }

  Serial.print("Transitions: ");
  Serial.println(transitions);
  Serial.print("Glitch segments (<5us): ");
  Serial.println(glitchSegments);
  Serial.print("Long segments (>");
  Serial.print(MAX_BIN_US);
  Serial.print("us): ");
  Serial.println(longSegments);
  Serial.println("Top segment bins (us -> count):");

  for (int i = 0; i < 6; ++i) {
    if (topCount[i] == 0) {
      continue;
    }
    const uint32_t centerUs = (topBin[i] * BIN_US) + (BIN_US / 2u);
    Serial.print("  ");
    Serial.print(centerUs);
    Serial.print("us -> ");
    Serial.println(topCount[i]);
  }

  if (topCount[0] > 0) {
    const uint32_t bitUs = (topBin[0] * BIN_US) + (BIN_US / 2u);
    const float estBaud = bitUs > 0 ? (1000000.0f / (float)bitUs) : 0.0f;
    Serial.print("Estimated base timing: ~");
    Serial.print(bitUs);
    Serial.print("us (about ");
    Serial.print(estBaud, 1);
    Serial.println(" baud)");
  }

  Serial.println("Edge histogram complete. Tach interrupt restored.");
}

void dumpPulseProtocol(uint32_t durationMs) {
  durationMs = constrain(durationMs, 200u, 5000u);

  // Based on measured bins near 500us and 1000us.
  static const uint32_t GLITCH_US = 80;
  static const uint32_t GAP_US = 3000;
  static const uint32_t SHORT_MAX_US = 750;
  static const uint32_t LONG_MAX_US = 1800;

  uint32_t shortCount = 0;
  uint32_t longCount = 0;
  uint32_t gapCount = 0;
  uint32_t glitchCount = 0;
  uint32_t otherCount = 0;

  Serial.println();
  Serial.print("Pulse protocol capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Tokens: S~500us, L~1000us, G>3000us, X=other");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  uint32_t tokenPrinted = 0;
  uint32_t transitions = 0;

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      char token = 'X';
      if (segUs < GLITCH_US) {
        token = 'g';
        glitchCount++;
      } else if (segUs > GAP_US) {
        token = 'G';
        gapCount++;
      } else if (segUs <= SHORT_MAX_US) {
        token = 'S';
        shortCount++;
      } else if (segUs <= LONG_MAX_US) {
        token = 'L';
        longCount++;
      } else {
        token = 'X';
        otherCount++;
      }

      if (tokenPrinted < 240u) {
        Serial.print(token);
        tokenPrinted++;
        if ((tokenPrinted % 60u) == 0u) {
          Serial.println();
        }
      }

      transitions++;
      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  if ((tokenPrinted % 60u) != 0u) {
    Serial.println();
  }
  if (tokenPrinted >= 240u) {
    Serial.println("(token stream truncated)");
  }

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Transitions: ");
  Serial.println(transitions);
  Serial.print("S count: ");
  Serial.println(shortCount);
  Serial.print("L count: ");
  Serial.println(longCount);
  Serial.print("G count: ");
  Serial.println(gapCount);
  Serial.print("glitch count: ");
  Serial.println(glitchCount);
  Serial.print("other count: ");
  Serial.println(otherCount);
  Serial.println("Pulse protocol capture complete. Tach interrupt restored.");
}

void dumpFrameSummary(uint32_t durationMs) {
  durationMs = constrain(durationMs, 200u, 5000u);

  static const uint32_t GLITCH_US = 80;
  static const uint32_t GAP_US = 3000;
  static const uint32_t SHORT_MAX_US = 750;
  static const uint32_t LONG_MAX_US = 1800;
  static const uint32_t MAX_FRAME_LEN = 96;
  static const uint32_t MAX_FRAMES = 10;

  Serial.println();
  Serial.print("Frame summary on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Frames split by gap > 3000us.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);

  char frameBuf[MAX_FRAME_LEN + 1];
  uint32_t frameLen = 0;
  uint32_t framesPrinted = 0;
  uint32_t transitions = 0;
  uint32_t glitchCount = 0;

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;
      transitions++;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        if (frameLen > 0 && framesPrinted < MAX_FRAMES) {
          frameBuf[frameLen] = '\0';
          Serial.print("F");
          Serial.print(framesPrinted + 1);
          Serial.print(" len=");
          Serial.print(frameLen);
          Serial.print(" : ");
          Serial.println(frameBuf);
          framesPrinted++;
        }
        frameLen = 0;
      } else {
        char token = 'X';
        if (segUs <= SHORT_MAX_US) {
          token = 'S';
        } else if (segUs <= LONG_MAX_US) {
          token = 'L';
        }

        if (frameLen < MAX_FRAME_LEN) {
          frameBuf[frameLen++] = token;
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  // Flush any trailing partial frame.
  if (frameLen > 0 && framesPrinted < MAX_FRAMES) {
    frameBuf[frameLen] = '\0';
    Serial.print("F");
    Serial.print(framesPrinted + 1);
    Serial.print(" len=");
    Serial.print(frameLen);
    Serial.print(" : ");
    Serial.println(frameBuf);
    framesPrinted++;
  }

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Transitions: ");
  Serial.println(transitions);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Frames shown: ");
  Serial.println(framesPrinted);
  Serial.println("Frame summary complete. Tach interrupt restored.");
}

uint32_t decodeBitsFromTokens(const char* tokens, uint32_t tokenLen, uint8_t* bits, uint32_t bitsMax) {
  uint32_t bitCount = 0;
  uint32_t i = 0;
  while (i + 1 < tokenLen && bitCount < bitsMax) {
    if (tokens[i] == 'S' && (tokens[i + 1] == 'S' || tokens[i + 1] == 'L')) {
      bits[bitCount++] = (tokens[i + 1] == 'L') ? 1u : 0u;
      i += 2;
    } else {
      i++;
    }
  }
  return bitCount;
}

void addWordHit(uint16_t word, uint16_t* words, uint16_t* counts, uint32_t maxItems) {
  for (uint32_t i = 0; i < maxItems; ++i) {
    if (counts[i] == 0) {
      words[i] = word;
      counts[i] = 1;
      return;
    }
    if (words[i] == word) {
      if (counts[i] < 65535u) {
        counts[i]++;
      }
      return;
    }
  }
}

void dumpDecode16(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 6000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_FRAME_BITS = 64;
  static const uint32_t MAX_UNIQUE_WORDS = 96;

  uint16_t wordsA[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsA[MAX_UNIQUE_WORDS] = {0};
  uint16_t wordsAInv[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsAInv[MAX_UNIQUE_WORDS] = {0};
  uint16_t wordsB[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsB[MAX_UNIQUE_WORDS] = {0};
  uint16_t wordsBInv[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsBInv[MAX_UNIQUE_WORDS] = {0};

  uint32_t totalFrames = 0;
  uint32_t decodedFramesA = 0;
  uint32_t decodedFramesB = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredSegments = 0;

  Serial.println();
  Serial.print("Decode16 capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Decoding two models:");
  Serial.println("  A: HIGH~500us pulse then LOW gap (500=0,1000=1)");
  Serial.println("  B: LOW~500us pulse then HIGH gap (500=0,1000=1)");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;
  bool armB = false;

  uint8_t frameBitsA[MAX_FRAME_BITS] = {0};
  uint8_t frameBitsB[MAX_FRAME_BITS] = {0};
  uint32_t frameBitCountA = 0;
  uint32_t frameBitCountB = 0;

  auto flushFrame = [&]() {
    if (frameBitCountA >= 16u || frameBitCountB >= 16u) {
      totalFrames++;
    }

    if (frameBitCountA >= 16u) {
      decodedFramesA++;
      for (uint32_t off = 0; off + 16u <= frameBitCountA; ++off) {
        uint16_t w = 0;
        uint16_t wi = 0;
        for (uint32_t b = 0; b < 16u; ++b) {
          const uint8_t bit = frameBitsA[off + b] ? 1u : 0u;
          w = (uint16_t)((w << 1) | bit);
          wi = (uint16_t)((wi << 1) | (bit ? 0u : 1u));
        }

        if (w != 0x0000u && w != 0xFFFFu) {
          addWordHit(w, wordsA, countsA, MAX_UNIQUE_WORDS);
        }
        if (wi != 0x0000u && wi != 0xFFFFu) {
          addWordHit(wi, wordsAInv, countsAInv, MAX_UNIQUE_WORDS);
        }
      }
    }

    if (frameBitCountB >= 16u) {
      decodedFramesB++;
      for (uint32_t off = 0; off + 16u <= frameBitCountB; ++off) {
        uint16_t w = 0;
        uint16_t wi = 0;
        for (uint32_t b = 0; b < 16u; ++b) {
          const uint8_t bit = frameBitsB[off + b] ? 1u : 0u;
          w = (uint16_t)((w << 1) | bit);
          wi = (uint16_t)((wi << 1) | (bit ? 0u : 1u));
        }

        if (w != 0x0000u && w != 0xFFFFu) {
          addWordHit(w, wordsB, countsB, MAX_UNIQUE_WORDS);
        }
        if (wi != 0x0000u && wi != 0xFFFFu) {
          addWordHit(wi, wordsBInv, countsBInv, MAX_UNIQUE_WORDS);
        }
      }
    }

    frameBitCountA = 0;
    frameBitCountB = 0;
    armA = false;
    armB = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        // Long segment (either polarity) separates repeated frames.
        flushFrame();
      } else {
        if (lastState == HIGH) {
          // Segment was HIGH.
          // Model B bit gap happens on HIGH, armed by prior LOW pulse.
          if (armB) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (frameBitCountB < MAX_FRAME_BITS) {
                frameBitsB[frameBitCountB++] = 0u;
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (frameBitCountB < MAX_FRAME_BITS) {
                frameBitsB[frameBitCountB++] = 1u;
              }
            } else {
              ignoredSegments++;
            }
            armB = false;
          }

          // Model A pulse arm happens on HIGH pulse.
          if (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US) {
            armA = true;
          } else {
            armA = false;
          }
        } else {
          // Segment was LOW.
          // Model A bit gap happens on LOW, armed by prior HIGH pulse.
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (frameBitCountA < MAX_FRAME_BITS) {
                frameBitsA[frameBitCountA++] = 0u;
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (frameBitCountA < MAX_FRAME_BITS) {
                frameBitsA[frameBitCountA++] = 1u;
              }
            } else {
              ignoredSegments++;
            }
            armA = false;
          }

          // Model B pulse arm happens on LOW pulse.
          if (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US) {
            armB = true;
          } else {
            armB = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Total frames seen: ");
  Serial.println(totalFrames);
  Serial.print("Frames decoded by model A (>=16 bits): ");
  Serial.println(decodedFramesA);
  Serial.print("Frames decoded by model B (>=16 bits): ");
  Serial.println(decodedFramesB);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit low segments ignored: ");
  Serial.println(ignoredSegments);

  auto printTop = [&](const char* title, uint16_t* wordArr, uint16_t* countArr) {
    Serial.println(title);
    for (int rank = 0; rank < 8; ++rank) {
      uint16_t bestCount = 0;
      int bestIdx = -1;
      for (uint32_t i = 0; i < MAX_UNIQUE_WORDS; ++i) {
        if (countArr[i] > bestCount) {
          bestCount = countArr[i];
          bestIdx = (int)i;
        }
      }

      if (bestIdx < 0 || bestCount == 0) {
        break;
      }

      const uint16_t w = wordArr[bestIdx];
      const uint8_t n3 = (uint8_t)((w >> 12) & 0x0F);
      const uint8_t n2 = (uint8_t)((w >> 8) & 0x0F);
      const uint8_t n1 = (uint8_t)((w >> 4) & 0x0F);
      const uint8_t n0 = (uint8_t)(w & 0x0F);

      Serial.print("  0x");
      printHexByte((uint8_t)(w >> 8));
      printHexByte((uint8_t)(w & 0xFF));
      Serial.print("  count=");
      Serial.print(bestCount);
      Serial.print("  nibbles=");
      Serial.print(n3, HEX);
      Serial.print(' ');
      Serial.print(n2, HEX);
      Serial.print(' ');
      Serial.print(n1, HEX);
      Serial.print(' ');
      Serial.println(n0, HEX);

      countArr[bestIdx] = 0;
    }
  };

  printTop("Top MODEL A DIRECT candidates:", wordsA, countsA);
  printTop("Top MODEL A INVERTED candidates:", wordsAInv, countsAInv);
  printTop("Top MODEL B DIRECT candidates:", wordsB, countsB);
  printTop("Top MODEL B INVERTED candidates:", wordsBInv, countsBInv);

  Serial.println("Decode16 complete. Tach interrupt restored.");
}

void dumpBitFrames(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 6000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 64;

  Serial.println();
  Serial.print("Bitframes capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("A: HIGH pulse + LOW gap, B: LOW pulse + HIGH gap");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;
  bool armB = false;

  char bitsA[MAX_BITS + 1];
  char bitsB[MAX_BITS + 1];
  uint32_t lenA = 0;
  uint32_t lenB = 0;
  uint32_t frameNo = 0;
  uint32_t glitches = 0;

  auto flushFrame = [&]() {
    if (lenA == 0 && lenB == 0) {
      armA = false;
      armB = false;
      return;
    }

    if (frameNo < 10u) {
      frameNo++;
      bitsA[lenA] = '\0';
      bitsB[lenB] = '\0';
      Serial.print("F");
      Serial.print(frameNo);
      Serial.print(" A(");
      Serial.print(lenA);
      Serial.print("): ");
      Serial.println(bitsA);
      Serial.print("F");
      Serial.print(frameNo);
      Serial.print(" B(");
      Serial.print(lenB);
      Serial.print("): ");
      Serial.println(bitsB);
    }

    lenA = 0;
    lenB = 0;
    armA = false;
    armB = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitches++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          if (armB) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (lenB < MAX_BITS) {
                bitsB[lenB++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (lenB < MAX_BITS) {
                bitsB[lenB++] = '1';
              }
            }
            armB = false;
          }

          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (lenA < MAX_BITS) {
                bitsA[lenA++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (lenA < MAX_BITS) {
                bitsA[lenA++] = '1';
              }
            }
            armA = false;
          }

          armB = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames shown: ");
  Serial.println(frameNo);
  Serial.print("Glitches ignored: ");
  Serial.println(glitches);
  Serial.println("Bitframes capture complete. Tach interrupt restored.");
}

uint16_t bitsToWord16(const char* bits, uint32_t start, uint32_t len) {
  uint16_t w = 0;
  if (start + 16u > len) {
    return 0;
  }
  for (uint32_t i = 0; i < 16u; ++i) {
    w = (uint16_t)(w << 1);
    if (bits[start + i] == '1') {
      w = (uint16_t)(w | 1u);
    }
  }
  return w;
}

void dumpBit3(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 6000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 64;
  static const uint32_t WANT_FRAMES = 3;

  char saved[WANT_FRAMES][MAX_BITS + 1];
  uint32_t savedLen[WANT_FRAMES] = {0, 0, 0};
  uint32_t savedCount = 0;

  Serial.println();
  Serial.print("Bit3 capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Capturing first 3 model-A frames at current speed.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;
  char bitsA[MAX_BITS + 1];
  uint32_t lenA = 0;

  auto flushFrame = [&]() {
    if (lenA == 0) {
      armA = false;
      return;
    }

    if (savedCount < WANT_FRAMES) {
      bitsA[lenA] = '\0';
      memcpy(saved[savedCount], bitsA, lenA + 1);
      savedLen[savedCount] = lenA;
      savedCount++;
    }

    lenA = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u) && savedCount < WANT_FRAMES) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        // ignore
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (lenA < MAX_BITS) {
                bitsA[lenA++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (lenA < MAX_BITS) {
                bitsA[lenA++] = '1';
              }
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  if (savedCount == 0) {
    Serial.println("No frames captured.");
    Serial.println("Bit3 capture complete. Tach interrupt restored.");
    return;
  }

  for (uint32_t i = 0; i < savedCount; ++i) {
    const uint32_t len = savedLen[i];
    const char* bits = saved[i];
    uint32_t firstOne = len;
    for (uint32_t k = 0; k < len; ++k) {
      if (bits[k] == '1') {
        firstOne = k;
        break;
      }
    }

    const uint16_t tail16 = bitsToWord16(bits, len >= 16u ? (len - 16u) : 0u, len);
    const uint16_t first16 = firstOne + 16u <= len ? bitsToWord16(bits, firstOne, len) : 0u;

    Serial.print("F");
    Serial.print(i + 1);
    Serial.print(" bits(");
    Serial.print(len);
    Serial.print("): ");
    Serial.println(bits);

    Serial.print("F");
    Serial.print(i + 1);
    Serial.print(" first1=");
    if (firstOne < len) {
      Serial.print(firstOne);
    } else {
      Serial.print("none");
    }
    Serial.print(" tail16=0x");
    printHexByte((uint8_t)(tail16 >> 8));
    printHexByte((uint8_t)(tail16 & 0xFF));
    Serial.print(" first16@first1=0x");
    printHexByte((uint8_t)(first16 >> 8));
    printHexByte((uint8_t)(first16 & 0xFF));
    Serial.println();
  }

  if (savedCount >= 3u) {
    const uint16_t t1 = bitsToWord16(saved[0], savedLen[0] >= 16u ? (savedLen[0] - 16u) : 0u, savedLen[0]);
    const uint16_t t2 = bitsToWord16(saved[1], savedLen[1] >= 16u ? (savedLen[1] - 16u) : 0u, savedLen[1]);
    const uint16_t t3 = bitsToWord16(saved[2], savedLen[2] >= 16u ? (savedLen[2] - 16u) : 0u, savedLen[2]);
    Serial.print("Tail16 match across 3 frames: ");
    Serial.println((t1 == t2 && t2 == t3) ? "YES" : "NO");
  }

  Serial.println("Bit3 capture complete. Tach interrupt restored.");
}

void dumpBitStream(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 15000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_CHARS = 4000;

  Serial.println();
  Serial.print("Bitstream capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Model A stream: 0/1 bits, '|' marks long frame gap.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  uint32_t frameCount = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;
  uint32_t outCount = 0;

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        frameCount++;
        if (outCount < MAX_CHARS) {
          Serial.print('|');
          outCount++;
        }
        armA = false;
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (outCount < MAX_CHARS) {
                Serial.print('0');
                outCount++;
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (outCount < MAX_CHARS) {
                Serial.print('1');
                outCount++;
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  if (outCount >= MAX_CHARS) {
    Serial.println();
    Serial.println("(bitstream truncated)");
  } else {
    Serial.println();
  }

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames detected: ");
  Serial.println(frameCount);
  Serial.print("Bits/chars printed: ");
  Serial.println(outCount);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);
  Serial.println("Bitstream capture complete. Tach interrupt restored.");
}

void dumpTail16(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 15000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 96;
  static const uint32_t MAX_UNIQUE = 96;

  uint16_t tailWords[MAX_UNIQUE] = {0};
  uint16_t tailCounts[MAX_UNIQUE] = {0};
  uint16_t firstWords[MAX_UNIQUE] = {0};
  uint16_t firstCounts[MAX_UNIQUE] = {0};

  Serial.println();
  Serial.print("Tail16 capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Model A only. Per frame: first16@first1 and tail16 histograms.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t usableFrames = 0;
  uint32_t glitches = 0;
  uint32_t ignored = 0;

  auto flushFrame = [&]() {
    if (len == 0) {
      armA = false;
      return;
    }

    frameCount++;

    uint32_t firstOne = len;
    for (uint32_t i = 0; i < len; ++i) {
      if (bits[i] == '1') {
        firstOne = i;
        break;
      }
    }

    if (len >= 16u) {
      const uint16_t tail16 = bitsToWord16(bits, len - 16u, len);
      if (tail16 != 0x0000u && tail16 != 0xFFFFu) {
        addWordHit(tail16, tailWords, tailCounts, MAX_UNIQUE);
      }
      usableFrames++;
    }

    if (firstOne + 16u <= len) {
      const uint16_t first16 = bitsToWord16(bits, firstOne, len);
      if (first16 != 0x0000u && first16 != 0xFFFFu) {
        addWordHit(first16, firstWords, firstCounts, MAX_UNIQUE);
      }
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitches++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignored++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Frames with >=16 bits: ");
  Serial.println(usableFrames);
  Serial.print("Glitches ignored: ");
  Serial.println(glitches);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignored);

  auto printTopWords = [&](const char* title, uint16_t* wordArr, uint16_t* countArr) {
    Serial.println(title);
    for (int rank = 0; rank < 10; ++rank) {
      uint16_t bestCount = 0;
      int bestIdx = -1;
      for (uint32_t i = 0; i < MAX_UNIQUE; ++i) {
        if (countArr[i] > bestCount) {
          bestCount = countArr[i];
          bestIdx = (int)i;
        }
      }
      if (bestIdx < 0 || bestCount == 0) {
        break;
      }

      const uint16_t w = wordArr[bestIdx];
      Serial.print("  0x");
      printHexByte((uint8_t)(w >> 8));
      printHexByte((uint8_t)(w & 0xFF));
      Serial.print(" count=");
      Serial.println(bestCount);

      countArr[bestIdx] = 0;
    }
  };

  printTopWords("Top first16@first1 words:", firstWords, firstCounts);
  printTopWords("Top tail16 words:", tailWords, tailCounts);
  Serial.println("Tail16 capture complete. Tach interrupt restored.");
}

void dumpBitVote(uint32_t durationMs) {
  durationMs = constrain(durationMs, 300u, 15000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 96;
  static const uint32_t VOTE_BITS = 32;

  uint16_t tailOnes[VOTE_BITS] = {0};
  uint16_t tailTotal[VOTE_BITS] = {0};
  uint16_t firstOnes[VOTE_BITS] = {0};
  uint16_t firstTotal[VOTE_BITS] = {0};

  Serial.println();
  Serial.print("Bitvote capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Model A only. Voting on last 32 bits and first-1 aligned 32 bits.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t usableFrames = 0;
  uint32_t glitches = 0;
  uint32_t ignored = 0;

  auto flushFrame = [&]() {
    if (len == 0) {
      armA = false;
      return;
    }

    frameCount++;
    bits[len] = '\0';

    if (len >= VOTE_BITS) {
      usableFrames++;
      const uint32_t tailStart = len - VOTE_BITS;
      for (uint32_t i = 0; i < VOTE_BITS; ++i) {
        tailTotal[i]++;
        if (bits[tailStart + i] == '1') {
          tailOnes[i]++;
        }
      }
    }

    uint32_t firstOne = len;
    for (uint32_t i = 0; i < len; ++i) {
      if (bits[i] == '1') {
        firstOne = i;
        break;
      }
    }

    if (firstOne + VOTE_BITS <= len) {
      for (uint32_t i = 0; i < VOTE_BITS; ++i) {
        firstTotal[i]++;
        if (bits[firstOne + i] == '1') {
          firstOnes[i]++;
        }
      }
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitches++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignored++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Frames with >=32 tail bits: ");
  Serial.println(usableFrames);
  Serial.print("Glitches ignored: ");
  Serial.println(glitches);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignored);

  auto printVote = [&](const char* title, uint16_t* ones, uint16_t* total) {
    Serial.println(title);
    for (uint32_t i = 0; i < VOTE_BITS; ++i) {
      if (total[i] == 0) {
        Serial.print('?');
      } else {
        const uint16_t zeroes = total[i] - ones[i];
        if (ones[i] == total[i]) {
          Serial.print('1');
        } else if (zeroes == total[i]) {
          Serial.print('0');
        } else {
          Serial.print('x');
        }
      }
    }
    Serial.println();

    Serial.println("Confidence (% ones per bit):");
    for (uint32_t i = 0; i < VOTE_BITS; ++i) {
      if (i > 0) {
        Serial.print(' ');
      }
      if (total[i] == 0) {
        Serial.print("--");
      } else {
        const uint32_t pct = (uint32_t)((100u * ones[i]) / total[i]);
        if (pct < 10u) {
          Serial.print('0');
        }
        Serial.print(pct);
      }
    }
    Serial.println();
  };

  printVote("Tail-aligned 32-bit vote:", tailOnes, tailTotal);
  printVote("First-1 aligned 32-bit vote:", firstOnes, firstTotal);
  Serial.println("Bitvote capture complete. Tach interrupt restored.");
}

void printAlignedBytes(const char* bits, uint32_t start, uint32_t len, uint32_t byteCount) {
  for (uint32_t byteIdx = 0; byteIdx < byteCount; ++byteIdx) {
    const uint32_t bitStart = start + (byteIdx * 8u);
    if (bitStart + 8u > len) {
      break;
    }

    uint8_t value = 0;
    for (uint32_t bit = 0; bit < 8u; ++bit) {
      value = (uint8_t)(value << 1);
      if (bits[bitStart + bit] == '1') {
        value = (uint8_t)(value | 1u);
      }
    }

    if (byteIdx > 0) {
      Serial.print(' ');
    }
    printHexByte(value);
  }
}

void findPattern12(uint32_t durationMs) {
  durationMs = constrain(durationMs, 1000u, 180000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 160;
  static const char* TARGET = "0000000000001100";
  static const uint32_t TARGET_LEN = 16;
  static const uint32_t PRE_BITS = 24;
  static const uint32_t POST_BITS = 32;

  Serial.println();
  Serial.print("find12 on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for up to ");
  Serial.print(durationMs);
  Serial.println(" ms");
  Serial.print("Searching for bits: ");
  Serial.println(TARGET);

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;
  bool found = false;

  auto flushFrame = [&]() {
    if (len == 0 || found) {
      armA = false;
      return;
    }

    frameCount++;
    bits[len] = '\0';

    for (uint32_t pos = 0; pos + TARGET_LEN <= len; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < TARGET_LEN; ++i) {
        if (bits[pos + i] != TARGET[i]) {
          matches = false;
          break;
        }
      }

      if (!matches) {
        continue;
      }

      found = true;
      const uint32_t elapsedMs = (nowUs - startUs) / 1000u;

      Serial.print("FOUND at ");
      Serial.print(elapsedMs);
      Serial.print(" ms, frame=");
      Serial.print(frameCount);
      Serial.print(", bitPos=");
      Serial.println(pos);

      const uint32_t showStart = (pos > PRE_BITS) ? (pos - PRE_BITS) : 0u;
      const uint32_t showEnd = (pos + TARGET_LEN + POST_BITS <= len) ? (pos + TARGET_LEN + POST_BITS) : len;

      Serial.print("Context bits: ");
      for (uint32_t i = showStart; i < showEnd; ++i) {
        if (i == pos) {
          Serial.print('<');
        }
        Serial.print(bits[i]);
        if (i == pos + TARGET_LEN - 1u) {
          Serial.print('>');
        }
      }
      Serial.println();

      Serial.print("Aligned bytes at hit: ");
      printAlignedBytes(bits, pos, len, 8);
      Serial.println();
      break;
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u) && !found) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  if (!found) {
    Serial.println("Pattern not found in capture window.");
  }
  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);
  Serial.println("find12 complete. Tach interrupt restored.");
}

void dumpC0Span(uint32_t durationMs) {
  durationMs = constrain(durationMs, 1000u, 60000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 256;
  static const uint32_t MAX_PACKET_BYTES = 64;
  static const char* SYNC = "11000000";
  static const uint32_t SYNC_LEN = 8;

  Serial.println();
  Serial.print("c0span on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms");
  Serial.println("Model A only. Passive listen on D- (no local PWM toggles).");
  Serial.println("pkt  bytes  [type]  bytes...  delta");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t packetCount = 0;
  uint8_t packet[MAX_PACKET_BYTES] = {0};
  uint32_t packetLen = 0;
  uint8_t prevStatus[MAX_PACKET_BYTES] = {0};
  uint32_t prevStatusLen = 0;
  bool havePrevStatus = false;
  uint32_t framesWithSync = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;

  auto flushPacket = [&](bool partial) {
    if (packetLen < 2u) {
      return;
    }
    const bool isStatus = (packetLen >= 5u && packet[4] == 0xC0u);
    packetCount++;
    Serial.print("P");
    Serial.print(packetCount);
    Serial.print("  ");
    Serial.print(packetLen);
    Serial.print("  ");
    // Classify by suffix (last 2 bytes), or byte[4] for stat packets.
    const uint8_t lastByte   = packet[packetLen - 1u];
    const uint8_t penultByte = (packetLen >= 2u) ? packet[packetLen - 2u] : 0u;
    if (isStatus) {
      Serial.print("[stat]  ");
    } else if (penultByte == 0x80u && lastByte == 0x2Au) {
      Serial.print("[telB]  ");
    } else if (lastByte == 0x55u) {
      Serial.print("[t55?]  ");
    } else {
      Serial.print("[oth?]  ");
    }
    for (uint32_t i = 0; i < packetLen; ++i) {
      if (i > 0u) {
        Serial.print(' ');
      }
      printHexByte(packet[i]);
    }
    if (isStatus && packetLen >= 4u) {
      // Show raw key bytes; avoid assuming a speed mapping without correlation.
      Serial.print("   f2=");
      printHexByte(packet[2]);
      Serial.print(" f3=");
      printHexByte(packet[3]);

      // Show exact status byte/bit changes relative to the previous status packet.
      if (havePrevStatus && prevStatusLen == packetLen) {
        bool anyChange = false;
        for (uint32_t i = 0; i < packetLen; ++i) {
          if (packet[i] != prevStatus[i]) {
            anyChange = true;
            const uint8_t xorMask = (uint8_t)(packet[i] ^ prevStatus[i]);
            char bits[9];
            for (uint32_t b = 0; b < 8u; ++b) {
              bits[b] = ((xorMask >> (7u - b)) & 1u) ? '1' : '0';
            }
            bits[8] = '\0';
            Serial.print(" dB");
            Serial.print(i);
            Serial.print(":");
            printHexByte(prevStatus[i]);
            Serial.print("->");
            printHexByte(packet[i]);
            Serial.print(" xor=");
            printHexByte(xorMask);
            Serial.print("(");
            Serial.print(bits);
            Serial.print(")");
          }
        }
        if (!anyChange) {
          Serial.print(" d=none");
        }
      } else {
        Serial.print(" d=baseline");
      }

      for (uint32_t i = 0; i < packetLen; ++i) {
        prevStatus[i] = packet[i];
      }
      prevStatusLen = packetLen;
      havePrevStatus = true;
    }
    if (partial) {
      Serial.println(" (partial)");
    } else {
      Serial.println();
    }
  };

  // Variable-length packet: one gap-delimited frame = one packet.
  // C0 is the sync start; all bytes decoded to end of frame are payload.
  auto flushFrame = [&](bool partial) {
    if (len == 0) {
      armA = false;
      return;
    }

    frameCount++;
    bits[len] = '\0';

    // Find first C0 sync (11000000) with sliding bit search.
    int32_t syncPos = -1;
    for (uint32_t pos = 0; pos + SYNC_LEN <= len; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) {
          matches = false;
          break;
        }
      }
      if (matches) {
        syncPos = (int32_t)pos;
        break;
      }
    }

    if (syncPos >= 0) {
      framesWithSync++;
      packetLen = 0;
      const uint32_t alignedBytes = (len - (uint32_t)syncPos) / 8u;
      for (uint32_t byteIdx = 0; byteIdx < alignedBytes && packetLen < MAX_PACKET_BYTES; ++byteIdx) {
        const uint32_t bitStart = (uint32_t)syncPos + (byteIdx * 8u);
        uint8_t value = 0;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
          value = (uint8_t)(value << 1);
          if (bits[bitStart + bit] == '1') {
            value = (uint8_t)(value | 1u);
          }
        }
        packet[packetLen++] = value;
      }
      flushPacket(partial);
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame(false);
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame(true);

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Frames with C0 sync: ");
  Serial.println(framesWithSync);
  Serial.print("Packets printed: ");
  Serial.println(packetCount);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);
  Serial.println("c0span complete. Tach interrupt restored.");
}

void dumpC0Timing(uint32_t durationMs) {
  durationMs = constrain(durationMs, 1000u, 60000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 256;
  static const uint32_t MAX_PACKET_BYTES = 64;
  static const char* SYNC = "11000000";
  static const uint32_t SYNC_LEN = 8;

  Serial.println();
  Serial.print("c0timing on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms");
  Serial.println("Measure arrival timing for C0-synced packets.");
  Serial.println("pkt  t(ms)   dt(ms)  bytes  first-bytes");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint8_t packet[MAX_PACKET_BYTES] = {0};
  uint32_t packetLen = 0;
  uint32_t packetCount = 0;
  uint32_t framesSeen = 0;
  uint32_t framesWithSync = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;

  bool havePrevPacketTime = false;
  uint32_t prevPacketTimeUs = 0;
  uint32_t intervalCount = 0;
  uint64_t intervalSumUs = 0;
  uint32_t intervalMinUs = 0xFFFFFFFFu;
  uint32_t intervalMaxUs = 0u;

  auto printPacketLine = [&](uint32_t packetTimeUs) {
    packetCount++;

    Serial.print("P");
    Serial.print(packetCount);
    Serial.print("  ");

    const uint32_t tMs = (packetTimeUs - startUs) / 1000u;
    Serial.print(tMs);
    Serial.print("  ");

    if (havePrevPacketTime) {
      const uint32_t dtUs = packetTimeUs - prevPacketTimeUs;
      const uint32_t dtMs = dtUs / 1000u;
      Serial.print(dtMs);
      Serial.print("  ");

      intervalCount++;
      intervalSumUs += dtUs;
      if (dtUs < intervalMinUs) {
        intervalMinUs = dtUs;
      }
      if (dtUs > intervalMaxUs) {
        intervalMaxUs = dtUs;
      }
    } else {
      Serial.print("-");
      Serial.print("  ");
    }

    Serial.print(packetLen);
    Serial.print("  ");

    const uint32_t showBytes = (packetLen < 6u) ? packetLen : 6u;
    for (uint32_t i = 0; i < showBytes; ++i) {
      if (i > 0u) {
        Serial.print(' ');
      }
      printHexByte(packet[i]);
    }
    if (packetLen > showBytes) {
      Serial.print(" ...");
    }
    Serial.println();

    prevPacketTimeUs = packetTimeUs;
    havePrevPacketTime = true;
  };

  auto flushFrame = [&](uint32_t packetTimeUs) {
    if (len == 0) {
      armA = false;
      return;
    }

    framesSeen++;
    bits[len] = '\0';

    int32_t syncPos = -1;
    for (uint32_t pos = 0; pos + SYNC_LEN <= len; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) {
          matches = false;
          break;
        }
      }
      if (matches) {
        syncPos = (int32_t)pos;
        break;
      }
    }

    if (syncPos >= 0) {
      framesWithSync++;
      packetLen = 0;
      const uint32_t alignedBytes = (len - (uint32_t)syncPos) / 8u;
      for (uint32_t byteIdx = 0; byteIdx < alignedBytes && packetLen < MAX_PACKET_BYTES; ++byteIdx) {
        const uint32_t bitStart = (uint32_t)syncPos + (byteIdx * 8u);
        uint8_t value = 0;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
          value = (uint8_t)(value << 1);
          if (bits[bitStart + bit] == '1') {
            value = (uint8_t)(value | 1u);
          }
        }
        packet[packetLen++] = value;
      }

      if (packetLen > 0u && packet[0] == 0xC0u) {
        printPacketLine(packetTimeUs);
      }
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        // The packet ends at the edge before the long idle segment.
        flushFrame(lastEdgeUs);
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  // Flush any trailing frame fragment at capture end.
  flushFrame(nowUs);

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(framesSeen);
  Serial.print("Frames with C0 sync: ");
  Serial.println(framesWithSync);
  Serial.print("C0 packets: ");
  Serial.println(packetCount);

  if (intervalCount > 0u) {
    const uint32_t avgUs = (uint32_t)(intervalSumUs / (uint64_t)intervalCount);
    Serial.print("Interval min/avg/max (ms): ");
    Serial.print(intervalMinUs / 1000u);
    Serial.print(" / ");
    Serial.print(avgUs / 1000u);
    Serial.print(" / ");
    Serial.println(intervalMaxUs / 1000u);
    Serial.print("Peak-to-peak jitter (ms): ");
    Serial.println((intervalMaxUs - intervalMinUs) / 1000u);
  } else {
    Serial.println("Interval stats: need at least 2 packets.");
  }

  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);
  Serial.println("c0timing complete. Tach interrupt restored.");
}

void dumpSyncFind(uint32_t durationMs) {
  durationMs = constrain(durationMs, 500u, 30000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 128;
  static const uint32_t WANT_MATCHES = 10;
  static const char* SYNC = "11000000";
  static const uint32_t SYNC_LEN = 8;
  static const uint32_t PRINT_BITS = 40;
  static const uint32_t PRINT_BYTES = 5;

  Serial.println();
  Serial.print("Syncfind on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Model A only. Searching for sync pattern 11000000 and printing following bits/bytes.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t matchCount = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;

  auto flushFrame = [&]() {
    if (len == 0) {
      armA = false;
      return;
    }

    frameCount++;
    bits[len] = '\0';

    for (uint32_t pos = 0; pos + SYNC_LEN <= len && matchCount < WANT_MATCHES; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) {
          matches = false;
          break;
        }
      }

      if (!matches) {
        continue;
      }

      matchCount++;
      Serial.print("M");
      Serial.print(matchCount);
      Serial.print(" frame=");
      Serial.print(frameCount);
      Serial.print(" pos=");
      Serial.print(pos);
      Serial.print(" bits=");

      const uint32_t end = (pos + PRINT_BITS <= len) ? (pos + PRINT_BITS) : len;
      for (uint32_t i = pos; i < end; ++i) {
        Serial.print(bits[i]);
      }
      Serial.println();

      Serial.print("M");
      Serial.print(matchCount);
      Serial.print(" bytes=");
      printAlignedBytes(bits, pos, len, PRINT_BYTES);
      Serial.println();
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u) && matchCount < WANT_MATCHES) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Matches found: ");
  Serial.println(matchCount);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);
  Serial.println("Syncfind complete. Tach interrupt restored.");
}

void addFrameHit(const uint8_t* frame, uint8_t frames[][6], uint16_t* counts, uint32_t maxItems) {
  for (uint32_t i = 0; i < maxItems; ++i) {
    if (counts[i] == 0) {
      for (uint32_t b = 0; b < 6u; ++b) {
        frames[i][b] = frame[b];
      }
      counts[i] = 1;
      return;
    }

    bool same = true;
    for (uint32_t b = 0; b < 6u; ++b) {
      if (frames[i][b] != frame[b]) {
        same = false;
        break;
      }
    }
    if (same) {
      if (counts[i] < 65535u) {
        counts[i]++;
      }
      return;
    }
  }
}

// ---------------------------------------------------------------------------
// dumpFrameBytes — stream ALL C0-sync-anchored 6-byte frames as they arrive.
// Each line is tagged with bytes[1..2] so the two frame families are distinct:
//   [4445] = config frame  (C0 44 45 49 C0 15)
//   [888A] = telemetry frame (C0 88 8A 93 80 2A)
// Run this while adjusting speed/mode to watch which frame bytes change.
// ---------------------------------------------------------------------------
void dumpFrameBytes(uint32_t durationMs) {
  durationMs = constrain(durationMs, 1000u, 60000u);

  static const uint32_t GLITCH_US   = 200;
  static const uint32_t GAP_US      = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US  = 850;
  static const uint32_t LONG_MAX_US  = 1300;
  static const uint32_t MAX_BITS    = 128;
  static const char*    SYNC        = "11000000";  // 0xC0
  static const uint32_t SYNC_LEN    = 8;
  static const uint32_t FRAME_BYTES = 6;
  static const uint32_t FRAME_BITS  = FRAME_BYTES * 8u;

  Serial.println();
  Serial.print("framebytes on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms  [all C0-anchored 6-byte frames]");
  Serial.println("t(s)   [type]  B1  B2  B3  B4  B5  B6");
  Serial.println("-------------------------------------------");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t matchCount = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;

  auto flushFrame = [&]() {
    if (len == 0) { armA = false; return; }
    bits[len] = '\0';

    for (uint32_t pos = 0; pos + FRAME_BITS <= len; ++pos) {
      // Require C0 sync at this position
      bool syncOk = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) { syncOk = false; break; }
      }
      if (!syncOk) continue;

      // Decode 6 bytes
      uint8_t frame[6] = {0};
      for (uint32_t byteIdx = 0; byteIdx < FRAME_BYTES; ++byteIdx) {
        uint8_t value = 0;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
          value = (uint8_t)(value << 1);
          if (bits[pos + (byteIdx * 8u) + bit] == '1') value |= 1u;
        }
        frame[byteIdx] = value;
      }

      matchCount++;
      uint32_t elapsedMs = (nowUs - startUs) / 1000u;

      // Timestamp  "  0.12s"
      uint32_t sec = elapsedMs / 1000u;
      uint32_t frac = (elapsedMs % 1000u) / 10u;
      if (sec < 10) Serial.print(" ");
      Serial.print(sec);
      Serial.print(".");
      if (frac < 10) Serial.print("0");
      Serial.print(frac);
      Serial.print("s  [");

      // Type tag = bytes[1..2] in compact hex
      printHexByte(frame[1]);
      printHexByte(frame[2]);
      Serial.print("]  ");

      // All 6 bytes
      for (uint32_t b = 0; b < 6u; ++b) {
        if (b > 0) Serial.print(' ');
        printHexByte(frame[b]);
      }
      Serial.println();
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) bits[len++] = '0';
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) bits[len++] = '1';
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.println("-------------------------------------------");
  Serial.print("Total frames printed: ");
  Serial.println(matchCount);
  Serial.print("Glitches: ");
  Serial.print(glitchCount);
  Serial.print("  Non-bit gaps: ");
  Serial.println(ignoredCount);
  Serial.println("framebytes complete. Tach interrupt restored.");
}

void dumpFrameStable(uint32_t durationMs) {
  durationMs = constrain(durationMs, 1000u, 120000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 128;
  static const uint32_t SYNC_LEN = 8;
  static const char* SYNC = "11000000";
  static const uint32_t FRAME_BYTES = 6;
  static const uint32_t FRAME_BITS = FRAME_BYTES * 8u;
  static const uint32_t MAX_UNIQUE = 24;
  static const uint32_t WINDOW_MS = 1000;

  uint8_t windowFrames[MAX_UNIQUE][6] = {{0}};
  uint16_t windowCounts[MAX_UNIQUE] = {0};

  Serial.println();
  Serial.print("framestable on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms (1s windows, dominant C0 frame only)");
  Serial.println("window(s)    dominant frame                count/total  confidence");
  Serial.println("-----------------------------------------------------------------");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;
  uint32_t totalFrames = 0;

  uint32_t windowStartMs = 0;
  uint32_t windowFramesTotal = 0;

  auto clearWindow = [&]() {
    for (uint32_t i = 0; i < MAX_UNIQUE; ++i) {
      windowCounts[i] = 0;
    }
    windowFramesTotal = 0;
  };

  auto printWindowSummary = [&](uint32_t fromMs, uint32_t toMs) {
    uint16_t bestCount = 0;
    int bestIdx = -1;
    for (uint32_t i = 0; i < MAX_UNIQUE; ++i) {
      if (windowCounts[i] > bestCount) {
        bestCount = windowCounts[i];
        bestIdx = (int)i;
      }
    }

    Serial.print("  ");
    Serial.print(fromMs / 1000u);
    Serial.print(".");
    uint32_t fFrac = (fromMs % 1000u) / 10u;
    if (fFrac < 10u) Serial.print("0");
    Serial.print(fFrac);
    Serial.print("-");
    Serial.print(toMs / 1000u);
    Serial.print(".");
    uint32_t tFrac = (toMs % 1000u) / 10u;
    if (tFrac < 10u) Serial.print("0");
    Serial.print(tFrac);
    Serial.print("s    ");

    if (bestIdx < 0 || bestCount == 0 || windowFramesTotal == 0) {
      Serial.println("(no valid frames)");
      return;
    }

    for (uint32_t b = 0; b < 6u; ++b) {
      if (b > 0) Serial.print(' ');
      printHexByte(windowFrames[bestIdx][b]);
    }

    Serial.print("      ");
    Serial.print(bestCount);
    Serial.print("/");
    Serial.print(windowFramesTotal);
    Serial.print("      ");
    uint32_t conf = (uint32_t)((bestCount * 100u) / windowFramesTotal);
    Serial.print(conf);
    Serial.println("%");
  };

  auto flushFrame = [&]() {
    if (len == 0) {
      armA = false;
      return;
    }

    bits[len] = '\0';

    for (uint32_t pos = 0; pos + FRAME_BITS <= len; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) {
          matches = false;
          break;
        }
      }
      if (!matches) {
        continue;
      }

      uint8_t frame[6] = {0};
      for (uint32_t byteIdx = 0; byteIdx < FRAME_BYTES; ++byteIdx) {
        uint8_t value = 0;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
          value = (uint8_t)(value << 1);
          if (bits[pos + (byteIdx * 8u) + bit] == '1') {
            value = (uint8_t)(value | 1u);
          }
        }
        frame[byteIdx] = value;
      }

      addFrameHit(frame, windowFrames, windowCounts, MAX_UNIQUE);
      totalFrames++;
      windowFramesTotal++;
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }

    const uint32_t elapsedMs = (nowUs - startUs) / 1000u;
    if ((elapsedMs - windowStartMs) >= WINDOW_MS) {
      printWindowSummary(windowStartMs, elapsedMs);
      clearWindow();
      windowStartMs = elapsedMs;
    }
  }

  flushFrame();

  const uint32_t finalElapsedMs = (nowUs - startUs) / 1000u;
  if (windowFramesTotal > 0) {
    printWindowSummary(windowStartMs, finalElapsedMs);
  }

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.println("-----------------------------------------------------------------");
  Serial.print("Total decoded C0 frames: ");
  Serial.println(totalFrames);
  Serial.print("Glitches: ");
  Serial.print(glitchCount);
  Serial.print("  Non-bit gaps: ");
  Serial.println(ignoredCount);
  Serial.println("framestable complete. Tach interrupt restored.");
}

void dumpBestFrame(uint32_t durationMs) {
  durationMs = constrain(durationMs, 500u, 30000u);

  static const uint32_t GLITCH_US = 200;
  static const uint32_t GAP_US = 2000;
  static const uint32_t PULSE_MIN_US = 350;
  static const uint32_t PULSE_MAX_US = 700;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_BITS = 128;
  static const uint32_t MAX_UNIQUE = 32;
  static const char* SYNC = "11000000";
  static const uint32_t SYNC_LEN = 8;
  static const uint32_t FRAME_BYTES = 6;
  static const uint32_t FRAME_BITS = FRAME_BYTES * 8u;

  uint8_t seenFrames[MAX_UNIQUE][6] = {{0}};
  uint16_t seenCounts[MAX_UNIQUE] = {0};

  Serial.println();
  Serial.print("Bestframe on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Model A only. Looking for sync C0 and extracting 6-byte frames.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);
  bool armA = false;

  char bits[MAX_BITS + 1];
  uint32_t len = 0;
  uint32_t frameCount = 0;
  uint32_t matchCount = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredCount = 0;

  auto flushFrame = [&]() {
    if (len == 0) {
      armA = false;
      return;
    }

    frameCount++;
    bits[len] = '\0';

    for (uint32_t pos = 0; pos + FRAME_BITS <= len; ++pos) {
      bool matches = true;
      for (uint32_t i = 0; i < SYNC_LEN; ++i) {
        if (bits[pos + i] != SYNC[i]) {
          matches = false;
          break;
        }
      }
      if (!matches) {
        continue;
      }

      uint8_t frame[6] = {0};
      for (uint32_t byteIdx = 0; byteIdx < FRAME_BYTES; ++byteIdx) {
        uint8_t value = 0;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
          value = (uint8_t)(value << 1);
          if (bits[pos + (byteIdx * 8u) + bit] == '1') {
            value = (uint8_t)(value | 1u);
          }
        }
        frame[byteIdx] = value;
      }

      addFrameHit(frame, seenFrames, seenCounts, MAX_UNIQUE);
      matchCount++;
    }

    len = 0;
    armA = false;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (segUs > GAP_US) {
        flushFrame();
      } else {
        if (lastState == HIGH) {
          armA = (segUs >= PULSE_MIN_US && segUs <= PULSE_MAX_US);
        } else {
          if (armA) {
            if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '0';
              }
            } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
              if (len < MAX_BITS) {
                bits[len++] = '1';
              }
            } else {
              ignoredCount++;
            }
            armA = false;
          }
        }
      }

      lastState = state;
      lastEdgeUs = nowUs;
    }
  }

  flushFrame();

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  Serial.print("Frames seen: ");
  Serial.println(frameCount);
  Serial.print("Sync matches extracted: ");
  Serial.println(matchCount);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit gaps ignored: ");
  Serial.println(ignoredCount);

  Serial.println("Top 6-byte frames:");
  for (int rank = 0; rank < 8; ++rank) {
    uint16_t bestCount = 0;
    int bestIdx = -1;
    for (uint32_t i = 0; i < MAX_UNIQUE; ++i) {
      if (seenCounts[i] > bestCount) {
        bestCount = seenCounts[i];
        bestIdx = (int)i;
      }
    }

    if (bestIdx < 0 || bestCount == 0) {
      break;
    }

    Serial.print("  count=");
    Serial.print(bestCount);
    Serial.print(" bytes=");
    for (uint32_t b = 0; b < 6u; ++b) {
      if (b > 0) {
        Serial.print(' ');
      }
      printHexByte(seenFrames[bestIdx][b]);
    }
    Serial.println();

    seenCounts[bestIdx] = 0;
  }

  Serial.println("Bestframe complete. Tach interrupt restored.");
}

void runStepTest() {
  const int testSpeeds[] = {10, 50, 100};
  const size_t stepCount = sizeof(testSpeeds) / sizeof(testSpeeds[0]);
  const uint32_t settleMs = 4000;
  const uint32_t sampleMs = 6000;

  Serial.println();
  Serial.println("=== Tach Step Test (avg RPM) ===");
  Serial.print("Settle per step (ms): ");
  Serial.println(settleMs);
  Serial.print("Sample per step (ms): ");
  Serial.println(sampleMs);

  for (size_t i = 0; i < stepCount; ++i) {
    const int speed = testSpeeds[i];
    applyFanSpeed(speed);
    Serial.print("Step ");
    Serial.print(i + 1);
    Serial.print(" -> speed ");
    Serial.print(speed);
    Serial.println("% (settling)...");
    delay(settleMs);

    uint32_t startPulses;
    noInterrupts();
    startPulses = tachPulseCount;
    interrupts();

    delay(sampleMs);

    uint32_t endPulses;
    noInterrupts();
    endPulses = tachPulseCount;
    interrupts();

    const uint32_t delta = endPulses - startPulses;
    const float revs = delta / TACH_PULSES_PER_REV;
    const float minutes = sampleMs / 60000.0f;
    const float avgRpm = minutes > 0.0f ? (revs / minutes) : 0.0f;

    Serial.print("Result speed ");
    Serial.print(speed);
    Serial.print("%: pulses=");
    Serial.print(delta);
    Serial.print(", avgRPM=");
    Serial.println(avgRpm, 1);
  }

  Serial.println("=== End Step Test ===");
}

void printHexByte(uint8_t b) {
  const char* hex = "0123456789ABCDEF";
  Serial.print(hex[(b >> 4) & 0x0F]);
  Serial.print(hex[b & 0x0F]);
}

void scanUartOnTach(uint32_t totalMs) {
  totalMs = constrain(totalMs, 1000u, 30000u);
  const uint32_t invertModes = UART_SCAN_INCLUDE_INVERTED ? 2u : 1u;
  const uint32_t modeCount = (sizeof(UART_SCAN_BAUDS) / sizeof(UART_SCAN_BAUDS[0])) * invertModes;
  const uint32_t perModeMs = totalMs / modeCount;

  Serial.println();
  Serial.print("UART scan on D- pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(totalMs);
  Serial.println(" ms");
  Serial.println(UART_SCAN_INCLUDE_INVERTED
                   ? "Trying common baud rates with normal and inverted polarity."
                   : "Trying common baud rates with normal polarity only.");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  HardwareSerial& probe = Serial1;
  for (size_t i = 0; i < sizeof(UART_SCAN_BAUDS) / sizeof(UART_SCAN_BAUDS[0]); ++i) {
    const uint32_t baud = UART_SCAN_BAUDS[i];
    for (uint32_t invertIdx = 0; invertIdx < invertModes; ++invertIdx) {
      const bool invert = invertIdx == 1;

      probe.end();
      delay(2);
      probe.begin(baud, SERIAL_8N1, FAN_TACH_PIN, -1, invert);
      delay(10);

      while (probe.available()) {
        (void)probe.read();
      }

      const uint32_t startMs = millis();
      uint32_t byteCount = 0;
      uint32_t lineCount = 0;

      Serial.print("Mode baud=");
      Serial.print(baud);
      Serial.print(" invert=");
      Serial.println(invert ? "true" : "false");

      while ((millis() - startMs) < perModeMs) {
        while (probe.available()) {
          const int v = probe.read();
          if (v < 0) {
            break;
          }

          if ((byteCount % 16u) == 0u) {
            Serial.print("  ");
          }

          printHexByte((uint8_t)v);
          Serial.print(' ');
          byteCount++;

          if ((byteCount % 16u) == 0u) {
            Serial.println();
            lineCount++;
            if (lineCount >= 6u) {
              break;
            }
          }
        }

        if (lineCount >= 6u) {
          break;
        }
      }

      if ((byteCount % 16u) != 0u) {
        Serial.println();
      }

      Serial.print("  bytes=");
      Serial.println(byteCount);
      Serial.println();
    }
  }

  probe.end();
  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);
  Serial.println("UART scan complete. Tach interrupt restored.");
}

void drawStaticUi() {
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(8, 8);
  tft.print("AC INFINITY");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 40);
  tft.print("Speed:");

  tft.setCursor(8, 66);
  tft.print("RPM:");

  tft.setCursor(8, 96);
  tft.print("TACH:");
}

void updateUi(bool force) {
  const int rpmInt = (int)currentRpm;
  uint32_t tachCountSnapshot;
  noInterrupts();
  tachCountSnapshot = tachPulseCount;
  interrupts();

  if (!force && rpmInt == lastUiRpm && targetSpeedPercent == lastUiSpeed) {
    if (tachCountSnapshot == lastUiTachCount) {
      return;
    }
  }

  tft.fillRect(92, 40, 140, 18, ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(92, 40);
  tft.print(targetSpeedPercent);
  tft.print("%");

  tft.fillRect(70, 66, 162, 18, ST77XX_BLACK);
  tft.setTextColor(ST77XX_MAGENTA);
  tft.setTextSize(2);
  tft.setCursor(70, 66);
  tft.print(rpmInt);

  tft.fillRect(70, 96, 162, 20, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(70, 96);
  tft.print("P:");
  tft.print(lastDeltaPulses);
  tft.print(" RPM:");
  tft.print((int)lastRawRpm);

  lastUiSpeed = targetSpeedPercent;
  lastUiRpm = rpmInt;
  lastUiTachCount = tachCountSnapshot;
}

int nextButtonSpeedUp(int currentSpeed) {
  currentSpeed = constrain(currentSpeed, 0, 100);
  if (currentSpeed < 10) {
    return 10;
  }
  if (currentSpeed < 99) {
    return currentSpeed + 1;
  }
  return 100;
}

int nextButtonSpeedDown(int currentSpeed) {
  currentSpeed = constrain(currentSpeed, 0, 100);
  if (currentSpeed > 99) {
    return 99;
  }
  if (currentSpeed > 10) {
    return currentSpeed - 1;
  }
  return 0;
}

void handleHardwareButtons() {
  const uint32_t now = millis();
  const int minusRaw = digitalRead(BTN_MINUS_PIN);
  const int presetRaw = digitalRead(BTN_PRESET_PIN);
  const int plusRaw = digitalRead(BTN_PLUS_PIN);

  const bool minusPressedNow = BTN_MINUS_ACTIVE_LOW ? (minusRaw == LOW) : (minusRaw == HIGH);
  const bool presetPressedNow = BTN_PRESET_ACTIVE_LOW ? (presetRaw == LOW) : (presetRaw == HIGH);
  const bool plusPressedNow = BTN_PLUS_ACTIVE_LOW ? (plusRaw == LOW) : (plusRaw == HIGH);

  if (now - lastButtonEdgeMs < BUTTON_DEBOUNCE_MS) {
    minusPressedPrev = minusPressedNow;
    presetPressedPrev = presetPressedNow;
    plusPressedPrev = plusPressedNow;
    return;
  }

  if (minusPressedNow && !minusPressedPrev) {
    applyFanSpeed(nextButtonSpeedDown(targetSpeedPercent));
    Serial.print("Button D0 -> Speed ");
    Serial.print(targetSpeedPercent);
    Serial.println("%");
    lastButtonEdgeMs = now;
  }

  if (presetPressedNow && !presetPressedPrev) {
    applyFanSpeed(50);
    Serial.println("Button D1 -> Speed 50%");
    lastButtonEdgeMs = now;
  }

  if (plusPressedNow && !plusPressedPrev) {
    applyFanSpeed(nextButtonSpeedUp(targetSpeedPercent));
    Serial.print("Button D2 -> Speed ");
    Serial.print(targetSpeedPercent);
    Serial.println("%");
    lastButtonEdgeMs = now;
  }

  minusPressedPrev = minusPressedNow;
  presetPressedPrev = presetPressedNow;
  plusPressedPrev = plusPressedNow;
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  speed <0-100>   Set fan speed percentage");
  Serial.println("  pwmduty <0-1023> Set raw PWM duty directly");
  Serial.println("  pwmauto         Return to mapped speed mode");
  Serial.println("  rawtach [ms]    Raw tach capture (200..10000 ms)");
  Serial.println("  edgesnap [ms]   Print high/low segment timings on D- (200..5000 ms)");
  Serial.println("  edgehist [ms]   Compact timing histogram on D- (200..5000 ms)");
  Serial.println("  pulsecap [ms]   Tokenize D- into S/L/G pulses (200..5000 ms)");
  Serial.println("  framesnap [ms]  Show pulse frames split by long gaps (200..5000 ms)");
  Serial.println("  decode16 [ms]   Extract candidate 16-bit payload words (300..6000 ms)");
  Serial.println("  bitframes [ms]  Show raw decoded bits per frame (300..6000 ms)");
  Serial.println("  bit3 [ms]       Capture and compare 3 frames at one speed (300..6000 ms)");
  Serial.println("  bitstream [ms]  Long model-A bit stream with frame separators");
  Serial.println("  tail16 [ms]     Histogram 16-bit frame candidates (model A)");
  Serial.println("  bitvote [ms]    Bit-by-bit consensus across model-A frames");
  Serial.println("  find12 [ms]     Stop when 0000000000001100 is seen (prints context)");
  Serial.println("  c0span [ms]     Show all C0 packets split by type [stat][telB][t55?][oth?]");
  Serial.println("  c0timing [ms]   Measure per-packet C0 arrival interval and jitter");
  Serial.println("  syncfind [ms]   Find 11000000 and print aligned bits/bytes");
  Serial.println("  bestframe [ms]  Rank repeated 6-byte frames starting at C0");
  Serial.println("  framebytes [ms] Stream ALL C0-anchored frames tagged by type bytes");
  Serial.println("  framestable [ms] Dominant C0 frame per 1s window + confidence");
  Serial.println("  uartscan [ms]   Probe D- as UART data (1000..30000 ms)");
  Serial.println("  steptest        Auto-test 10/50/100% with averaged RPM");
  Serial.println("  + / inc         Increase speed by 10%");
  Serial.println("  - / dec         Decrease speed by 10%");
  Serial.println("  D0 button       Step down: 100,99..10,0");
  Serial.println("  D1 button       Set speed to 50%");
  Serial.println("  D2 button       Step up: 0,10..99,100");
  Serial.println("  status          Show speed and RPM");
  Serial.println("  help            Show this menu");
  Serial.println();
}

void handleSerialCommand() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();

  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("status")) {
    printStatus();
    return;
  }

  if (line.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  if (line.equalsIgnoreCase("pwmauto")) {
    applyFanSpeed(targetSpeedPercent);
    Serial.println("PWM auto mapping restored.");
    return;
  }

  if (line.startsWith("pwmduty ") || line.startsWith("PWMDUTY ")) {
    String valueText = line.substring(8);
    valueText.trim();
    long value = valueText.toInt();
    if (value < 0 || value > ((1 << PWM_RES_BITS) - 1)) {
      Serial.print("Invalid duty. Use 0..");
      Serial.println((1 << PWM_RES_BITS) - 1);
      return;
    }
    applyRawDuty((uint32_t)value);
    return;
  }

  if (line.equalsIgnoreCase("rawtach")) {
    dumpRawTach(2000);
    return;
  }

  if (line.equalsIgnoreCase("edgesnap")) {
    dumpEdgeTimings(1000);
    return;
  }

  if (line.equalsIgnoreCase("edgehist")) {
    dumpEdgeHistogram(1000);
    return;
  }

  if (line.equalsIgnoreCase("pulsecap")) {
    dumpPulseProtocol(1200);
    return;
  }

  if (line.equalsIgnoreCase("framesnap")) {
    dumpFrameSummary(1200);
    return;
  }

  if (line.equalsIgnoreCase("decode16")) {
    dumpDecode16(1500);
    return;
  }

  if (line.equalsIgnoreCase("bitframes")) {
    dumpBitFrames(1500);
    return;
  }

  if (line.equalsIgnoreCase("bit3")) {
    dumpBit3(2000);
    return;
  }

  if (line.equalsIgnoreCase("bitstream")) {
    dumpBitStream(10000);
    return;
  }

  if (line.equalsIgnoreCase("tail16")) {
    dumpTail16(10000);
    return;
  }

  if (line.equalsIgnoreCase("bitvote")) {
    dumpBitVote(10000);
    return;
  }

  if (line.equalsIgnoreCase("find12")) {
    findPattern12(30000);
    return;
  }

  if (line.equalsIgnoreCase("c0span")) {
    dumpC0Span(15000);
    return;
  }

  if (line.equalsIgnoreCase("c0timing")) {
    dumpC0Timing(15000);
    return;
  }

  if (line.equalsIgnoreCase("syncfind")) {
    dumpSyncFind(10000);
    return;
  }

  if (line.equalsIgnoreCase("bestframe")) {
    dumpBestFrame(10000);
    return;
  }

  if (line.equalsIgnoreCase("framebytes")) {
    dumpFrameBytes(15000);
    return;
  }

  if (line.equalsIgnoreCase("framestable")) {
    dumpFrameStable(15000);
    return;
  }

  if (line.startsWith("framebytes ") || line.startsWith("FRAMEBYTES ")) {
    String valueText = line.substring(11);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid framebytes duration. Use milliseconds, e.g. framebytes 20000");
      return;
    }
    dumpFrameBytes(durationMs);
    return;
  }

  if (line.startsWith("framestable ") || line.startsWith("FRAMESTABLE ")) {
    String valueText = line.substring(12);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid framestable duration. Use milliseconds, e.g. framestable 20000");
      return;
    }
    dumpFrameStable(durationMs);
    return;
  }

  if (line.equalsIgnoreCase("steptest")) {
    runStepTest();
    return;
  }

  if (line.equalsIgnoreCase("uartscan")) {
    scanUartOnTach(12000);
    return;
  }

  if (line.startsWith("rawtach ") || line.startsWith("RAWTACH ")) {
    String valueText = line.substring(8);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid rawtach duration. Use milliseconds, e.g. rawtach 3000");
      return;
    }
    dumpRawTach(durationMs);
    return;
  }

  if (line.startsWith("edgesnap ") || line.startsWith("EDGESNAP ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid edgesnap duration. Use milliseconds, e.g. edgesnap 1000");
      return;
    }
    dumpEdgeTimings(durationMs);
    return;
  }

  if (line.startsWith("edgehist ") || line.startsWith("EDGEHIST ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid edgehist duration. Use milliseconds, e.g. edgehist 1000");
      return;
    }
    dumpEdgeHistogram(durationMs);
    return;
  }

  if (line.startsWith("pulsecap ") || line.startsWith("PULSECAP ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid pulsecap duration. Use milliseconds, e.g. pulsecap 1200");
      return;
    }
    dumpPulseProtocol(durationMs);
    return;
  }

  if (line.startsWith("framesnap ") || line.startsWith("FRAMESNAP ")) {
    String valueText = line.substring(10);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid framesnap duration. Use milliseconds, e.g. framesnap 1200");
      return;
    }
    dumpFrameSummary(durationMs);
    return;
  }

  if (line.startsWith("decode16 ") || line.startsWith("DECODE16 ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid decode16 duration. Use milliseconds, e.g. decode16 1500");
      return;
    }
    dumpDecode16(durationMs);
    return;
  }

  if (line.startsWith("bitframes ") || line.startsWith("BITFRAMES ")) {
    String valueText = line.substring(10);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid bitframes duration. Use milliseconds, e.g. bitframes 1500");
      return;
    }
    dumpBitFrames(durationMs);
    return;
  }

  if (line.startsWith("bit3 ") || line.startsWith("BIT3 ")) {
    String valueText = line.substring(5);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid bit3 duration. Use milliseconds, e.g. bit3 2000");
      return;
    }
    dumpBit3(durationMs);
    return;
  }

  if (line.startsWith("bitstream ") || line.startsWith("BITSTREAM ")) {
    String valueText = line.substring(10);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid bitstream duration. Use milliseconds, e.g. bitstream 10000");
      return;
    }
    dumpBitStream(durationMs);
    return;
  }

  if (line.startsWith("tail16 ") || line.startsWith("TAIL16 ")) {
    String valueText = line.substring(7);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid tail16 duration. Use milliseconds, e.g. tail16 10000");
      return;
    }
    dumpTail16(durationMs);
    return;
  }

  if (line.startsWith("bitvote ") || line.startsWith("BITVOTE ")) {
    String valueText = line.substring(8);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid bitvote duration. Use milliseconds, e.g. bitvote 10000");
      return;
    }
    dumpBitVote(durationMs);
    return;
  }

  if (line.startsWith("find12 ") || line.startsWith("FIND12 ")) {
    String valueText = line.substring(7);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid find12 duration. Use milliseconds, e.g. find12 60000");
      return;
    }
    findPattern12(durationMs);
    return;
  }

  if (line.startsWith("c0span ") || line.startsWith("C0SPAN ")) {
    String valueText = line.substring(7);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid c0span duration. Use milliseconds, e.g. c0span 15000");
      return;
    }
    dumpC0Span(durationMs);
    return;
  }

  if (line.startsWith("c0timing ") || line.startsWith("C0TIMING ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid c0timing duration. Use milliseconds, e.g. c0timing 15000");
      return;
    }
    dumpC0Timing(durationMs);
    return;
  }

  if (line.startsWith("syncfind ") || line.startsWith("SYNCFIND ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid syncfind duration. Use milliseconds, e.g. syncfind 10000");
      return;
    }
    dumpSyncFind(durationMs);
    return;
  }

  if (line.startsWith("bestframe ") || line.startsWith("BESTFRAME ")) {
    String valueText = line.substring(10);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid bestframe duration. Use milliseconds, e.g. bestframe 10000");
      return;
    }
    dumpBestFrame(durationMs);
    return;
  }

  if (line.startsWith("uartscan ") || line.startsWith("UARTSCAN ")) {
    String valueText = line.substring(9);
    valueText.trim();
    uint32_t durationMs = (uint32_t)valueText.toInt();
    if (durationMs == 0) {
      Serial.println("Invalid uartscan duration. Use milliseconds, e.g. uartscan 12000");
      return;
    }
    scanUartOnTach(durationMs);
    return;
  }

  if (line == "+" || line.equalsIgnoreCase("inc")) {
    applyFanSpeed(targetSpeedPercent + 10);
    Serial.print("Speed set to ");
    Serial.print(targetSpeedPercent);
    Serial.println("%.");
    return;
  }

  if (line == "-" || line.equalsIgnoreCase("dec")) {
    applyFanSpeed(targetSpeedPercent - 10);
    Serial.print("Speed set to ");
    Serial.print(targetSpeedPercent);
    Serial.println("%.");
    return;
  }

  if (line.startsWith("speed ") || line.startsWith("SPEED ")) {
    String valueText = line.substring(6);
    valueText.trim();
    int value = valueText.toInt();

    if (value < 0 || value > 100) {
      Serial.println("Invalid speed. Use 0 to 100.");
      return;
    }

    applyFanSpeed(value);
    Serial.print("Speed set to ");
    Serial.print(value);
    Serial.println("%.");
    return;
  }

  Serial.println("Unknown command. Type 'help'.");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("AC Infinity controller booting...");
  Serial.print("PWM pin: ");
  Serial.println(FAN_PWM_PIN);
  Serial.print("Tach pin: ");
  Serial.println(FAN_TACH_PIN);
  Serial.print("PWM freq (Hz): ");
  Serial.println(PWM_FREQ_HZ);
  Serial.print("PWM resolution (bits): ");
  Serial.println(PWM_RES_BITS);
  Serial.print("PWM active-low-at-fan-line: ");
  Serial.println(PWM_ACTIVE_LOW_AT_FAN_LINE ? "true" : "false");

  pinMode(FAN_TACH_PIN, USE_INTERNAL_TACH_PULLUP ? INPUT_PULLUP : INPUT);
  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), onTachPulse, RISING);

  pinMode(BTN_MINUS_PIN, BTN_MINUS_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(BTN_PRESET_PIN, BTN_PRESET_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(BTN_PLUS_PIN, BTN_PLUS_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);

  if (!setupFanPwm()) {
    Serial.println("ERROR: PWM setup failed.");
  }

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);
  tft.init(135, 240);
  tft.setRotation(3);
  drawStaticUi();

  applyFanSpeed(targetSpeedPercent);
  lastRpmSampleMs = millis();
  lastPulseSeenMs = lastRpmSampleMs;
  updateUi(true);

  printHelp();
  printStatus();
}

void loop() {
  handleSerialCommand();
  handleHardwareButtons();
  updateRpm();

  const uint32_t now = millis();
  if (now - lastUiDrawMs > 250) {
    lastUiDrawMs = now;
    updateUi(false);
  }

  // Periodic serial spam disabled; use 'status' when needed.
}
