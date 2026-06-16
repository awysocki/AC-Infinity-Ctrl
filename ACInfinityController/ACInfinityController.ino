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
  return (uint32_t)((effectivePercent * (int)maxDuty) / 100);
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

  static const uint32_t GLITCH_US = 80;
  static const uint32_t GAP_US = 2000;
  static const uint32_t SHORT_MIN_US = 350;
  static const uint32_t SHORT_MAX_US = 700;
  static const uint32_t LONG_MIN_US = 850;
  static const uint32_t LONG_MAX_US = 1300;
  static const uint32_t MAX_FRAME_BITS = 64;
  static const uint32_t MAX_UNIQUE_WORDS = 96;

  uint16_t wordsDirect[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsDirect[MAX_UNIQUE_WORDS] = {0};
  uint16_t wordsInverted[MAX_UNIQUE_WORDS] = {0};
  uint16_t countsInverted[MAX_UNIQUE_WORDS] = {0};
  uint32_t totalFrames = 0;
  uint32_t decodedFrames = 0;
  uint32_t glitchCount = 0;
  uint32_t ignoredSegments = 0;

  Serial.println();
  Serial.print("Decode16 capture on pin ");
  Serial.print(FAN_TACH_PIN);
  Serial.print(" for ");
  Serial.print(durationMs);
  Serial.println(" ms...");
  Serial.println("Decoding low segments: ~500us=0, ~1000us=1");

  detachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN));
  delay(5);

  const uint32_t startUs = micros();
  uint32_t nowUs = startUs;
  uint32_t lastEdgeUs = startUs;
  int lastState = digitalRead(FAN_TACH_PIN);

  uint8_t frameBits[MAX_FRAME_BITS] = {0};
  uint32_t frameBitCount = 0;

  auto flushFrame = [&]() {
    if (frameBitCount < 16u) {
      frameBitCount = 0;
      return;
    }

    totalFrames++;
    decodedFrames++;

    // Use first 16 bits in each frame; also record inverted mapping because
    // line polarity might be opposite of our assumption.
    uint16_t wDirect = 0;
    uint16_t wInverted = 0;
    for (uint32_t b = 0; b < 16u; ++b) {
      const uint8_t bit = frameBits[b] ? 1u : 0u;
      wDirect = (uint16_t)((wDirect << 1) | bit);
      wInverted = (uint16_t)((wInverted << 1) | (bit ? 0u : 1u));
    }

    addWordHit(wDirect, wordsDirect, countsDirect, MAX_UNIQUE_WORDS);
    addWordHit(wInverted, wordsInverted, countsInverted, MAX_UNIQUE_WORDS);

    frameBitCount = 0;
  };

  while ((uint32_t)(nowUs - startUs) < (durationMs * 1000u)) {
    const int state = digitalRead(FAN_TACH_PIN);
    nowUs = micros();
    if (state != lastState) {
      const uint32_t segUs = nowUs - lastEdgeUs;

      if (segUs < GLITCH_US) {
        glitchCount++;
      } else if (lastState == LOW && segUs > GAP_US) {
        // Long low gap separates repeated frames.
        flushFrame();
      } else if (lastState == LOW) {
        // Decode only LOW segment widths as bits.
        if (segUs >= SHORT_MIN_US && segUs <= SHORT_MAX_US) {
          if (frameBitCount < MAX_FRAME_BITS) {
            frameBits[frameBitCount++] = 0u;
          }
        } else if (segUs >= LONG_MIN_US && segUs <= LONG_MAX_US) {
          if (frameBitCount < MAX_FRAME_BITS) {
            frameBits[frameBitCount++] = 1u;
          }
        } else {
          ignoredSegments++;
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
  Serial.print("Frames decoded (>=16 bits): ");
  Serial.println(decodedFrames);
  Serial.print("Glitches ignored: ");
  Serial.println(glitchCount);
  Serial.print("Non-bit low segments ignored: ");
  Serial.println(ignoredSegments);

  Serial.println("Top DIRECT candidates (word, count, nibbles):");

  // Print up to 12 strongest hits.
  for (int rank = 0; rank < 12; ++rank) {
    uint16_t bestCount = 0;
    int bestIdx = -1;
    for (uint32_t i = 0; i < MAX_UNIQUE_WORDS; ++i) {
      if (countsDirect[i] > bestCount) {
        bestCount = countsDirect[i];
        bestIdx = (int)i;
      }
    }

    if (bestIdx < 0 || bestCount == 0) {
      break;
    }

    const uint16_t w = wordsDirect[bestIdx];
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

    countsDirect[bestIdx] = 0;
  }

  Serial.println("Top INVERTED candidates (word, count, nibbles):");
  for (int rank = 0; rank < 12; ++rank) {
    uint16_t bestCount = 0;
    int bestIdx = -1;
    for (uint32_t i = 0; i < MAX_UNIQUE_WORDS; ++i) {
      if (countsInverted[i] > bestCount) {
        bestCount = countsInverted[i];
        bestIdx = (int)i;
      }
    }

    if (bestIdx < 0 || bestCount == 0) {
      break;
    }

    const uint16_t w = wordsInverted[bestIdx];
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

    countsInverted[bestIdx] = 0;
  }

  Serial.println("Decode16 complete. Tach interrupt restored.");
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
    applyFanSpeed(targetSpeedPercent - 10);
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
    applyFanSpeed(targetSpeedPercent + 10);
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
  Serial.println("  uartscan [ms]   Probe D- as UART data (1000..30000 ms)");
  Serial.println("  steptest        Auto-test 10/50/100% with averaged RPM");
  Serial.println("  + / inc         Increase speed by 10%");
  Serial.println("  - / dec         Decrease speed by 10%");
  Serial.println("  D0 button       Decrease speed by 10%");
  Serial.println("  D1 button       Set speed to 50%");
  Serial.println("  D2 button       Increase speed by 10%");
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
