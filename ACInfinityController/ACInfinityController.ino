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
static const int FAN_PWM_PIN = 12;      // Drives BS170 gate
static const int FAN_TACH_PIN = 13;     // Reads tach pulses (FG)

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

// AC Infinity FG line appears to output 3 pulses per revolution.
static const float TACH_PULSES_PER_REV = 3.0f;
// 5000 us min edge interval ~= 6000 RPM max for 2 pulses/rev.
// This aggressively rejects high-frequency noise seen on the tach line.
static const uint32_t TACH_MIN_EDGE_US = 5000;
static const float MAX_REASONABLE_RPM = 4500.0;
static const uint32_t RPM_SAMPLE_MS = 1000;
static const uint32_t RPM_NO_PULSE_TIMEOUT_MS = 3000;

volatile uint32_t tachPulseCount = 0;
volatile uint32_t lastTachEdgeUs = 0;
uint32_t lastRpmSampleMs = 0;
uint32_t lastPulseSnapshot = 0;
uint32_t lastPulseSeenMs = 0;
float currentRpm = 0.0f;
int targetSpeedPercent = 40;

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
  if ((uint32_t)(nowUs - lastTachEdgeUs) < TACH_MIN_EDGE_US) {
    return;
  }
  lastTachEdgeUs = nowUs;
  tachPulseCount++;
}

uint32_t percentToDuty(int percent) {
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

      // Map user request MIN_RUN..100 to calibrated effective range.
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

  effectivePercent = constrain(effectivePercent, 0, 100);

  const uint32_t maxDuty = (1u << PWM_RES_BITS) - 1u;
  return (uint32_t)((effectivePercent * (int)maxDuty) / 100);
}

void applyFanSpeed(int percent) {
  targetSpeedPercent = constrain(percent, 0, 100);
  writeFanDuty(percentToDuty(targetSpeedPercent));
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
  lastPulseSnapshot = pulseSnapshot;
  lastRpmSampleMs = now;

  const float revs = deltaPulses / TACH_PULSES_PER_REV;
  const float minutes = intervalMs / 60000.0f;
  if (deltaPulses == 0) {
    if ((now - lastPulseSeenMs) >= RPM_NO_PULSE_TIMEOUT_MS) {
      currentRpm = 0.0f;
    }
    return;
  }

  lastPulseSeenMs = now;

  if (minutes > 0.0f) {
    float rpm = revs / minutes;
    if (rpm > MAX_REASONABLE_RPM) {
      rpm = MAX_REASONABLE_RPM;
    }
    // Smooth tach updates so brief pulse jitter does not flicker the reading.
    currentRpm = (currentRpm * 0.6f) + (rpm * 0.4f);
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
  Serial.println("------------------------------");
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

  tft.fillRect(8, 124, 220, 10, ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(8, 124);
  tft.print("TACH RAW: ");
  tft.print(tachCountSnapshot);

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

  // External bias network shown in your diagram (1k to 3.3V, 10k to GND)
  // provides line conditioning, so use plain INPUT.
  pinMode(FAN_TACH_PIN, INPUT);
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

  static uint32_t lastPrint = 0;
  if (now - lastPrint > 3000) {
    lastPrint = now;
    uint32_t pulseSnapshot;
    noInterrupts();
    pulseSnapshot = tachPulseCount;
    interrupts();
    Serial.print("RPM: ");
    Serial.print((int)currentRpm);
    Serial.print(" | TachCount: ");
    Serial.print(pulseSnapshot);
    Serial.print(" | Speed: ");
    Serial.print(targetSpeedPercent);
    Serial.println("%");
  }
}
