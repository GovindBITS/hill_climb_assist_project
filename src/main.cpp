/*
 File: src/main.cpp
 Brief: Hill-climb assist firmware for STM32 BlackPill interfacing IMU, OLED, TB6612FNG motor driver, brake servos, and ESP32 bridge.
 Project: Hill Climb Assist Platform
 Author: Govind S Warrier
 Student ID: 2025NS01140
 Institution: BITS Pilani
 Date: 2025-03-11
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include <Servo.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>

#if defined(LED_BUILTIN)
constexpr uint8_t kStatusLedPin = LED_BUILTIN;
#else
constexpr uint8_t kStatusLedPin = 13;
#endif

constexpr uint8_t kMotorPWMA = PB13;
constexpr uint8_t kMotorAIN2 = PB14;
constexpr uint8_t kMotorAIN1 = PB15;
constexpr uint8_t kMotorStandby = PA8;
constexpr uint8_t kMotorBIN1 = PA9;
constexpr uint8_t kMotorBIN2 = PA10;
constexpr uint8_t kMotorPWMB = PB0;

constexpr uint8_t kBrakeServoLeftPin = PA5;
constexpr uint8_t kBrakeServoRightPin = PA6;

#if !defined(PB7) || !defined(PB8) || !defined(PB3) || !defined(PB10)
#error "Required STM32 pin macros (PB7, PB8, PB3, PB10) are not defined for this board."
#endif

TwoWire imuWire(PB7, PB8);      // I2C1 for MPU9250 (SDA, SCL)
TwoWire displayWire(PB3, PB10); // I2C2 for OLED (SDA, SCL)

constexpr uint8_t kMpuPrimaryAddr = 0x68;
constexpr uint8_t kMpuSecondaryAddr = 0x69;

constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegPwrMgmt2 = 0x6C;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelConfig2 = 0x1D;
constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegAccelXoutH = 0x3B;

constexpr float kAccelScale = 16384.0f; // LSB per g @ +/-2g
constexpr float kGyroScale = 131.0f;    // LSB per deg/s @ +/-250dps
constexpr float kTempScale = 333.87f;
constexpr float kTempOffset = 21.0f;

Adafruit_SSD1306 display(128, 32, &displayWire, -1);

struct ImuSample {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  float tempC = 0.0f;
  uint32_t timestampMs = 0;
  bool valid = false;
};

ImuSample gImuSample{};
bool gImuReady = false;
bool gDisplayReady = false;
uint8_t gMpuAddress = kMpuPrimaryAddr;

constexpr uint32_t kImuReadIntervalMs = 20;
constexpr uint32_t kDisplayRefreshIntervalMs = 200;
constexpr uint32_t kSerialLogIntervalMs = 500;
constexpr uint32_t kLedBlinkIntervalMs = 400;

uint32_t gLastImuReadMs = 0;
uint32_t gLastDisplayRefreshMs = 0;
uint32_t gLastSerialLogMs = 0;
uint32_t gLastLedToggleMs = 0;
bool gLedState = false;

constexpr uint8_t kAssistButtonPin = PC14;
constexpr uint32_t kAssistDebounceMs = 30;
constexpr uint32_t kAssistMessageDisplayMs = 2000;
constexpr float kAssistPitchThresholdDeg = 8.0f;
constexpr float kAssistGainPerDeg = 6.0f;
constexpr int16_t kAssistMaxBoost = 120;
constexpr int16_t kAssistRampStep = 5;

constexpr uint8_t kEspRxPin = PA3;
constexpr uint8_t kEspTxPin = PA2;
constexpr uint32_t kEspBaudRate = 115200;
constexpr uint32_t kEspHeartbeatTimeoutMs = 3000;
constexpr size_t kEspLineBufferSize = 160;

HardwareSerial espSerial(kEspRxPin, kEspTxPin);

enum class Orientation : uint8_t {
  Unknown = 0,
  X_POS,
  X_NEG,
  Y_POS,
  Y_NEG,
  Z_POS,
  Z_NEG,
};

struct AxisMapping {
  uint8_t axisIndex;
  int8_t sign;
  const char* label;
};

struct OrientationProfile {
  Orientation id;
  uint8_t verticalAxis;
  int8_t verticalSign;
  AxisMapping axisX; // horizontal axis mapped to display X (left/right)
  AxisMapping axisY; // horizontal axis mapped to display Y (up/down)
  const char* orientationLabel;
};

struct LevelState {
  float axisXDeg = 0.0f;
  float axisYDeg = 0.0f;
  float bubbleX = 0.0f; // normalized -1..1
  float bubbleY = 0.0f; // normalized -1..1
  Orientation orientation = Orientation::Unknown;
  const OrientationProfile* profile = nullptr;
  bool valid = false;
};

LevelState gLevelState{};

constexpr float kAccelFilterAlpha = 0.2f;
constexpr float kMaxTiltDeg = 20.0f;
constexpr float kLevelThresholdDeg = 1.0f;
constexpr float kOrientationAdoptThreshold = 0.78f;
constexpr float kOrientationHoldThreshold = 0.60f;

float gFilteredAx = 0.0f;
float gFilteredAy = 0.0f;
float gFilteredAz = 1.0f;
bool gAccelFilterInitialised = false;

struct LineBuffer {
  char data[kEspLineBufferSize];
  size_t length = 0;

  void clear() { length = 0; }

  void append(char c) {
    if (length + 1 >= sizeof(data)) {
      data[sizeof(data) - 2] = '\0';
      length = sizeof(data) - 1;
      return;
    }
    data[length++] = c;
  }

  bool empty() const { return length == 0; }

  const char* c_str() {
    if (length >= sizeof(data)) {
      length = sizeof(data) - 1;
    }
    data[length] = '\0';
    return data;
  }
};

constexpr OrientationProfile kOrientationProfiles[] = {
    {Orientation::Z_POS,
     2,
     +1,
     {1, -1, "-Y"},
     {0, -1, "-X"},
     "Surface +Z"},
    {Orientation::Z_NEG,
     2,
     -1,
     {1, +1, "+Y"},
     {0, +1, "+X"},
     "Surface -Z"},
    {Orientation::X_POS,
     0,
     +1,
     {1, -1, "-Y"},
     {2, +1, "+Z"},
     "Surface +X"},
    {Orientation::X_NEG,
     0,
     -1,
     {1, +1, "+Y"},
     {2, -1, "-Z"},
     "Surface -X"},
    {Orientation::Y_POS,
     1,
     +1,
     {0, +1, "+X"},
     {2, +1, "+Z"},
     "Surface +Y"},
    {Orientation::Y_NEG,
     1,
     -1,
     {0, -1, "-X"},
     {2, -1, "-Z"},
     "Surface -Y"},
};

Orientation gCurrentOrientation = Orientation::Unknown;
LineBuffer gEspRxLine{};
bool gEspBridgeActive = false;
bool gEsp32Ready = false;
uint32_t gLastEspHeartbeatMs = 0;
bool gButtonState = false;
bool gAssistEnabled = false;
uint32_t gLastButtonChangeMs = 0;
uint32_t gAssistMessageUntilMs = 0;
int16_t gCommandLeft = 0;
int16_t gCommandRight = 0;
int16_t gMotorTargetLeft = 0;
int16_t gMotorTargetRight = 0;
int16_t gMotorAppliedLeft = 0;
int16_t gMotorAppliedRight = 0;
int16_t gAssistBoostLeft = 0;
int16_t gAssistBoostRight = 0;
Servo gBrakeServoLeft;
Servo gBrakeServoRight;
int gBrakeLeftRelease = 90;   // degree position when left brake is released
int gBrakeLeftApplied = 120;  // degree position when left brake is applied
int gBrakeRightRelease = 120;  // degree position when right brake is released
int gBrakeRightApplied = 90; // degree position when right brake is applied
bool gBrakesEngaged = false;

const OrientationProfile* findProfile(Orientation id);
Orientation makeOrientation(uint8_t axisIndex, int8_t sign);
float axisValue(const float components[3], const AxisMapping& mapping);

bool probeMpu(uint8_t address, uint8_t& whoAmI);
bool mpuWriteByte(uint8_t reg, uint8_t value);
bool mpuReadBytes(uint8_t reg, uint8_t* buffer, size_t length);
bool initMpu9250();
bool readMpu9250(ImuSample& sample);
void updateLevelState(const ImuSample& sample);
void updateDisplay();
void logImuToSerial(const ImuSample& sample, uint32_t nowMs);
Orientation detectOrientation(const float components[3], Orientation previous);
void setupEspBridge();
void setupMotorDriver();
void setupBrakeServos();
void handleEspInput();
void processEspLine(const char* line);
void sendEspStatus(uint32_t nowMs);
void sendEspTelemetry(const ImuSample& sample, const LevelState& state, uint32_t nowMs);
void updateButton(uint32_t nowMs);
void setMotorTargets(int16_t left, int16_t right);
void serviceMotorOutputs();
void recomputeMotorTargets();
void applyBrakes();
void releaseBrakes();
void applyMotorOutputs(int16_t channelA, int16_t channelB);
void setMotorStandby(bool enable);

void setupMotorDriver() {
  pinMode(kMotorStandby, OUTPUT);
  pinMode(kMotorAIN1, OUTPUT);
  pinMode(kMotorAIN2, OUTPUT);
  pinMode(kMotorBIN1, OUTPUT);
  pinMode(kMotorBIN2, OUTPUT);
  pinMode(kMotorPWMA, OUTPUT);
  pinMode(kMotorPWMB, OUTPUT);

  digitalWrite(kMotorAIN1, LOW);
  digitalWrite(kMotorAIN2, LOW);
  digitalWrite(kMotorBIN1, LOW);
  digitalWrite(kMotorBIN2, LOW);
  analogWrite(kMotorPWMA, 0);
  analogWrite(kMotorPWMB, 0);
  setMotorStandby(false);

  gMotorTargetLeft = 0;
  gMotorTargetRight = 0;
  gMotorAppliedLeft = 0;
  gMotorAppliedRight = 0;
}

void setupBrakeServos() {
  gBrakeServoLeft.attach(kBrakeServoLeftPin);
  gBrakeServoRight.attach(kBrakeServoRightPin);
  releaseBrakes();
}

void applyBrakes() {
  if (!gBrakeServoLeft.attached() || !gBrakeServoRight.attached()) {
    return;
  }
  if (!gBrakesEngaged) {
    gBrakeServoLeft.write(constrain(gBrakeLeftApplied, 0, 180));
    gBrakeServoRight.write(constrain(gBrakeRightApplied, 0, 180));
    gBrakesEngaged = true;
  }
}

void releaseBrakes() {
  if (!gBrakeServoLeft.attached() || !gBrakeServoRight.attached()) {
    return;
  }
  gBrakeServoLeft.write(constrain(gBrakeLeftRelease, 0, 180));
  gBrakeServoRight.write(constrain(gBrakeRightRelease, 0, 180));
  gBrakesEngaged = false;
}

const OrientationProfile* findProfile(Orientation id) {
  for (const auto& profile : kOrientationProfiles) {
    if (profile.id == id) {
      return &profile;
    }
  }
  return nullptr;
}

Orientation makeOrientation(uint8_t axisIndex, int8_t sign) {
  switch (axisIndex) {
    case 0:
      return sign >= 0 ? Orientation::X_POS : Orientation::X_NEG;
    case 1:
      return sign >= 0 ? Orientation::Y_POS : Orientation::Y_NEG;
    case 2:
      return sign >= 0 ? Orientation::Z_POS : Orientation::Z_NEG;
    default:
      return Orientation::Unknown;
  }
}

float axisValue(const float components[3], const AxisMapping& mapping) {
  const uint8_t idx = mapping.axisIndex;
  if (idx > 2) {
    return 0.0f;
  }
  return components[idx] * static_cast<float>(mapping.sign);
}

Orientation detectOrientation(const float components[3], Orientation previous) {
  const float absComponents[3] = {fabsf(components[0]), fabsf(components[1]), fabsf(components[2])};

  if (previous != Orientation::Unknown) {
    const OrientationProfile* prevProfile = findProfile(previous);
    if (prevProfile) {
      const float currentMagnitude = absComponents[prevProfile->verticalAxis];
      if (currentMagnitude >= kOrientationHoldThreshold) {
        const int8_t expectedSign =
            components[prevProfile->verticalAxis] >= 0.0f ? 1 : -1;
        if (expectedSign == prevProfile->verticalSign) {
          return previous;
        }
      }
    }
  }

  int bestAxis = 0;
  float bestMagnitude = absComponents[0];
  for (int i = 1; i < 3; ++i) {
    if (absComponents[i] > bestMagnitude) {
      bestMagnitude = absComponents[i];
      bestAxis = i;
    }
  }

  if (bestMagnitude < kOrientationAdoptThreshold) {
    return Orientation::Unknown;
  }

  const int8_t sign = components[bestAxis] >= 0.0f ? 1 : -1;
  return makeOrientation(bestAxis, sign);
}

void setup() {
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);

  pinMode(kAssistButtonPin, INPUT_PULLUP);
  gButtonState = digitalRead(kAssistButtonPin) == LOW;
  gAssistEnabled = gButtonState;
  gLastButtonChangeMs = millis();
  gAssistMessageUntilMs = 0;

  Serial.begin(115200);
  const uint32_t startWait = millis();
  while (!Serial && (millis() - startWait) < 2000) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("=== Hillclimbing dual-I2C bring-up ==="));
  Serial.print(F("Build: "));
  Serial.print(F(__DATE__));
  Serial.print(F(" "));
  Serial.println(F(__TIME__));

  imuWire.begin();
  imuWire.setClock(400000);

  displayWire.begin();
  displayWire.setClock(400000);

  gImuReady = initMpu9250();
  if (!gImuReady) {
    Serial.println(F("[imu] MPU9250 not detected. Check wiring on PB7/PB8."));
  }

  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Hillclimbing IMU"));
    display.println(F("OLED online"));
    if (gImuReady) {
      display.println(F("MPU9250 ok"));
    } else {
      display.println(F("MPU9250 missing"));
    }
    display.display();
    gDisplayReady = true;
  } else {
    Serial.println(F("[oled] SSD1306 init failed. Check wiring on PB3/PB10."));
    gDisplayReady = false;
  }

  setupBrakeServos();
  setupMotorDriver();
  recomputeMotorTargets();
  if (gAssistEnabled && gCommandLeft == 0 && gCommandRight == 0) {
    applyBrakes();
  } else {
    releaseBrakes();
  }
  setupEspBridge();
}

void setupEspBridge() {
  gEspBridgeActive = false;
  gEsp32Ready = false;
  gEspRxLine.clear();
  gLastEspHeartbeatMs = 0;

  espSerial.begin(kEspBaudRate);
  gEspBridgeActive = true;

  Serial.print(F("[esp-link] UART online (RX="));
  Serial.print(kEspRxPin);
  Serial.print(F(", TX="));
  Serial.print(kEspTxPin);
  Serial.println(F(", 115200 bps)"));
  espSerial.println(F("STM_READY"));
}

void loop() {
  const uint32_t nowMs = millis();

  handleEspInput();
  updateButton(nowMs);
  serviceMotorOutputs();

  if (nowMs - gLastLedToggleMs >= kLedBlinkIntervalMs) {
    gLastLedToggleMs = nowMs;
    gLedState = !gLedState;
    digitalWrite(kStatusLedPin, gLedState ? HIGH : LOW);
  }

  if (gImuReady && (nowMs - gLastImuReadMs) >= kImuReadIntervalMs) {
    gLastImuReadMs = nowMs;
    ImuSample sample;
    if (readMpu9250(sample)) {
      sample.timestampMs = nowMs;
      sample.valid = true;
      gImuSample = sample;
      updateLevelState(gImuSample);
    } else {
      gImuSample.valid = false;
      gLevelState.valid = false;
      Serial.println(F("[imu] Read failed."));
    }
  }

  if (gDisplayReady && (nowMs - gLastDisplayRefreshMs) >= kDisplayRefreshIntervalMs) {
    gLastDisplayRefreshMs = nowMs;
    updateDisplay();
  }

  if (gImuSample.valid && (nowMs - gLastSerialLogMs) >= kSerialLogIntervalMs) {
    gLastSerialLogMs = nowMs;
    logImuToSerial(gImuSample, nowMs);
  }
}

void handleEspInput() {
  if (!gEspBridgeActive) {
    return;
  }

  while (espSerial.available() > 0) {
    const char c = static_cast<char>(espSerial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (!gEspRxLine.empty()) {
        const char* line = gEspRxLine.c_str();
        processEspLine(line);
        gEspRxLine.clear();
      }
      continue;
    }

    gEspRxLine.append(c);
  }

  if (gEsp32Ready && (millis() - gLastEspHeartbeatMs > kEspHeartbeatTimeoutMs)) {
    gEsp32Ready = false;
    Serial.println(F("[esp-link] Lost ESP32 heartbeat."));
  }
}

void processEspLine(const char* line) {
  if (line == nullptr || line[0] == '\0') {
    return;
  }

  auto skipWhitespace = [](const char* ptr) {
    while (*ptr == ' ' || *ptr == '\t') {
      ++ptr;
    }
    return ptr;
  };

  auto parseMagnitude = [&](const char* ptr, int defaultValue) -> int16_t {
    const char* start = skipWhitespace(ptr);
    if (*start == '\0') {
      return static_cast<int16_t>(defaultValue);
    }
    char* end = nullptr;
    const long value = strtol(start, &end, 10);
    if (end == start) {
      return static_cast<int16_t>(defaultValue);
    }
    const long clamped = max(0l, min(255l, labs(value)));
    return static_cast<int16_t>(clamped);
  };

  if (strncmp(line, "ESP32_READY", 11) == 0) {
    gEsp32Ready = true;
    gLastEspHeartbeatMs = millis();
    Serial.println(F("[esp-link] ESP32 ready."));
    espSerial.println(F("STM_READY"));
    return;
  }

  if (strncmp(line, "ESP32_PING", 10) == 0) {
    Serial.println(F("[esp-link] Ping from ESP32."));
    espSerial.println(F("STM_PONG"));
    return;
  }

  if (strncmp(line, "ESP32_RESET", 11) == 0) {
    Serial.println(F("[esp-link] Reset request from ESP32."));
    espSerial.println(F("STM_ACK RESET"));
    return;
  }

  if (strncmp(line, "ESP32_HEARTBEAT", 15) == 0) {
    const char* value = line + 15;
    while (*value == ' ') {
      ++value;
    }
    (void)strtoul(value, nullptr, 10);
    gLastEspHeartbeatMs = millis();
    if (!gEsp32Ready) {
      gEsp32Ready = true;
      Serial.println(F("[esp-link] Heartbeat detected; link established."));
      espSerial.println(F("STM_READY"));
    }
    espSerial.print(F("STM_HEARTBEAT "));
    espSerial.println(gLastEspHeartbeatMs);
    return;
  }

  if (strncmp(line, "FWD", 3) == 0) {
    const int16_t speed = parseMagnitude(line + 3, 180);
    setMotorTargets(speed, speed);
    return;
  }

  if (strncmp(line, "BACK", 4) == 0) {
    const int16_t speed = parseMagnitude(line + 4, 180);
    setMotorTargets(-speed, -speed);
    return;
  }

  if (strncmp(line, "LEFT", 4) == 0) {
    const int16_t speed = parseMagnitude(line + 4, 160);
    setMotorTargets(-speed, speed);
    return;
  }

  if (strncmp(line, "RIGHT", 5) == 0) {
    const int16_t speed = parseMagnitude(line + 5, 160);
    setMotorTargets(speed, -speed);
    return;
  }

  if (strncmp(line, "STOP", 4) == 0 || strncmp(line, "BRAKE", 5) == 0) {
    setMotorTargets(0, 0);
    return;
  }

  if (strncasecmp(line, "RELEASE", 7) == 0 || strncasecmp(line, "BRAKE OFF", 9) == 0) {
    releaseBrakes();
    return;
  }

  if (strncmp(line, "MOTORS", 6) == 0 || strncmp(line, "DRV", 3) == 0) {
    const char* ptr = skipWhitespace(line + (line[0] == 'M' ? 6 : 3));
    char* end = nullptr;
    const long left = strtol(ptr, &end, 10);
    if (end == ptr) {
      Serial.println(F("[motor] Invalid left speed"));
      return;
    }
    ptr = skipWhitespace(end);
    const long right = strtol(ptr, &end, 10);
    if (end == ptr) {
      Serial.println(F("[motor] Invalid right speed"));
      return;
    }
    setMotorTargets(static_cast<int16_t>(left), static_cast<int16_t>(right));
    return;
  }

  Serial.print(F("[esp-link] RX: "));
  Serial.println(line);
}

void updateButton(uint32_t nowMs) {
  const bool pressed = digitalRead(kAssistButtonPin) == LOW;
  if (pressed == gButtonState) {
    return;
  }

  if (nowMs - gLastButtonChangeMs < kAssistDebounceMs) {
    return;
  }

  gButtonState = pressed;
  gLastButtonChangeMs = nowMs;

  gAssistEnabled = pressed;
  gAssistMessageUntilMs = nowMs + kAssistMessageDisplayMs;
  Serial.print(F("[button] Assist "));
  Serial.println(gAssistEnabled ? F("ENABLED") : F("DISABLED"));
  if (gEspBridgeActive) {
    espSerial.print(F("STM_ASSIST state="));
    espSerial.println(gAssistEnabled ? F("ON") : F("OFF"));
  }
  // Recalculate motor targets to apply new assist state immediately.
  recomputeMotorTargets();
  if (!gAssistEnabled) {
    releaseBrakes();
  } else if (gCommandLeft == 0 && gCommandRight == 0) {
    applyBrakes();
  } else {
    releaseBrakes();
  }
}

void setMotorStandby(bool enable) {
  digitalWrite(kMotorStandby, enable ? HIGH : LOW);
}

void applyMotorOutputs(int16_t channelA, int16_t channelB) {
  auto applyChannel = [](uint8_t pwmPin, uint8_t in1, uint8_t in2, int16_t value) {
    const int16_t limited = constrain(value, -255, 255);
    const uint16_t duty = static_cast<uint16_t>(abs(limited));

    if (limited > 0) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    } else if (limited < 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }

    analogWrite(pwmPin, duty);
  };

  applyChannel(kMotorPWMA, kMotorAIN1, kMotorAIN2, channelA);
  applyChannel(kMotorPWMB, kMotorBIN1, kMotorBIN2, channelB);
}

void setMotorTargets(int16_t left, int16_t right) {
  const int16_t limitedLeft = constrain(left, -255, 255);
  const int16_t limitedRight = constrain(right, -255, 255);

  if (limitedLeft == gCommandLeft && limitedRight == gCommandRight) {
    return;
  }

  gCommandLeft = limitedLeft;
  gCommandRight = limitedRight;

  Serial.print(F("[motor] target L="));
  Serial.print(gCommandLeft);
  Serial.print(F(" R="));
  Serial.println(gCommandRight);

  if (gEspBridgeActive) {
    espSerial.print(F("STM_MOTOR targetL="));
    espSerial.print(gCommandLeft);
    espSerial.print(F(" targetR="));
    espSerial.println(gCommandRight);
  }

  if (gAssistEnabled) {
    if (gCommandLeft == 0 && gCommandRight == 0) {
      applyBrakes();
    } else {
      releaseBrakes();
    }
  } else if (gCommandLeft != 0 || gCommandRight != 0) {
    // If user commands motion with assist disabled, make sure brakes are off.
    releaseBrakes();
  }

  recomputeMotorTargets();
}

void serviceMotorOutputs() {
  if (gMotorTargetLeft == gMotorAppliedLeft && gMotorTargetRight == gMotorAppliedRight) {
    return;
  }

  const bool enable = (gMotorTargetLeft != 0) || (gMotorTargetRight != 0);
  setMotorStandby(enable);
  if (enable) {
    applyMotorOutputs(gMotorTargetLeft, gMotorTargetRight);
  } else {
    applyMotorOutputs(0, 0);
  }

  gMotorAppliedLeft = gMotorTargetLeft;
  gMotorAppliedRight = gMotorTargetRight;
}

void recomputeMotorTargets() {
  auto clampBoost = [](int32_t value) -> int16_t {
    if (value > kAssistMaxBoost) {
      return kAssistMaxBoost;
    }
    if (value < -kAssistMaxBoost) {
      return -kAssistMaxBoost;
    }
    return static_cast<int16_t>(value);
  };

  auto rampTowards = [](int16_t& current, int16_t target) {
    int16_t delta = target - current;
    if (delta > kAssistRampStep) {
      delta = kAssistRampStep;
    } else if (delta < -kAssistRampStep) {
      delta = -kAssistRampStep;
    }
    current += delta;
  };

  int32_t assistGoalLeft = 0;
  int32_t assistGoalRight = 0;

  if (gAssistEnabled && gLevelState.valid && gLevelState.profile) {
    const float pitchDeg = gLevelState.axisYDeg;
    if (pitchDeg > kAssistPitchThresholdDeg && gCommandLeft > 0 && gCommandRight > 0) {
      const float gradeExcess = pitchDeg - kAssistPitchThresholdDeg;
      const int32_t boost = static_cast<int32_t>(min(kAssistGainPerDeg * gradeExcess, static_cast<float>(kAssistMaxBoost)));
      assistGoalLeft = boost;
      assistGoalRight = boost;
    } else if (pitchDeg < -kAssistPitchThresholdDeg && gCommandLeft < 0 && gCommandRight < 0) {
      const float gradeExcess = (-pitchDeg) - kAssistPitchThresholdDeg;
      const int32_t boost = static_cast<int32_t>(min(kAssistGainPerDeg * gradeExcess, static_cast<float>(kAssistMaxBoost)));
      assistGoalLeft = -boost;
      assistGoalRight = -boost;
    }
  }

  rampTowards(gAssistBoostLeft, clampBoost(assistGoalLeft));
  rampTowards(gAssistBoostRight, clampBoost(assistGoalRight));

  const int32_t finalLeft = static_cast<int32_t>(gCommandLeft) + gAssistBoostLeft;
  const int32_t finalRight = static_cast<int32_t>(gCommandRight) + gAssistBoostRight;

  gMotorTargetLeft = constrain(static_cast<int16_t>(finalLeft), -255, 255);
  gMotorTargetRight = constrain(static_cast<int16_t>(finalRight), -255, 255);
}

void sendEspStatus(uint32_t nowMs) {
  if (!gEspBridgeActive) {
    return;
  }

  espSerial.print(F("STM_STATUS uptime="));
  espSerial.print(nowMs / 1000.0f, 2);
  espSerial.print(F("s led="));
  espSerial.print(gLedState ? 1 : 0);

  if (gLevelState.valid && gLevelState.profile) {
    espSerial.print(F(" orient="));
    espSerial.print(gLevelState.profile->orientationLabel);
    espSerial.print(F(" axisX="));
    espSerial.print(gLevelState.axisXDeg, 1);
    espSerial.print(F("deg axisY="));
    espSerial.print(gLevelState.axisYDeg, 1);
    espSerial.print(F("deg"));
  } else {
    espSerial.print(F(" orient=UNKNOWN"));
  }

  espSerial.print(F(" btn="));
  espSerial.print(gButtonState ? 1 : 0);
  espSerial.print(F(" assist="));
  espSerial.print(gAssistEnabled ? 1 : 0);
  espSerial.print(F(" cmdL="));
  espSerial.print(gCommandLeft);
  espSerial.print(F(" cmdR="));
  espSerial.print(gCommandRight);
  espSerial.print(F(" brake="));
  espSerial.print(gBrakesEngaged ? 1 : 0);
  espSerial.print(F(" motL="));
  espSerial.print(gMotorAppliedLeft);
  espSerial.print(F(" motR="));
  espSerial.print(gMotorAppliedRight);

  espSerial.println();
}

void sendEspTelemetry(const ImuSample& sample, const LevelState& state, uint32_t nowMs) {
  if (!gEspBridgeActive || !sample.valid) {
    return;
  }

  espSerial.print(F("STM_IMU t="));
  espSerial.print(nowMs);
  espSerial.print(F("ms ax="));
  espSerial.print(sample.ax, 3);
  espSerial.print(F(" ay="));
  espSerial.print(sample.ay, 3);
  espSerial.print(F(" az="));
  espSerial.print(sample.az, 3);
  espSerial.print(F(" gx="));
  espSerial.print(sample.gx, 1);
  espSerial.print(F(" gy="));
  espSerial.print(sample.gy, 1);
  espSerial.print(F(" gz="));
  espSerial.print(sample.gz, 1);
  espSerial.print(F(" tempC="));
  espSerial.print(sample.tempC, 1);

  if (state.valid && state.profile) {
    espSerial.print(F(" ref="));
    espSerial.print(state.profile->orientationLabel);
    espSerial.print(F(" axisX="));
    espSerial.print(state.axisXDeg, 1);
    espSerial.print(F("deg axisY="));
    espSerial.print(state.axisYDeg, 1);
    espSerial.print(F("deg"));
  }

  espSerial.print(F(" btn="));
  espSerial.print(gButtonState ? 1 : 0);
  espSerial.print(F(" assist="));
  espSerial.print(gAssistEnabled ? 1 : 0);
  espSerial.print(F(" cmdL="));
  espSerial.print(gCommandLeft);
  espSerial.print(F(" cmdR="));
  espSerial.print(gCommandRight);
  espSerial.print(F(" brake="));
  espSerial.print(gBrakesEngaged ? 1 : 0);
  espSerial.print(F(" motL="));
  espSerial.print(gMotorAppliedLeft);
  espSerial.print(F(" motR="));
  espSerial.print(gMotorAppliedRight);

  espSerial.println();
}

bool probeMpu(uint8_t address, uint8_t& whoAmI) {
  imuWire.beginTransmission(address);
  imuWire.write(kRegWhoAmI);
  if (imuWire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = imuWire.requestFrom(address, static_cast<uint8_t>(1));
  if (received != 1) {
    return false;
  }

  whoAmI = imuWire.read();
  return true;
}

bool initMpu9250() {
  uint8_t whoAmI = 0;
  if (probeMpu(kMpuPrimaryAddr, whoAmI)) {
    gMpuAddress = kMpuPrimaryAddr;
  } else if (probeMpu(kMpuSecondaryAddr, whoAmI)) {
    gMpuAddress = kMpuSecondaryAddr;
  } else {
    return false;
  }

  if (whoAmI != 0x71 && whoAmI != 0x73) {
    Serial.print(F("[imu] Unexpected WHO_AM_I: 0x"));
    Serial.println(whoAmI, HEX);
  } else {
    Serial.print(F("[imu] WHO_AM_I: 0x"));
    Serial.println(whoAmI, HEX);
  }

  if (!mpuWriteByte(kRegPwrMgmt1, 0x80)) { // reset
    Serial.println(F("[imu] Reset command failed."));
    return false;
  }
  delay(100);

  if (!mpuWriteByte(kRegPwrMgmt1, 0x01)) { // auto select clock
    return false;
  }
  if (!mpuWriteByte(kRegPwrMgmt2, 0x00)) { // enable all sensors
    return false;
  }
  if (!mpuWriteByte(kRegSmplrtDiv, 0x04)) { // 200 Hz
    return false;
  }
  if (!mpuWriteByte(kRegConfig, 0x03)) { // DLPF ~44 Hz
    return false;
  }
  if (!mpuWriteByte(kRegGyroConfig, 0x00)) { // +/-250 dps
    return false;
  }
  if (!mpuWriteByte(kRegAccelConfig, 0x00)) { // +/-2 g
    return false;
  }
  if (!mpuWriteByte(kRegAccelConfig2, 0x03)) { // DLPF ~44 Hz
    return false;
  }

  Serial.println(F("[imu] MPU9250 initialised."));
  return true;
}

bool mpuWriteByte(uint8_t reg, uint8_t value) {
  imuWire.beginTransmission(gMpuAddress);
  imuWire.write(reg);
  imuWire.write(value);
  return imuWire.endTransmission() == 0;
}

bool mpuReadBytes(uint8_t reg, uint8_t* buffer, size_t length) {
  imuWire.beginTransmission(gMpuAddress);
  imuWire.write(reg);
  if (imuWire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = imuWire.requestFrom(gMpuAddress, static_cast<uint8_t>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    buffer[i] = imuWire.read();
  }
  return true;
}

bool readMpu9250(ImuSample& sample) {
  uint8_t raw[14] = {0};
  if (!mpuReadBytes(kRegAccelXoutH, raw, sizeof(raw))) {
    return false;
  }

  const int16_t axRaw = (static_cast<int16_t>(raw[0]) << 8) | raw[1];
  const int16_t ayRaw = (static_cast<int16_t>(raw[2]) << 8) | raw[3];
  const int16_t azRaw = (static_cast<int16_t>(raw[4]) << 8) | raw[5];
  const int16_t tempRaw = (static_cast<int16_t>(raw[6]) << 8) | raw[7];
  const int16_t gxRaw = (static_cast<int16_t>(raw[8]) << 8) | raw[9];
  const int16_t gyRaw = (static_cast<int16_t>(raw[10]) << 8) | raw[11];
  const int16_t gzRaw = (static_cast<int16_t>(raw[12]) << 8) | raw[13];

  sample.ax = static_cast<float>(axRaw) / kAccelScale;
  sample.ay = static_cast<float>(ayRaw) / kAccelScale;
  sample.az = static_cast<float>(azRaw) / kAccelScale;
  sample.tempC = static_cast<float>(tempRaw) / kTempScale + kTempOffset;
  sample.gx = static_cast<float>(gxRaw) / kGyroScale;
  sample.gy = static_cast<float>(gyRaw) / kGyroScale;
  sample.gz = static_cast<float>(gzRaw) / kGyroScale;
  return true;
}

void updateLevelState(const ImuSample& sample) {
  if (!sample.valid) {
    gLevelState.valid = false;
    gLevelState.orientation = Orientation::Unknown;
    gLevelState.profile = nullptr;
    gLevelState.axisXDeg = 0.0f;
    gLevelState.axisYDeg = 0.0f;
    gLevelState.bubbleX = 0.0f;
    gLevelState.bubbleY = 0.0f;
    recomputeMotorTargets();
    return;
  }

  if (!gAccelFilterInitialised) {
    gFilteredAx = sample.ax;
    gFilteredAy = sample.ay;
    gFilteredAz = sample.az;
    gAccelFilterInitialised = true;
  } else {
    gFilteredAx += kAccelFilterAlpha * (sample.ax - gFilteredAx);
    gFilteredAy += kAccelFilterAlpha * (sample.ay - gFilteredAy);
    gFilteredAz += kAccelFilterAlpha * (sample.az - gFilteredAz);
  }

  const float components[3] = {gFilteredAx, gFilteredAy, gFilteredAz};
  const Orientation orientation = detectOrientation(components, gCurrentOrientation);
  gCurrentOrientation = orientation;
  gLevelState.orientation = orientation;

  if (orientation == Orientation::Unknown) {
    gLevelState.valid = false;
    gLevelState.profile = nullptr;
    gLevelState.axisXDeg = 0.0f;
    gLevelState.axisYDeg = 0.0f;
    gLevelState.bubbleX = 0.0f;
    gLevelState.bubbleY = 0.0f;
    recomputeMotorTargets();
    return;
  }

  const OrientationProfile* profile = findProfile(orientation);
  if (!profile) {
    gLevelState.valid = false;
    gLevelState.profile = nullptr;
    gLevelState.axisXDeg = 0.0f;
    gLevelState.axisYDeg = 0.0f;
    gLevelState.bubbleX = 0.0f;
    gLevelState.bubbleY = 0.0f;
    recomputeMotorTargets();
    return;
  }

  const float vertical = axisValue(components, {profile->verticalAxis,
                                                profile->verticalSign,
                                                nullptr});
  if (fabsf(vertical) < 0.1f) {
    gLevelState.valid = false;
    gLevelState.profile = profile;
    gLevelState.axisXDeg = 0.0f;
    gLevelState.axisYDeg = 0.0f;
    gLevelState.bubbleX = 0.0f;
    gLevelState.bubbleY = 0.0f;
    recomputeMotorTargets();
    return;
  }

  const float axisX = axisValue(components, profile->axisX);
  const float axisY = axisValue(components, profile->axisY);

  const float axisXDeg = atan2f(axisX, vertical) * RAD_TO_DEG;
  const float axisYDeg = atan2f(axisY, vertical) * RAD_TO_DEG;

  gLevelState.axisXDeg = axisXDeg;
  gLevelState.axisYDeg = axisYDeg;
  gLevelState.bubbleX = constrain(axisXDeg / kMaxTiltDeg, -1.0f, 1.0f);
  gLevelState.bubbleY = constrain(axisYDeg / kMaxTiltDeg, -1.0f, 1.0f);
  gLevelState.profile = profile;
  gLevelState.valid = true;

  recomputeMotorTargets();
}

void updateDisplay() {
  if (!gDisplayReady) {
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  const uint32_t now = millis();

  if (!gImuReady) {
    display.println(F("MPU9250 missing"));
    display.println(F("Check PB7/PB8"));
  } else if (!gLevelState.valid || gLevelState.profile == nullptr) {
    display.println(F("MPU9250 ready"));
    display.println(F("Stabilising..."));
  } else {
    display.print(gLevelState.profile->orientationLabel);
    display.setCursor(display.width() - 48, 0);
    display.print(F("dT:"));
    display.print(gImuSample.tempC, 1);

    display.setCursor(0, 8);
    display.print(gLevelState.profile->axisX.label);
    display.print(':');
    display.print(gLevelState.axisXDeg, 1);
    display.setCursor(52, 8);
    display.print(gLevelState.profile->axisY.label);
    display.print(':');
    display.print(gLevelState.axisYDeg, 1);
    display.setCursor(96, 8);
    if (fabsf(gLevelState.axisXDeg) <= kLevelThresholdDeg &&
        fabsf(gLevelState.axisYDeg) <= kLevelThresholdDeg) {
      display.print(F("LEVEL"));
    } else {
      display.print(F("ADJ"));
    }

    const int16_t frameWidth = 96;
    const int16_t frameHeight = 16;
    const int16_t frameX = (display.width() - frameWidth) / 2;
    const int16_t frameY = 16;
    const int16_t frameCenterX = frameX + frameWidth / 2;
    const int16_t frameCenterY = frameY + frameHeight / 2;
    const int16_t bubbleRadius = 3;

    display.drawRect(frameX, frameY, frameWidth, frameHeight, SSD1306_WHITE);
    display.drawLine(frameCenterX, frameY + 1, frameCenterX, frameY + frameHeight - 2, SSD1306_WHITE);
    display.drawLine(frameX + 1, frameCenterY, frameX + frameWidth - 2, frameCenterY, SSD1306_WHITE);

    const int16_t bubbleMaxX = (frameWidth / 2) - bubbleRadius - 2;
    const int16_t bubbleMaxY = (frameHeight / 2) - bubbleRadius - 2;

    const int16_t bubblePixelX =
        frameCenterX + static_cast<int16_t>(gLevelState.bubbleX * bubbleMaxX);
    const int16_t bubblePixelY =
        frameCenterY + static_cast<int16_t>(gLevelState.bubbleY * bubbleMaxY);

    display.fillCircle(bubblePixelX, bubblePixelY, bubbleRadius, SSD1306_WHITE);
    display.fillCircle(bubblePixelX, bubblePixelY, bubbleRadius - 1, SSD1306_BLACK);
  }

  display.setCursor(0, 24);
  if (now < gAssistMessageUntilMs) {
    display.print(F("Assist "));
    display.print(gAssistEnabled ? F("ON") : F("OFF"));
  } else {
    display.print(F("Cmd:"));
    display.print(gCommandLeft);
    display.print(',');
    display.print(gCommandRight);
    display.print(F(" Out:"));
    display.print(gMotorAppliedLeft);
    display.print(',');
    display.print(gMotorAppliedRight);
    display.print(F(" Br:"));
    display.print(gBrakesEngaged ? F("ON") : F("OFF"));
  }

  display.display();
}

void logImuToSerial(const ImuSample& sample, uint32_t nowMs) {
  Serial.print(F("[imu] t="));
  Serial.print(nowMs);
  Serial.print(F("ms "));
  Serial.print(F("a[g]:"));
  Serial.print(sample.ax, 3);
  Serial.print(',');
  Serial.print(sample.ay, 3);
  Serial.print(',');
  Serial.print(sample.az, 3);
  Serial.print(F(" g[dps]:"));
  Serial.print(sample.gx, 1);
  Serial.print(',');
  Serial.print(sample.gy, 1);
  Serial.print(',');
  Serial.print(sample.gz, 1);
  Serial.print(F(" tempC:"));
  Serial.print(sample.tempC, 1);
  if (gLevelState.valid && gLevelState.profile) {
    Serial.print(F(" ref="));
    Serial.print(gLevelState.profile->orientationLabel);
    Serial.print(F(" "));
    Serial.print(gLevelState.profile->axisX.label);
    Serial.print('=');
    Serial.print(gLevelState.axisXDeg, 1);
    Serial.print(F("deg "));
    Serial.print(gLevelState.profile->axisY.label);
    Serial.print('=');
    Serial.print(gLevelState.axisYDeg, 1);
    Serial.print(F("deg"));
  }
  Serial.print(F(" btn="));
  Serial.print(gButtonState ? F("ON") : F("OFF"));
  Serial.print(F(" assist="));
  Serial.print(gAssistEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" cmdL="));
  Serial.print(gCommandLeft);
  Serial.print(F(" cmdR="));
  Serial.print(gCommandRight);
  Serial.print(F(" brake="));
  Serial.print(gBrakesEngaged ? F("ON") : F("OFF"));
  Serial.print(F(" motL="));
  Serial.print(gMotorAppliedLeft);
  Serial.print(F(" motR="));
  Serial.print(gMotorAppliedRight);
  Serial.println();

  sendEspTelemetry(sample, gLevelState, nowMs);
  sendEspStatus(nowMs);
}
