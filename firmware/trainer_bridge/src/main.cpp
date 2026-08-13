#include <Arduino.h>
#include <math.h>

// Wired, additive trainer bridge. This firmware never emits ARM or active
// throttle commands. Confirm TX16S electrical levels, jack wiring, PPM
// polarity, channel order, and EdgeTX trainer setup before making a connection.

#ifndef PD_TRAINER_PPM_PIN
#define PD_TRAINER_PPM_PIN 2
#endif

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t HOST_WATCHDOG_US = 250000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 100;
constexpr uint16_t PPM_FRAME_US = 22500;
constexpr uint16_t PPM_PULSE_US = 300;
constexpr uint8_t CHANNEL_COUNT = 8;
constexpr float MAX_ROLL = 0.15f;
constexpr float MAX_YAW = 0.15f;

float contribution[CHANNEL_COUNT] = {0.0f};
uint32_t lastValidPacketUs = 0;
uint32_t heartbeatSequence = 0;
uint32_t lastHeartbeatMs = 0;

void neutralize() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; ++i) contribution[i] = 0.0f;
}

uint8_t xorChecksum(const char* value) {
  uint8_t checksum = 0;
  while (*value) checksum ^= static_cast<uint8_t>(*value++);
  return checksum;
}

bool parseHexByte(const char* value, uint8_t* parsed) {
  char* end = nullptr;
  const unsigned long result = strtoul(value, &end, 16);
  if (end == value || *end != '\0' || result > 0xFF) return false;
  *parsed = static_cast<uint8_t>(result);
  return true;
}

bool acceptPacket(char* line) {
  char* finalComma = strrchr(line, ',');
  if (finalComma == nullptr) return false;
  *finalComma = '\0';
  uint8_t suppliedChecksum = 0;
  if (!parseHexByte(finalComma + 1, &suppliedChecksum)) return false;
  if (xorChecksum(line) != suppliedChecksum) return false;

  unsigned long sequence = 0;
  float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, throttle = 0.0f;
  const int fields = sscanf(
      line, "PD1,%lu,%f,%f,%f,%f", &sequence, &roll, &pitch, &yaw, &throttle);
  (void)sequence;
  if (fields != 5) return false;
  if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) ||
      !isfinite(throttle)) {
    return false;
  }
  if (roll < -1.0f || roll > 1.0f || pitch < -1.0f || pitch > 1.0f ||
      yaw < -1.0f || yaw > 1.0f || throttle < -1.0f || throttle > 1.0f) {
    return false;
  }

  // Independent firmware limits. Pitch and throttle are ignored even if a
  // malformed/older host attempts to make them non-zero.
  contribution[0] = constrain(roll, -MAX_ROLL, MAX_ROLL);
  contribution[1] = 0.0f;
  contribution[2] = 0.0f;
  contribution[3] = constrain(yaw, -MAX_YAW, MAX_YAW);
  lastValidPacketUs = micros();
  return true;
}

void serviceSerial() {
  static char line[128] = {0};
  static size_t position = 0;
  while (Serial.available()) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\n' || value == '\r') {
      if (position > 0) {
        line[position] = '\0';
        if (!acceptPacket(line)) neutralize();
        position = 0;
      }
    } else if (position < sizeof(line) - 1) {
      line[position++] = value;
    } else {
      position = 0;
      neutralize();
    }
  }
}

uint16_t channelPulse(uint8_t channel) {
  const float bounded = constrain(contribution[channel], -1.0f, 1.0f);
  return static_cast<uint16_t>(1500.0f + bounded * 500.0f);
}

void servicePpm() {
  static bool pulseHigh = false;
  static uint8_t channel = 0;
  static uint32_t frameUsedUs = 0;
  static uint32_t nextEdgeUs = 0;
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextEdgeUs) < 0) return;

  if (!pulseHigh) {
    digitalWrite(PD_TRAINER_PPM_PIN, HIGH);
    pulseHigh = true;
    nextEdgeUs = now + PPM_PULSE_US;
    return;
  }

  digitalWrite(PD_TRAINER_PPM_PIN, LOW);
  pulseHigh = false;
  if (channel < CHANNEL_COUNT) {
    const uint16_t pulse = channelPulse(channel++);
    frameUsedUs += pulse;
    nextEdgeUs = now + max<uint16_t>(1, pulse - PPM_PULSE_US);
  } else {
    const uint32_t sync =
        PPM_FRAME_US > frameUsedUs ? PPM_FRAME_US - frameUsedUs : 3000;
    nextEdgeUs = now + max<uint32_t>(1, sync - PPM_PULSE_US);
    channel = 0;
    frameUsedUs = 0;
  }
}

void setup() {
  neutralize();
  pinMode(PD_TRAINER_PPM_PIN, OUTPUT);
  digitalWrite(PD_TRAINER_PPM_PIN, LOW);
  Serial.begin(SERIAL_BAUD);
}

void loop() {
  serviceSerial();
  if (lastValidPacketUs == 0 || micros() - lastValidPacketUs > HOST_WATCHDOG_US) {
    neutralize();
  }
  servicePpm();

  const uint32_t nowMs = millis();
  if (nowMs - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    Serial.print("HB,");
    Serial.println(heartbeatSequence++);
    lastHeartbeatMs = nowMs;
  }
}
