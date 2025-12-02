#include <Arduino.h>
#include <AudioDelayFeedback.h>
#include <MozziGuts.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>

// -----------------------------------------------------------------------------
// Minimal Mozzi patch turned into a radio + MSP reactive feedback instrument.
// - Fast path: oscillator -> drive shaper -> delay feedback line.
// - Control path: map RC gimbals/switches and MSP telemetry into parameters.
//   * TX16S gimbals (analog ins) steer pitch, delay time, feedback, drive.
//   * MSP (ATTITUDE + ANALOG) leans into the modulation so the drone’s motion
//     and battery state change the texture.
// Keep it integer-only in the audio lane and avoid blocking anywhere Mozzi
// calls us.
// Mozzi docs: https://sensorium.github.io/Mozzi/
// -----------------------------------------------------------------------------

// --- RC input mapping (wire your receiver to these pins).
constexpr uint8_t PIN_CH_THROTTLE = A0;  // Base pitch
constexpr uint8_t PIN_CH_ROLL = A1;      // Delay time
constexpr uint8_t PIN_CH_PITCH = A2;     // Feedback amount
constexpr uint8_t PIN_CH_YAW = A3;       // Vibrato depth
constexpr uint8_t PIN_CH_AUX1 = A4;      // Drive/saturation

// --- MSP constants (V1 frames, Betaflight/Cleanflight compatible).
constexpr uint8_t MSP_ATTITUDE = 108;
constexpr uint8_t MSP_ANALOG = 110;
constexpr uint8_t MSP_MAX_PAYLOAD = 32;
constexpr uint8_t MSP_REQUEST_INTERVAL = 32;  // Control ticks between polls.

struct MspTelemetry {
  int16_t roll_decideg = 0;
  int16_t pitch_decideg = 0;
  int16_t yaw_decideg = 0;
  uint8_t vbat = 0;      // 0.1 V units
  uint16_t rssi = 0;     // 0..1023 typical
};

struct MspParser {
  enum State { WAIT_HEADER1, WAIT_HEADER2, WAIT_DIRECTION, READ_SIZE, READ_CMD,
               READ_PAYLOAD, READ_CHECKSUM };

  State state = WAIT_HEADER1;
  uint8_t data_size = 0;
  uint8_t command = 0;
  uint8_t checksum = 0;
  uint8_t payload_index = 0;
  uint8_t payload[MSP_MAX_PAYLOAD] = {0};
};

MspTelemetry telemetry;
MspParser parser;
uint8_t msp_request_phase = 0;

// --- Audio building blocks.
Oscil<SIN2048_NUM_CELLS, AUDIO_RATE> osc(SIN2048_DATA);  // sine oscillator
AudioDelayFeedback<128> delay;  // 128 sample (~2.9 ms @ 44.1 kHz) buffer

// --- Live control state (refreshed at control rate).
uint16_t osc_frequency = 220;
uint8_t vibrato_depth = 0;
uint8_t delay_time = 64;
uint8_t feedback_level = 200;
uint8_t drive_amount = 0;  // 0..64

// --- Utility: tiny soft clip for drive (fast path friendly).
inline int16_t softClip(int16_t sample, uint8_t drive) {
  // drive 0..64, scales pre-clip gain and uses polynomial soft clip to avoid
  // harsh wrap-around.  All integer math for speed.
  int32_t boosted = sample * (64 + drive);  // up to ~ (127 * 128) * 64 ~ 1M
  boosted >>= 6;  // scale back to 16-bit-ish range

  // cubic soft clip approximation: x - x^3/3, in fixed-point form.
  int32_t x = boosted;
  int32_t x3 = (x * x * x) >> 15;  // keep in range
  int32_t shaped = x - (x3 / 3);

  // Saturate to int16_t
  if (shaped > 32767) return 32767;
  if (shaped < -32768) return -32768;
  return static_cast<int16_t>(shaped);
}

// --- MSP helpers -----------------------------------------------------------
void resetParser() {
  parser.state = MspParser::WAIT_HEADER1;
  parser.data_size = 0;
  parser.command = 0;
  parser.checksum = 0;
  parser.payload_index = 0;
}

void handleMspFrame(uint8_t command, const uint8_t* payload, uint8_t size) {
  if (command == MSP_ATTITUDE && size >= 6) {
    telemetry.roll_decideg = static_cast<int16_t>(payload[0] | (payload[1] << 8));
    telemetry.pitch_decideg = static_cast<int16_t>(payload[2] | (payload[3] << 8));
    telemetry.yaw_decideg = static_cast<int16_t>(payload[4] | (payload[5] << 8));
  } else if (command == MSP_ANALOG && size >= 7) {
    telemetry.vbat = payload[0];
    telemetry.rssi = static_cast<uint16_t>(payload[5] | (payload[6] << 8));
  }
}

void consumeMspByte(uint8_t b) {
  switch (parser.state) {
    case MspParser::WAIT_HEADER1:
      parser.state = (b == '$') ? MspParser::WAIT_HEADER2 : MspParser::WAIT_HEADER1;
      break;
    case MspParser::WAIT_HEADER2:
      parser.state = (b == 'M') ? MspParser::WAIT_DIRECTION : MspParser::WAIT_HEADER1;
      break;
    case MspParser::WAIT_DIRECTION:
      parser.state = (b == '>' || b == '!') ? MspParser::READ_SIZE : MspParser::WAIT_HEADER1;
      break;
    case MspParser::READ_SIZE:
      if (b > MSP_MAX_PAYLOAD) {
        resetParser();
        break;
      }
      parser.data_size = b;
      parser.checksum = b;
      parser.payload_index = 0;
      parser.state = MspParser::READ_CMD;
      break;
    case MspParser::READ_CMD:
      parser.command = b;
      parser.checksum ^= b;
      parser.state = (parser.data_size == 0) ? MspParser::READ_CHECKSUM : MspParser::READ_PAYLOAD;
      break;
    case MspParser::READ_PAYLOAD:
      parser.payload[parser.payload_index++] = b;
      parser.checksum ^= b;
      if (parser.payload_index >= parser.data_size) {
        parser.state = MspParser::READ_CHECKSUM;
      }
      break;
    case MspParser::READ_CHECKSUM:
      if (parser.checksum == b) {
        handleMspFrame(parser.command, parser.payload, parser.data_size);
      }
      resetParser();
      break;
  }
}

void pollMsp() {
  while (Serial1.available()) {
    consumeMspByte(static_cast<uint8_t>(Serial1.read()));
  }
}

void sendMspRequest(uint8_t command) {
  uint8_t checksum = 0 ^ command;  // zero-length payload
  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');
  Serial1.write(static_cast<uint8_t>(0));
  Serial1.write(command);
  Serial1.write(checksum);
}

// --- Setup / Control / Audio ---------------------------------------------
void setup() {
  Serial1.begin(115200);  // MSP telemetry bus
  resetParser();

  startMozzi();
  osc.setFreq(osc_frequency);
}

void updateControl() {
  // --- MSP polling (runs every control tick, throttled by MSP_REQUEST_INTERVAL)
  pollMsp();
  if ((msp_request_phase++ % MSP_REQUEST_INTERVAL) == 0) {
    sendMspRequest((msp_request_phase / MSP_REQUEST_INTERVAL) % 2 ? MSP_ANALOG : MSP_ATTITUDE);
  }

  // --- RC inputs
  int throttle_raw = mozziAnalogRead(PIN_CH_THROTTLE);
  int roll_raw = mozziAnalogRead(PIN_CH_ROLL);
  int pitch_raw = mozziAnalogRead(PIN_CH_PITCH);
  int yaw_raw = mozziAnalogRead(PIN_CH_YAW);
  int aux1_raw = mozziAnalogRead(PIN_CH_AUX1);

  // Base oscillator frequency driven by throttle, nudged by MSP pitch attitude.
  int16_t pitch_mod = telemetry.pitch_decideg / 40;  // ~ +/-200 -> +/-5 Hz-ish
  osc_frequency = constrain(static_cast<int32_t>(map(throttle_raw, 0, 1023, 80, 640)) + pitch_mod, 40, 880);
  osc.setFreq(osc_frequency);

  // Vibrato depth from yaw stick plus MSP yaw drift. Limit to keep it musical.
  vibrato_depth = constrain(map(yaw_raw, 0, 1023, 0, 18) + (abs(telemetry.yaw_decideg) / 120), 0, 24);

  // Delay time on roll; add attitude roll so banking leans the echo.
  delay_time = constrain(map(roll_raw, 0, 1023, 8, 120) + (abs(telemetry.roll_decideg) / 100), 4, 124);
  delay.setDelayTime(delay_time);

  // Feedback from pitch stick, reduced as battery sags to keep ears safe.
  uint8_t battery_drop = (telemetry.vbat < 100) ? static_cast<uint8_t>(100 - telemetry.vbat) : 0;  // 10.0 V ref
  uint8_t feedback_target = constrain(map(pitch_raw, 0, 1023, 140, 235), 120, 240);
  feedback_level = (battery_drop > 20) ? feedback_target - 10 : feedback_target;  // soften when voltage low
  feedback_level = min<uint8_t>(feedback_level, 220);  // hard ceiling to dodge runaway howl
  delay.setFeedbackLevel(feedback_level);

  // Drive control from AUX1 plus RSSI (stronger link = hotter signal).
  uint8_t rssi_bonus = (telemetry.rssi > 800) ? 8 : 0;
  drive_amount = constrain(map(aux1_raw, 0, 1023, 0, 56) + rssi_bonus, 0, 64);
}

int updateAudio() {
  // Fast path: oscillator -> vibrato wiggle -> drive -> delay feedback.
  static uint16_t vibrato_phase = 0;
  vibrato_phase += vibrato_depth + 1;  // tiny phasor for vibrato (integer only)
  int16_t vibrato = static_cast<int16_t>((vibrato_phase >> 8) - 128);  // -128..127
  int16_t detuned = static_cast<int16_t>((vibrato * vibrato_depth) >> 8);

  osc.setFreq(osc_frequency + detuned);  // safe because vibrato_depth is small

  int16_t sample = osc.next();
  int16_t driven = softClip(sample, drive_amount);
  return delay.next(driven);  // Feedback handled by AudioDelayFeedback
}

void loop() {
  audioHook();  // Mozzi scheduler. Nothing blocking lives here.
}
