// Cubli 1D firmware: ESP32 + MPU6050 + Nidec 24H reaction wheel.
//
// Streams IMU + encoder frames at 200 Hz over USB serial and applies motor commands
// from the ROS driver (cubli_control/hw_driver_node.py). Protocol (NMEA-style checksum):
//   ESP32 -> PC  $S,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<enc>*HH   (m/s^2, rad/s, counts)
//                $X,<t_us>,<tilt>,<rate>,<wheel_speed>,<torque>,<mode>*HH   on-board mode only
//                $I,<text>*HH
//   PC -> ESP32  $T,<effort -1..1>*HH   $B,<brake ms>*HH   $E,<0|1>*HH
//                $M,<0 passthrough|1 on-board>*HH   $K,<k1>,<k2>,<k3>*HH
//                $P,<name>,<value>*HH   $A,<ax>,<ay>,<az>*HH (body axes, e.g. +x,-z,+y)
// Safety: the motor stops if no $T arrives for 100 ms, and starts disabled until $E,1.
//
// On-board balance ($M,1): the estimator (gyro-bias calibration, complementary filter
// with lever-arm compensation, 8-sample encoder speed) and the LQR run here at 200 Hz, so
// the balance loop doesn't wait for USB or the PC. $X modes: 1 calibrating (keep still),
// 2 idle ($T applies, e.g. jump spin-up), 3 balancing ($T ignored). It engages within
// `engage`, or within `catch` for jump_timeout s after a $B, and gives up beyond
// `disengage` while moving away from upright, or beyond `catch`. It stops balancing if
// nothing valid arrives from the PC for 2 s. $P names: engage, disengage, catch,
// jump_timeout, max_torque, stall_torque, no_load_speed, max_effort, angle_offset, tau,
// imu_offset, cpr, enc_sign. Python twin of this code: cubli_control/onboard.py.
//
// Arduino IDE / arduino-cli with the ESP32 core (2.x or 3.x). No extra libraries needed.

#include <Arduino.h>
#include <Wire.h>

// ---- Wiring: change to match your build ----------------------------------------
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;
constexpr int PIN_PWM = 25;      // Nidec PWM input (inverted: 100% duty = stopped)
constexpr int PIN_DIR = 26;      // Nidec CW/CCW
constexpr int PIN_BRAKE = 27;    // Nidec START/STOP
constexpr int PIN_ENC_A = 32;    // Nidec encoder A (open collector, pulled up to 3.3 V)
constexpr int PIN_ENC_B = 33;    // Nidec encoder B
constexpr int PIN_SERVO = -1;    // optional mechanical brake servo, -1 = none

// Check these against your motor's datasheet.
constexpr int BRAKE_ACTIVE = LOW;        // START/STOP level that brakes the motor
constexpr int DIR_POSITIVE = HIGH;       // CW/CCW level for positive effort
constexpr int SERVO_RELEASED_US = 1000;  // servo pulse with the brake off
constexpr int SERVO_ENGAGED_US = 1600;   // servo pulse pressing the brake on the wheel

// ---- Timing and scaling --------------------------------------------------------
constexpr uint32_t BAUD = 460800;
constexpr uint32_t PERIOD_US = 5000;     // 200 Hz
constexpr uint32_t CMD_TIMEOUT_MS = 100;
constexpr uint32_t LINK_TIMEOUT_MS = 2000;  // on-board mode: no valid PC line -> stop
constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_BITS = 8;
constexpr uint32_t PWM_MAX = (1u << PWM_BITS) - 1;
constexpr uint8_t MPU_ADDR = 0x68;
constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;          // +-2 g
constexpr float GYRO_SCALE = (PI / 180.0f) / 65.5f;          // +-500 deg/s

// ---- State -----------------------------------------------------------------------
volatile int32_t encCount = 0;
volatile uint8_t encState = 0;
const int8_t QUAD_TABLE[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

bool motorEnabled = false;
float effort = 0.0f;
uint32_t lastCmdMs = 0;
bool braking = false;
uint32_t brakeUntilMs = 0;
uint32_t nextUs = 0;
uint32_t lastRxMs = 0;

char lineBuf[96];
size_t lineLen = 0;

// ---- Encoder -------------------------------------------------------------------
void IRAM_ATTR onEncoder() {
  encState = ((encState << 2) | (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B)) & 0x0F;
  encCount += QUAD_TABLE[encState];
}

// ---- PWM (ESP32 Arduino core 2.x and 3.x have different LEDC APIs) ---------------
void pwmSetup(int pin, int channel, uint32_t freq, uint8_t bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, freq, bits);
#else
  ledcSetup(channel, freq, bits);
  ledcAttachPin(pin, channel);
#endif
}

void pwmWrite(int pin, int channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void servoWriteUs(int us) {
  if (PIN_SERVO < 0) return;
  pwmWrite(PIN_SERVO, 1, (uint32_t)us * 65535u / 20000u);  // 16-bit at 50 Hz
}

// ---- Serial protocol -------------------------------------------------------------
void sendLine(const char *body) {
  uint8_t sum = 0;
  for (const char *p = body; *p; p++) sum ^= (uint8_t)*p;
  char out[160];
  int n = snprintf(out, sizeof(out), "$%s*%02X\n", body, sum);
  if (n > 0) Serial.write((const uint8_t *)out, (size_t)min(n, (int)sizeof(out) - 1));
}

void sendInfo(const char *text) {
  char body[96];
  snprintf(body, sizeof(body), "I,%s", text);
  sendLine(body);
}

// ---- Motor -----------------------------------------------------------------------
void setEffort(float e) {
  if (!motorEnabled || braking) e = 0.0f;
  e = constrain(e, -1.0f, 1.0f);
  digitalWrite(PIN_DIR, e >= 0.0f ? DIR_POSITIVE : !DIR_POSITIVE);
  uint32_t duty = PWM_MAX - (uint32_t)lroundf(fabsf(e) * PWM_MAX);  // inverted PWM
  pwmWrite(PIN_PWM, 0, duty);
}

void startBrake(int ms) {
  braking = true;
  brakeUntilMs = millis() + (uint32_t)constrain(ms, 0, 2000);
  setEffort(0.0f);
  digitalWrite(PIN_BRAKE, BRAKE_ACTIVE);
  servoWriteUs(SERVO_ENGAGED_US);
  sendInfo("brake");
}

void endBrake() {
  braking = false;
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE);
  servoWriteUs(SERVO_RELEASED_US);
}

// ---- On-board estimator + LQR (mirrors estimator.py / controller_node.py) ----------
enum Mode : int { MODE_PASSTHROUGH = 0, MODE_CALIBRATING = 1, MODE_IDLE = 2, MODE_BALANCE = 3 };

struct OnboardParams {
  float engage = 0.12f, disengage = 0.16f, catchAngle = 0.4f, jumpTimeout = 1.5f;  // rad, s
  float maxTorque = 0.08f, stallTorque = 0.10f, noLoadSpeed = 600.0f, maxEffort = 1.0f;
  float angleOffset = 0.0f, tau = 0.5f, imuOffset = 0.106f;  // rad, s, m
  int cpr = 400, encSign = 1;
} prm;

constexpr int CALIB_SAMPLES = 200;
constexpr float CALIB_MAX_RATE_STD = 0.05f;  // rad/s
constexpr float GATE_ACCEL = 1.0f;           // m/s^2 from g: skip the accelerometer
constexpr float GATE_RATE = 3.0f;            // rad/s
constexpr float GRAVITY = 9.81f;
constexpr int SPEED_WINDOW = 8;

Mode mode = MODE_PASSTHROUGH;
float lqrK[3] = {0.0f, 0.0f, 0.0f};
int8_t axisIndex[3] = {0, 1, 2}, axisSign[3] = {1, 1, 1};  // body axis i = sign * sensor[index]
uint32_t catchUntilMs = 0;
bool catchArmed = false;

struct Estimator {
  int calibCount = 0;
  double sumGyro = 0.0, sumGyroSq = 0.0, sumAy = 0.0, sumAz = 0.0;
  float bias = 0.0f, angle = 0.0f, rate = 0.0f, accelF = 0.0f;
  bool haveT = false;
  uint32_t lastUs = 0;
  uint32_t speedT[SPEED_WINDOW + 1];
  float speedA[SPEED_WINDOW + 1];
  int speedN = 0, speedHead = 0;
  float speed = 0.0f;
} est;

void resetEstimator() {
  est = Estimator();
  if (mode != MODE_PASSTHROUGH) mode = MODE_CALIBRATING;
}

// Returns true once a still second has given the gyro bias and the starting tilt.
bool calibrate(float gx, float ay, float az) {
  est.calibCount++;
  est.sumGyro += gx;
  est.sumGyroSq += (double)gx * gx;
  est.sumAy += ay;
  est.sumAz += az;
  if (est.calibCount < CALIB_SAMPLES) return false;
  const double mean = est.sumGyro / est.calibCount;
  const double var = est.sumGyroSq / est.calibCount - mean * mean;
  if (var > (double)CALIB_MAX_RATE_STD * CALIB_MAX_RATE_STD) {  // it moved; start again
    est.calibCount = 0;
    est.sumGyro = est.sumGyroSq = est.sumAy = est.sumAz = 0.0;
    return false;
  }
  est.bias = (float)mean;
  est.angle = atan2f((float)est.sumAy, (float)est.sumAz);
  return true;
}

// Complementary filter with lever-arm compensation (TiltEstimator.update).
void updateTilt(float gx, float ay, float az, float dt) {
  const float rate = gx - est.bias;
  if (dt > 0.0f) {
    const float rawAccel = (rate - est.rate) / dt;
    const float k = dt / (0.01f + dt);  // ~16 Hz low-pass on the differentiated gyro
    est.accelF += k * (rawAccel - est.accelF);
  }
  est.rate = rate;
  est.angle += rate * dt;
  const float fy = ay + prm.imuOffset * est.accelF;
  const float fz = az + prm.imuOffset * rate * rate;
  if (fabsf(hypotf(fy, fz) - GRAVITY) < GATE_ACCEL && fabsf(rate) < GATE_RATE) {
    const float alpha = prm.tau / (prm.tau + dt);
    est.angle = alpha * est.angle + (1.0f - alpha) * atan2f(fy, fz);
  }
}

// Wheel speed from the encoder angle now and SPEED_WINDOW samples ago.
float updateSpeed(uint32_t tUs, float wheelAngle) {
  const int size = SPEED_WINDOW + 1;
  est.speedT[est.speedHead] = tUs;
  est.speedA[est.speedHead] = wheelAngle;
  est.speedHead = (est.speedHead + 1) % size;
  if (est.speedN < size) est.speedN++;
  const int oldest = (est.speedHead - est.speedN + size) % size;
  const uint32_t span = tUs - est.speedT[oldest];
  if (span > 0) est.speed = (wheelAngle - est.speedA[oldest]) / (span * 1e-6f);
  return est.speed;
}

float effortFor(float torque, float speed) {  // Nidec model, NidecMotor.effort_for
  const float e = torque / prm.stallTorque + speed / prm.noLoadSpeed;
  return constrain(e, -prm.maxEffort, prm.maxEffort);
}

// One 200 Hz step. Returns true and sets *effortOut while the on-board LQR drives the motor.
bool onboardStep(uint32_t tUs, const float accel[3], const float gyro[3], int32_t enc,
                 float *effortOut) {
  float a[3], g[3];
  for (int i = 0; i < 3; i++) {
    a[i] = axisSign[i] * accel[axisIndex[i]];
    g[i] = axisSign[i] * gyro[axisIndex[i]];
  }
  float dt = est.haveT ? (tUs - est.lastUs) * 1e-6f : 0.0f;
  dt = constrain(dt, 0.0f, 0.05f);
  est.haveT = true;
  est.lastUs = tUs;
  const float wheelAngle = (float)(prm.encSign * enc) * 2.0f * PI / (float)prm.cpr;
  const float speed = updateSpeed(tUs, wheelAngle);

  char body[96];
  if (mode == MODE_CALIBRATING) {
    if (calibrate(g[0], a[1], a[2])) {
      mode = MODE_IDLE;
      snprintf(body, sizeof(body), "calibrated: gyro bias %+.4f rad/s, tilt %+.3f rad",
               est.bias, est.angle);
      sendInfo(body);
    }
    snprintf(body, sizeof(body), "X,%lu,0.00000,0.0000,%.2f,0.00000,%d", (unsigned long)tUs,
             speed, (int)mode);
    sendLine(body);
    return false;
  }
  updateTilt(g[0], a[1], a[2], dt);

  const float angle = est.angle - prm.angleOffset;
  const float rate = est.rate;
  const bool linkOk = millis() - lastRxMs < LINK_TIMEOUT_MS;
  if (mode == MODE_BALANCE) {
    const bool movingAway = angle * rate > 0.0f;
    const bool lost = fabsf(angle) > prm.catchAngle ||
                      (fabsf(angle) > max(prm.disengage, prm.engage) && movingAway);
    if (lost || !motorEnabled || !linkOk) {
      mode = MODE_IDLE;
      sendInfo(linkOk ? "onboard: lost balance" : "onboard: PC link lost, motor off");
    }
  } else if (motorEnabled && linkOk) {
    if (catchArmed && (int32_t)(millis() - catchUntilMs) >= 0) catchArmed = false;
    const bool catching = catchArmed && fabsf(angle) < max(prm.catchAngle, prm.engage);
    if (fabsf(angle) < prm.engage || catching) {
      mode = MODE_BALANCE;
      catchArmed = false;
    }
  }

  float torque = 0.0f;
  bool driving = false;
  if (mode == MODE_BALANCE) {
    const float u = -(lqrK[0] * angle + lqrK[1] * rate + lqrK[2] * speed);
    torque = constrain(u, -prm.maxTorque, prm.maxTorque);
    *effortOut = effortFor(torque, speed);
    driving = true;
  } else if (!linkOk) {
    *effortOut = 0.0f;
    driving = true;
  }
  snprintf(body, sizeof(body), "X,%lu,%.5f,%.4f,%.2f,%.5f,%d", (unsigned long)tUs, est.angle,
           rate, speed, torque, (int)mode);
  sendLine(body);
  return driving;
}

bool setParam(const char *name, float v) {
  if (!strcmp(name, "engage")) prm.engage = v;
  else if (!strcmp(name, "disengage")) prm.disengage = v;
  else if (!strcmp(name, "catch")) prm.catchAngle = v;
  else if (!strcmp(name, "jump_timeout")) prm.jumpTimeout = v;
  else if (!strcmp(name, "max_torque")) prm.maxTorque = v;
  else if (!strcmp(name, "stall_torque") && v > 0.0f) prm.stallTorque = v;
  else if (!strcmp(name, "no_load_speed") && v > 0.0f) prm.noLoadSpeed = v;
  else if (!strcmp(name, "max_effort")) prm.maxEffort = constrain(v, 0.0f, 1.0f);
  else if (!strcmp(name, "angle_offset")) prm.angleOffset = v;
  else if (!strcmp(name, "tau") && v > 0.0f) prm.tau = v;
  else if (!strcmp(name, "imu_offset")) prm.imuOffset = v;
  else if (!strcmp(name, "cpr") && v >= 1.0f) prm.cpr = (int)lroundf(v);
  else if (!strcmp(name, "enc_sign")) prm.encSign = v < 0.0f ? -1 : 1;
  else return false;
  return true;
}

// "+x,-z,+y" -> axisIndex/axisSign. Returns false (and changes nothing) if malformed.
bool setAxes(const char *arg) {
  int8_t idx[3], sgn[3];
  const char *p = arg;
  for (int i = 0; i < 3; i++) {
    sgn[i] = 1;
    if (*p == '+' || *p == '-') sgn[i] = (*p++ == '-') ? -1 : 1;
    if (*p < 'x' || *p > 'z') return false;
    idx[i] = (int8_t)(*p++ - 'x');
    if (i < 2 && *p++ != ',') return false;
  }
  if (*p != '\0') return false;
  memcpy(axisIndex, idx, sizeof(idx));
  memcpy(axisSign, sgn, sizeof(sgn));
  return true;
}

// ---- Commands from the PC --------------------------------------------------------
void handleLine(char *line) {
  if (line[0] != '$') return;
  char *star = strrchr(line, '*');
  if (star == nullptr) return;
  *star = '\0';
  uint8_t sum = 0;
  for (char *p = line + 1; *p; p++) sum ^= (uint8_t)*p;
  if (sum != (uint8_t)strtoul(star + 1, nullptr, 16)) return;

  const char kind = line[1];
  char *arg = (line[2] == ',') ? line + 3 : line + strlen(line);  // "" without fields
  lastRxMs = millis();
  char text[80];
  switch (kind) {
    case 'T':
      effort = atof(arg);
      lastCmdMs = millis();
      break;
    case 'B': {
      const int ms = atoi(arg);
      startBrake(ms);
      catchArmed = true;  // on-board mode: catch the cube within `catch` after the jump
      catchUntilMs = millis() + (uint32_t)ms + (uint32_t)(prm.jumpTimeout * 1000.0f);
      break;
    }
    case 'E':
      motorEnabled = atoi(arg) == 1;
      if (!motorEnabled) effort = 0.0f;
      sendInfo(motorEnabled ? "motor enabled" : "motor disabled");
      break;
    case 'M':
      if (atoi(arg) == 1) {
        mode = MODE_CALIBRATING;
        resetEstimator();
        sendInfo("onboard balance on, calibrating");
      } else {
        mode = MODE_PASSTHROUGH;
        sendInfo("onboard balance off");
      }
      break;
    case 'K': {
      char *end = arg;
      float k[3];
      for (int i = 0; i < 3; i++) {
        k[i] = strtof(end, &end);
        if (i < 2 && *end++ != ',') return;
      }
      memcpy(lqrK, k, sizeof(k));
      snprintf(text, sizeof(text), "K %.6g,%.6g,%.6g", lqrK[0], lqrK[1], lqrK[2]);
      sendInfo(text);
      break;
    }
    case 'P': {
      char *comma = strchr(arg, ',');
      if (comma == nullptr) return;
      *comma = '\0';
      if (!setParam(arg, strtof(comma + 1, nullptr))) {
        snprintf(text, sizeof(text), "unknown parameter %s", arg);
        sendInfo(text);
      }
      break;
    }
    case 'A':
      if (setAxes(arg)) {
        resetEstimator();
      } else {
        sendInfo("bad axes");
      }
      break;
    default:
      break;
  }
}

void readCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
    } else if (c != '\r') {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = c;
      } else {
        lineLen = 0;  // overlong line: drop it
      }
    }
  }
}

// ---- MPU6050 ---------------------------------------------------------------------
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x01);  // wake up, clock from the X gyro PLL
  delay(50);
  mpuWrite(0x19, 0x00);  // sample rate divider: 1 kHz
  mpuWrite(0x1A, 0x03);  // digital low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x08);  // gyro +-500 deg/s
  mpuWrite(0x1C, 0x00);  // accel +-2 g
  return true;
}

bool mpuRead(float accel[3], float gyro[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  for (int i = 0; i < 3; i++) {
    accel[i] = raw[i] * ACCEL_SCALE;
    gyro[i] = raw[i + 4] * GYRO_SCALE;  // raw[3] is temperature
  }
  return true;
}

// ---- Main ------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);

  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_BRAKE, !BRAKE_ACTIVE);
  pwmSetup(PIN_PWM, 0, PWM_FREQ, PWM_BITS);
  setEffort(0.0f);
  if (PIN_SERVO >= 0) {
    pwmSetup(PIN_SERVO, 1, 50, 16);
    servoWriteUs(SERVO_RELEASED_US);
  }

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), onEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), onEncoder, CHANGE);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  while (!mpuInit()) {
    sendInfo("MPU6050 not found, check wiring");
    delay(1000);
  }
  sendInfo("cubli_esp32 ready");
  nextUs = micros();
}

void loop() {
  readCommands();

  const uint32_t now = micros();
  if ((int32_t)(now - nextUs) < 0) return;
  nextUs += PERIOD_US;
  if ((int32_t)(now - nextUs) > (int32_t)(4 * PERIOD_US)) nextUs = now + PERIOD_US;  // overrun

  if (braking && (int32_t)(millis() - brakeUntilMs) >= 0) endBrake();
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) effort = 0.0f;  // watchdog

  float accel[3], gyro[3];
  if (!mpuRead(accel, gyro)) {
    if (mode == MODE_BALANCE) mode = MODE_IDLE;  // no tilt estimate: stop balancing
    setEffort(effort);
    sendInfo("IMU read failed");
    return;
  }
  noInterrupts();
  const int32_t enc = encCount;
  interrupts();

  char body[128];
  snprintf(body, sizeof(body), "S,%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%ld",
           (unsigned long)now, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2],
           (long)enc);
  sendLine(body);

  float onboardEffort = 0.0f;
  if (mode != MODE_PASSTHROUGH && onboardStep(now, accel, gyro, enc, &onboardEffort)) {
    setEffort(onboardEffort);  // straight from this sample: no USB or PC in the loop
  } else {
    setEffort(effort);
  }
}
