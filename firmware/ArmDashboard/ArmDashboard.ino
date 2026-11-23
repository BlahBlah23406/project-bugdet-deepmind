// Mega differential arm and claw. USB lines follow PROTOCOL.md, version 1.
#include <AccelStepper.h>
#include <Wire.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

const byte EN_PIN = 43, CLAW_SWITCH_PIN = 32;
const byte limitPins[5] = {22, 24, 26, 28, 30};
const float CLAW_STEPS_PER_DEG = 4096.0f / 360.0f;
const float CLAW_SPEED = 600, CLAW_CLOSE_SPEED = 400, CLAW_HOME_SPEED = 300, CLAW_ACCEL = 3000;
const long HOME_OVERSHOOT = 20 * CLAW_STEPS_PER_DEG;
const long NEAR_OPEN = 30 * CLAW_STEPS_PER_DEG;
const long FAR_TARGET = 1000000000L;
const unsigned long MAGIC = 0x41524D31UL;
const uint16_t VERSION = 1;
const byte VALID_MARK = 0xA5;

AccelStepper j2(AccelStepper::DRIVER, 27, 29);
AccelStepper j3(AccelStepper::DRIVER, 31, 33);
AccelStepper claw(AccelStepper::HALF4WIRE, 11, 9, 10, 8);

struct Saved {
  unsigned long magic;
  uint16_t version;
  float gear, maxSteps, accel;
  float limits[4];
  long pos2, pos3, posC, closeLimit;
  byte speedPercent, limitsOn, inv2, inv3, trusted, enabled;
};
Saved cfg, snapshot;
bool driversOn = true, trusted = false;
bool fitted[5], switchFault = false;
bool useSwitch = false, hasEncoder = false, clawUncertain = false;
bool clawMoving = false, clawHoming = false, clawJogging = false;
int clawVelocity = 0, pendingClawVelocity = 0;
bool clawJogPending = false;
int encSign = 1, lastRaw = 0;
bool encSignKnown = false;
long encCounts = 0, encOrigin = 0, chkCmd = 0, chkEnc = 0;
unsigned long lastEncRead = 0, lastStallCheck = 0;
unsigned long lastFeed = 0, lastStatus = 0, idleSince = 0;
bool watchdogTripped = false, jogging = false, pending = false;
float wanted2 = 0, wanted3 = 0;
bool pendingGoto = false, gotoActive = false;
long goal2 = 0, goal3 = 0;
bool dirty = false, saveActive = false, savedValid = false;
bool invalidatePending = false, invalidWriting = false;
uint16_t saveIndex = 0;

// Build JSON in RAM, then drain only as much as the UART can accept.
class JsonLine : public Print {
public:
  char data[360];
  uint16_t used;
  JsonLine() : used(0) {}
  void number(float value, byte digits) {
    float magnitude = fabs(value);
    if (!isfinite(value)) { print(F("null")); return; }
    if (magnitude >= 1.0e8f || (magnitude > 0 && magnitude < 0.0001f)) {
      char text[24];
      dtostre(value, text, digits, 0);
      print(text);
    } else print(value, digits);
  }
  size_t write(uint8_t c) override {
    if (used >= sizeof(data) - 1) return 0;
    data[used++] = c;
    return 1;
  }
};
char tx[1536];
uint16_t txHead = 0, txTail = 0;
char input[160];
byte inputLen = 0;
bool inputOverflow = false;
char deferredInput[160];
bool deferredCommand = false;

void queueLine(JsonLine &line) {
  uint16_t freeBytes = (txTail + sizeof(tx) - txHead - 1) % sizeof(tx);
  if (freeBytes < line.used + 1) return;
  for (uint16_t i = 0; i < line.used; ++i) {
    tx[txHead] = line.data[i];
    txHead = (txHead + 1) % sizeof(tx);
  }
  tx[txHead] = '\n';
  txHead = (txHead + 1) % sizeof(tx);
}

void pumpOutput() {
  int room = Serial.availableForWrite();
  // A short burst leaves time for the step generators.
  if (room > 16) room = 16;
  while (room-- > 0 && txTail != txHead) {
    Serial.write(tx[txTail]);
    txTail = (txTail + 1) % sizeof(tx);
  }
}

void message(const __FlashStringHelper *s, bool error = false) {
  JsonLine out;
  out.print(error ? F("{\"t\":\"err\",\"s\":\"") : F("{\"t\":\"msg\",\"s\":\""));
  out.print(s);
  out.print(F("\"}"));
  queueLine(out);
}

float stepsPerDegree() { return 1600.0f * cfg.gear / 360.0f; }
float pitchSteps() { return j2.currentPosition() * 0.5f + j3.currentPosition() * 0.5f; }
float rollSteps() { return j2.currentPosition() * 0.5f - j3.currentPosition() * 0.5f; }
bool armMoving() { return j2.isRunning() || j3.isRunning(); }
bool moving() { return armMoving() || clawMoving || clawJogPending || pending; }

void printConfig() {
  JsonLine out;
  out.print(F("{\"t\":\"cfg\",\"ver\":1,\"gear\":")); out.number(cfg.gear, 4);
  out.print(F(",\"max\":")); out.number(cfg.maxSteps, 1);
  out.print(F(",\"acc\":")); out.number(cfg.accel, 1);
  out.print(F(",\"lim\":"));
  if (cfg.limitsOn) {
    out.print('[');
    for (byte i = 0; i < 4; ++i) { if (i) out.print(','); out.number(cfg.limits[i], 2); }
    out.print(']');
  } else out.print(F("null"));
  out.print(F(",\"inv\":[")); out.print(cfg.inv2); out.print(','); out.print(cfg.inv3);
  out.print(F("],\"cmax\":")); out.number(cfg.closeLimit / CLAW_STEPS_PER_DEG, 1);
  out.print('}'); queueLine(out);
}

void printStatus() {
  JsonLine out;
  out.print(F("{\"t\":\"st\",\"j2\":")); out.print(j2.currentPosition());
  out.print(F(",\"j3\":")); out.print(j3.currentPosition());
  out.print(F(",\"p\":")); out.number(pitchSteps() / stepsPerDegree(), 1);
  out.print(F(",\"r\":")); out.number(rollSteps() / stepsPerDegree(), 1);
  out.print(F(",\"c\":")); out.number(max(0L, claw.currentPosition()) / CLAW_STEPS_PER_DEG, 1);
  out.print(F(",\"mv\":")); out.print(moving() ? 1 : 0);
  out.print(F(",\"en\":")); out.print(driversOn ? 1 : 0);
  out.print(F(",\"tr\":")); out.print(trusted ? 1 : 0);
  out.print(F(",\"spd\":")); out.print(cfg.speedPercent);
  out.print(F(",\"wd\":")); out.print(watchdogTripped ? 1 : 0);
  out.print(F(",\"lim\":\""));
  for (byte i = 0; i < 5; ++i) out.print(!fitted[i] ? '-' : (digitalRead(limitPins[i]) == HIGH ? '1' : '0'));
  out.print(F("\",\"sw\":")); out.print(useSwitch ? (digitalRead(CLAW_SWITCH_PIN) == LOW ? 1 : 0) : -1);
  out.print(F(",\"enc\":")); out.print(hasEncoder ? 1 : 0);
  out.print(F(",\"cu\":")); out.print(clawUncertain ? 1 : 0);
  out.print('}'); queueLine(out);
}

// Invalidate before any step. A reset during a move must not trust an old count.
void changed() {
  dirty = true;
  saveActive = false;
  idleSince = millis();
  if (savedValid) invalidatePending = true;
}

bool prepareSteps() {
  // Finish invalidating the saved count before the first new step.
  if (invalidatePending) {
    if (!eeprom_is_ready()) return false;
    byte invalid = 0;
    EEPROM.put(0, invalid);
    savedValid = false; invalidatePending = false; invalidWriting = true;
  }
  if (invalidWriting) {
    if (!eeprom_is_ready()) return false;
    invalidWriting = false;
  }
  return true;
}

void defaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = MAGIC; cfg.version = VERSION;
  cfg.gear = 1; cfg.maxSteps = 1200; cfg.accel = 1500;
  cfg.speedPercent = 50;
  cfg.closeLimit = 180 * CLAW_STEPS_PER_DEG;
}

bool goodConfig() {
  if (cfg.magic != MAGIC || cfg.version != VERSION) return false;
  if (!isfinite(cfg.gear) || cfg.gear <= 0 || !isfinite(1600.0f * cfg.gear / 360.0f) || 1600.0f * cfg.gear / 360.0f <= 0) return false;
  if (!isfinite(cfg.maxSteps) || cfg.maxSteps <= 0) return false;
  if (!isfinite(cfg.accel) || cfg.accel <= 0) return false;
  if (cfg.speedPercent < 10 || cfg.speedPercent > 100 || cfg.closeLimit < 0 || cfg.closeLimit > FAR_TARGET) return false;
  for (byte i = 0; i < 4; ++i) if (!isfinite(cfg.limits[i])) return false;
  return cfg.limits[0] <= cfg.limits[1] && cfg.limits[2] <= cfg.limits[3];
}

void persistLoop() {
  if (moving()) { idleSince = millis(); return; }
  if (!dirty || invalidatePending || invalidWriting || millis() - idleSince < 1000 || !eeprom_is_ready()) return;
  if (!saveActive) {
    snapshot = cfg;
    snapshot.pos2 = j2.currentPosition(); snapshot.pos3 = j3.currentPosition();
    snapshot.posC = claw.currentPosition();
    snapshot.trusted = trusted; snapshot.enabled = driversOn;
    saveIndex = 0; saveActive = true;
  }
  // EEPROM.put updates only changed bytes. One byte per pass avoids a long pause.
  if (saveIndex < sizeof(snapshot)) {
    byte value = reinterpret_cast<const byte *>(&snapshot)[saveIndex];
    EEPROM.put(1 + saveIndex, value);
    ++saveIndex;
  } else {
    byte valid = VALID_MARK;
    EEPROM.put(0, valid);
    savedValid = true; saveActive = false; dirty = false;
  }
}

void stopClaw() {
  if (claw.currentPosition() < 0) {
    changed(); claw.setCurrentPosition(0); encCounts = encOrigin = 0; clawUncertain = false;
  } else claw.setCurrentPosition(claw.currentPosition());
  claw.disableOutputs(); claw.setMaxSpeed(CLAW_SPEED);
  clawMoving = false; clawHoming = false; clawJogging = false;
  clawVelocity = 0; clawJogPending = false;
}

void stopArm(bool quick) {
  pending = false;
  float v2 = fabs(j2.speed()), v3 = fabs(j3.speed());
  float biggest = max(v2, v3);
  if (biggest > 0) {
    float a = cfg.accel * (quick ? 3 : 1);
    if (v2 > 0) j2.setAcceleration(a * v2 / biggest);
    if (v3 > 0) j3.setAcceleration(a * v3 / biggest);
  }
  // stop() does nothing at zero speed, so cancel an unstarted target too.
  if (v2 > 0) j2.stop(); else j2.moveTo(j2.currentPosition());
  if (v3 > 0) j3.stop(); else j3.moveTo(j3.currentPosition());
}

void rampClawStop() {
  clawJogging = false; clawVelocity = 0; clawJogPending = false;
  if (claw.currentPosition() < 0) { stopClaw(); return; }
  if (claw.speed() != 0) {
    claw.stop();
    // A stop should still respect the normal claw travel.
    if (!clawHoming) claw.moveTo(constrain(claw.targetPosition(), 0L, cfg.closeLimit));
    clawHoming = false;
  } else stopClaw();
}

void stopAll(bool quick) {
  jogging = false; gotoActive = false; wanted2 = wanted3 = 0;
  stopArm(quick);
  if (quick) stopClaw(); else rampClawStop();
}

bool switchBlocked() {
  for (byte i = 0; i < 5; ++i) if (fitted[i] && digitalRead(limitPins[i]) == HIGH) return true;
  return false;
}

bool insideLimits(float p, float r) {
  return !cfg.limitsOn || (p >= cfg.limits[0] && p <= cfg.limits[1] && r >= cfg.limits[2] && r <= cfg.limits[3]);
}

// Choose a target on the jog ray, shortened at the first joint boundary.
bool jogTarget(float v2, float v3) {
  float biggest = max(fabs(v2), fabs(v3));
  if (biggest < 0.001f) return false;
  float p = pitchSteps(), r = rollSteps();
  float vp = (v2 + v3) * 0.5f, vr = (v2 - v3) * 0.5f;
  float seconds = FAR_TARGET / biggest;
  if (cfg.limitsOn) {
    float scale = stepsPerDegree();
    if (vp > 0) seconds = min(seconds, (cfg.limits[1] * scale - p) / vp);
    if (vp < 0) seconds = min(seconds, (cfg.limits[0] * scale - p) / vp);
    if (vr > 0) seconds = min(seconds, (cfg.limits[3] * scale - r) / vr);
    if (vr < 0) seconds = min(seconds, (cfg.limits[2] * scale - r) / vr);
    // Leave two motor steps for rounding and the run() step interval.
    seconds -= 2.0f / biggest;
  }
  if (seconds <= 0) return false;
  double t2 = j2.currentPosition() + v2 * seconds;
  double t3 = j3.currentPosition() + v3 * seconds;
  if (fabs(t2) > 2000000000.0 || fabs(t3) > 2000000000.0) return false;
  j2.setAcceleration(cfg.accel * max(fabs(v2) / biggest, 0.0001f));
  j3.setAcceleration(cfg.accel * max(fabs(v3) / biggest, 0.0001f));
  j2.setMaxSpeed(max(fabs(v2), 0.001f));
  j3.setMaxSpeed(max(fabs(v3), 0.001f));
  j2.moveTo(lround(t2)); j3.moveTo(lround(t3));
  return true;
}

void startGoto() {
  float d2 = fabs((float)goal2 - j2.currentPosition());
  float d3 = fabs((float)goal3 - j3.currentPosition());
  float biggest = max(d2, d3);
  if (biggest < 1) return;
  float speed = cfg.maxSteps * cfg.speedPercent / 100.0f;
  j2.setMaxSpeed(max(speed * d2 / biggest, 0.001f));
  j3.setMaxSpeed(max(speed * d3 / biggest, 0.001f));
  j2.setAcceleration(max(cfg.accel * d2 / biggest, 0.001f));
  j3.setAcceleration(max(cfg.accel * d3 / biggest, 0.001f));
  j2.moveTo(goal2); j3.moveTo(goal3);
}

void requestJog(float v2, float v3) {
  if (jogging && !pendingGoto && wanted2 == v2 && wanted3 == v3) return;
  jogging = (v2 != 0 || v3 != 0); pendingGoto = false; gotoActive = false;
  wanted2 = v2; wanted3 = v3;
  stopArm(false);
  pending = (v2 != 0 || v3 != 0);
  if (pending) changed();
}

void armLoop() {
  if (gotoActive && !pending && !armMoving()) gotoActive = false;
  if (armMoving() && switchBlocked()) {
    if (!switchFault) {
      for (byte i = 0; i < 5; ++i) {
        if (fitted[i] && digitalRead(limitPins[i]) == HIGH) {
          JsonLine out;
          out.print(F("{\"t\":\"err\",\"s\":\"Limit D")); out.print(limitPins[i]);
          out.print(F(" triggered\"}")); queueLine(out);
        }
      }
      stopAll(true); trusted = false; changed(); switchFault = true;
    }
  } else if (!switchBlocked()) switchFault = false;
  if (pending && !armMoving()) {
    pending = false;
    if (driversOn && !switchBlocked()) {
      if (pendingGoto) startGoto(); else jogTarget(wanted2, wanted3);
    }
    pendingGoto = false;
  }
  if (!cfg.limitsOn || !armMoving()) return;
  float v2 = j2.speed(), v3 = j3.speed();
  float vp = (v2 + v3) * 0.5f, vr = (v2 - v3) * 0.5f;
  float rate = 1.0e10f;
  if (fabs(v2) > 0.001f) rate = min(rate, j2.acceleration() / fabs(v2));
  if (fabs(v3) > 0.001f) rate = min(rate, j3.acceleration() / fabs(v3));
  float scale = stepsPerDegree();
  float positions[2] = {pitchSteps(), rollSteps()};
  float velocities[2] = {vp, vr};
  for (byte i = 0; i < 2; ++i) {
    float v = velocities[i];
    if (fabs(v) < 0.001f) continue;
    float a = fabs(v) * rate;
    float stopping = v * v / (2.0f * a);
    float room = v > 0 ? cfg.limits[i * 2 + 1] * scale - positions[i]
                       : positions[i] - cfg.limits[i * 2] * scale;
    // Targets already stop at the limit. This also guards limits changed in motion.
    if (room <= stopping + 2) {
      if (jogging || pending) { jogging = false; wanted2 = wanted3 = 0; stopArm(false); }
    }
  }
}

// AS5600 support matches the working claw sketch.
int as5600Read(byte reg, byte count) {
  Wire.beginTransmission(0x36); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((byte)0x36, count) != count) return -1;
  int value = 0;
  for (byte i = 0; i < count; ++i) value = (value << 8) | Wire.read();
  return value;
}

long encSteps() { return encCounts; }
void updateEncoder() {
  int raw = as5600Read(0x0C, 2);
  if (raw < 0) return;
  raw &= 0x0FFF;
  int delta = raw - lastRaw;
  if (delta > 2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  lastRaw = raw; encCounts += (long)delta * encSign;
}
void startStallWindow() {
  lastStallCheck = millis(); chkCmd = claw.currentPosition(); chkEnc = encSteps();
}
bool stalled() {
  if (millis() - lastStallCheck < 100) return false;
  long cmd = claw.currentPosition() - chkCmd, act = encSteps() - chkEnc;
  if (!encSignKnown && labs(cmd) >= 40 && labs(act) > labs(cmd) / 2) {
    if ((act > 0) != (cmd > 0)) {
      encSign = -encSign;
      encCounts = 2 * encOrigin - encCounts;
      message(F("Claw encoder direction learned"));
    }
    encSignKnown = true;
  }
  startStallWindow();
  return labs(cmd) >= 20 && labs(act) < labs(cmd) / 4;
}
void zeroClaw(const __FlashStringHelper *why) {
  changed(); stopClaw(); claw.setCurrentPosition(0); encCounts = encOrigin = 0;
  clawUncertain = false;
  message(why);
}
void moveClaw(long target, float speed) {
  if (!clawMoving) startStallWindow();
  if (!hasEncoder && target > claw.currentPosition()) clawUncertain = true;
  changed(); claw.enableOutputs(); claw.setMaxSpeed(speed); claw.moveTo(target);
  clawMoving = claw.isRunning();
}
void homeClaw() {
  changed(); stopClaw();
  if (useSwitch && digitalRead(CLAW_SWITCH_PIN) == LOW) { zeroClaw(F("Claw open: switch")); return; }
  long travel = (useSwitch || hasEncoder) ? cfg.closeLimit + 30 * CLAW_STEPS_PER_DEG
                                         : max(claw.currentPosition(), 0L) + HOME_OVERSHOOT;
  moveClaw(claw.currentPosition() - travel, CLAW_HOME_SPEED);
  clawHoming = true;
  message(F("Claw homing"));
}
void jogClaw(int velocity) {
  if (clawHoming) return;
  if (clawJogPending && velocity == pendingClawVelocity) return;
  if (!clawJogPending && clawJogging && velocity == clawVelocity) return;
  if (!velocity) { if (clawMoving || clawJogPending) rampClawStop(); return; }
  if (clawMoving) {
    rampClawStop();
    pendingClawVelocity = velocity; clawJogPending = true; clawJogging = true;
    return;
  }
  clawJogPending = false;
  clawVelocity = velocity;
  float base = velocity > 0 ? CLAW_CLOSE_SPEED : (claw.currentPosition() <= 0 ? CLAW_HOME_SPEED : CLAW_SPEED);
  moveClaw(velocity > 0 ? cfg.closeLimit : -HOME_OVERSHOOT, base * abs(velocity) / 100.0f);
  clawJogging = true;
}
void clawLoop() {
  if (hasEncoder && millis() - lastEncRead >= 20) { lastEncRead = millis(); updateEncoder(); }
  if (!clawMoving) {
    if (clawJogPending) { int velocity = pendingClawVelocity; clawJogPending = false; jogClaw(velocity); }
    return;
  }
  bool opening = claw.distanceToGo() < 0 || claw.speed() < 0;
  // Slow down for the open-stop overshoot once past zero.
  if (opening && claw.targetPosition() < 0 && claw.currentPosition() <= 0 && claw.maxSpeed() > CLAW_HOME_SPEED) {
    claw.setMaxSpeed(CLAW_HOME_SPEED);
  }
  if (useSwitch && opening && digitalRead(CLAW_SWITCH_PIN) == LOW) { zeroClaw(F("Claw open: switch")); return; }
  if (hasEncoder && stalled()) {
    if (opening && (clawHoming || claw.currentPosition() < NEAR_OPEN)) zeroClaw(F("Claw open: encoder stop"));
    else {
      changed(); stopClaw(); claw.setCurrentPosition(encSteps());
      message(opening ? F("Claw blocked while opening") : F("Claw stalled: gripping"), true);
    }
    return;
  }
  if (!claw.isRunning()) {
    if (clawHoming) {
      zeroClaw((useSwitch || hasEncoder) ? F("Claw home travel ended; no stop found")
                                        : F("Claw open: hard stop"));
      return;
    }
    if (claw.currentPosition() < 0 || claw.targetPosition() < 0) {
      zeroClaw(F("Claw open: hard stop"));
      return;
    }
    bool restart = clawJogPending;
    int velocity = pendingClawVelocity;
    stopClaw();
    if (hasEncoder && labs(encSteps() - claw.currentPosition()) > 8) {
      changed(); claw.setCurrentPosition(encSteps());
    }
    if (restart) jogClaw(velocity);
  }
}

bool number(const char *token, float &value) {
  if (!token || !*token) return false;
  char *end;
  value = strtod(token, &end);
  return *end == 0 && isfinite(value);
}
bool integer(const char *token, int &value, int low, int high) {
  if (!token || !*token) return false;
  char *end;
  long n = strtol(token, &end, 10);
  if (*end || n < low || n > high) return false;
  value = (int)n; return true;
}
void configChanged() { changed(); printConfig(); }
bool requireIdle() {
  if (armMoving() || pending) { message(F("Arm is still stopping"), true); return false; }
  return true;
}

void command() {
  char original[sizeof(input)];
  memcpy(original, input, sizeof(input));
  char *tokens[6]; byte count = 0;
  char *save;
  char *token = strtok_r(input, " ", &save);
  while (token && count < 6) { tokens[count++] = token; token = strtok_r(NULL, " ", &save); }
  if (!count || token) { message(F("Invalid command"), true); return; }
  const char *cmd = tokens[0];
  bool needsIdle = !strcmp(cmd, "Z") || !strcmp(cmd, "CFG") || !strcmp(cmd, "INV") ||
                   !strcmp(cmd, "LIM") || !strcmp(cmd, "LIMHERE");
  if (needsIdle && (armMoving() || pending)) {
    memcpy(deferredInput, original, sizeof(input));
    deferredCommand = true;
    jogging = false; wanted2 = wanted3 = 0;
    stopArm(false);
    return;
  }
  float a = 0, b = 0, c = 0, d = 0;
  int x = 0, y = 0, z = 0;
  if ((!strcmp(cmd, "V") && count == 4) || (!strcmp(cmd, "M") && count == 3)) {
    bool raw = cmd[0] == 'M';
    if (!integer(tokens[1], x, -100, 100) || !integer(tokens[2], y, -100, 100) ||
        (!raw && !integer(tokens[3], z, -100, 100))) { message(F("Invalid jog velocity"), true); return; }
    lastFeed = millis();
    if (!raw) watchdogTripped = false;
    float speed = cfg.maxSteps * cfg.speedPercent / 100.0f;
    float v2 = raw ? x : x + y, v3 = raw ? y : x - y;
    float divisor = max(100.0f, max(fabs(v2), fabs(v3)));
    if (deferredCommand) {
      // Apply the queued setting at rest; the next held-key update starts motion.
    } else if ((v2 || v3) && (!driversOn || switchBlocked())) {
      message(F("Arm drivers off or limit switch active"), true);
      requestJog(0, 0);
    } else requestJog(v2 * speed / divisor, v3 * speed / divisor);
    if (!raw) jogClaw(z);
  } else if (!strcmp(cmd, "P") && count == 1) {
    lastFeed = millis();
  } else if (!strcmp(cmd, "S") && count == 1) {
    deferredCommand = false; stopAll(true);
  } else if (!strcmp(cmd, "G") && count == 3 && number(tokens[1], a) && number(tokens[2], b)) {
    if (deferredCommand) { message(F("Configuration pending; arm is stopping"), true); return; }
    if (!trusted) { message(F("Position not trusted; use Z"), true); return; }
    if (!driversOn || switchBlocked()) { message(F("Arm drivers off or limit switch active"), true); return; }
    if (!insideLimits(a, b)) { message(F("Target outside soft limits"), true); return; }
    float t2 = (a + b) * stepsPerDegree(), t3 = (a - b) * stepsPerDegree();
    if (fabs(t2) > FAR_TARGET || fabs(t3) > FAR_TARGET) { message(F("Target too large"), true); return; }
    // Quantize within the joint box, including limits not on a full step.
    goal2 = lround(t2); goal3 = lround(t3);
    bool found = false;
    float best = 1.0e10f;
    for (byte i = 0; i < 4; ++i) {
      long q2 = (i & 1) ? (long)ceil(t2) : (long)floor(t2);
      long q3 = (i & 2) ? (long)ceil(t3) : (long)floor(t3);
      if (!insideLimits((q2 * 0.5f + q3 * 0.5f) / stepsPerDegree(),
                        (q2 * 0.5f - q3 * 0.5f) / stepsPerDegree())) continue;
      float error = (q2 - t2) * (q2 - t2) + (q3 - t3) * (q3 - t3);
      if (error < best) { goal2 = q2; goal3 = q3; best = error; found = true; }
    }
    if (!found) { message(F("No motor step inside target limits"), true); return; }
    stopArm(false); jogging = false; wanted2 = wanted3 = 0;
    pendingGoto = true; gotoActive = true; pending = true; changed();
  } else if (!strcmp(cmd, "GC") && count == 2 && number(tokens[1], a)) {
    if (a < 0 || a * CLAW_STEPS_PER_DEG > cfg.closeLimit) { message(F("Claw target outside limits"), true); return; }
    stopClaw();
    long target = a <= 0.5f ? -HOME_OVERSHOOT : lround(a * CLAW_STEPS_PER_DEG);
    float speed = target > claw.currentPosition() ? CLAW_CLOSE_SPEED
                : (target < 0 && claw.currentPosition() <= 0 ? CLAW_HOME_SPEED : CLAW_SPEED);
    moveClaw(target, speed);
  } else if (!strcmp(cmd, "HC") && count == 1) {
    homeClaw();
  } else if (!strcmp(cmd, "ZC") && count == 1) {
    zeroClaw(F("Claw zero marked"));
  } else if (!strcmp(cmd, "KC") && count == 1) {
    stopClaw();
    if (claw.currentPosition() < 0) { message(F("Claw close limit must be nonnegative"), true); return; }
    cfg.closeLimit = claw.currentPosition(); configChanged();
  } else if (!strcmp(cmd, "Z") && count == 1) {
    if (!requireIdle()) return;
    changed(); j2.setCurrentPosition(0); j3.setCurrentPosition(0); trusted = true;
    message(F("Arm zero marked"));
  } else if (!strcmp(cmd, "E") && count == 2 && integer(tokens[1], x, 0, 1)) {
    changed(); driversOn = x;
    if (!driversOn) {
      stopArm(false); jogging = false; gotoActive = false; wanted2 = wanted3 = 0;
      j2.setCurrentPosition(j2.currentPosition()); j3.setCurrentPosition(j3.currentPosition());
      trusted = false;
    }
    digitalWrite(EN_PIN, driversOn ? LOW : HIGH);
  } else if (!strcmp(cmd, "SPD") && count == 2 && integer(tokens[1], x, 10, 100)) {
    float ratio = (float)x / cfg.speedPercent;
    cfg.speedPercent = x;
    if (jogging) requestJog(wanted2 * ratio, wanted3 * ratio);
    else if (gotoActive) {
      stopArm(false); pendingGoto = true; pending = true;
    }
    configChanged();
  } else if (!strcmp(cmd, "CFG") && count == 4 && number(tokens[1], a) && number(tokens[2], b) && number(tokens[3], c)) {
    if (a <= 0 || b <= 0 || c <= 0 || !isfinite(1600.0f * a / 360.0f) || 1600.0f * a / 360.0f <= 0) {
      message(F("Configuration out of range"), true); return;
    }
    if (!requireIdle()) return;
    cfg.gear = a; cfg.maxSteps = b; cfg.accel = c; configChanged();
  } else if (!strcmp(cmd, "LIM") && count == 2 && !strcmp(tokens[1], "OFF")) {
    cfg.limitsOn = false; configChanged();
  } else if (!strcmp(cmd, "LIM") && count == 5 && number(tokens[1], a) && number(tokens[2], b) && number(tokens[3], c) && number(tokens[4], d)) {
    if (a > b || c > d) {
      message(F("Invalid soft limits"), true); return;
    }
    if (!requireIdle()) return;
    cfg.limits[0] = a; cfg.limits[1] = b; cfg.limits[2] = c; cfg.limits[3] = d;
    cfg.limitsOn = true; configChanged();
  } else if (!strcmp(cmd, "LIMHERE") && count == 2) {
    if (!requireIdle()) return;
    const char *names[] = {"PMIN", "PMAX", "RMIN", "RMAX"};
    byte i;
    for (i = 0; i < 4; ++i) if (!strcmp(tokens[1], names[i])) break;
    if (i == 4) { message(F("Unknown limit name"), true); return; }
    float value = (i < 2 ? pitchSteps() : rollSteps()) / stepsPerDegree();
    if (!cfg.limitsOn) {
      cfg.limits[0] = cfg.limits[2] = -1.0e30f;
      cfg.limits[1] = cfg.limits[3] = 1.0e30f;
    }
    if ((i % 2 == 0 && value > cfg.limits[i + 1]) || (i % 2 && value < cfg.limits[i - 1])) {
      message(F("Limit minimum exceeds maximum"), true); return;
    }
    cfg.limits[i] = value; cfg.limitsOn = true; configChanged();
  } else if (!strcmp(cmd, "INV") && count == 3 && integer(tokens[1], x, 0, 1) && integer(tokens[2], y, 0, 1)) {
    if (!requireIdle()) return;
    cfg.inv2 = x; cfg.inv3 = y;
    j2.setPinsInverted(x, false, false); j3.setPinsInverted(y, false, false); configChanged();
  } else if (!strcmp(cmd, "?") && count == 1) {
    printConfig(); printStatus();
  } else message(F("Unknown command or invalid arguments"), true);
}

void readInput() {
  // One byte per loop bounds parsing work while a host is sending continuously.
  if (!Serial.available()) return;
  int ch = Serial.read();
  if (ch == '\n') {
    if (inputOverflow) message(F("Input line too long"), true);
    else { input[inputLen] = 0; command(); }
    inputLen = 0; inputOverflow = false;
  } else if (ch >= 32 && ch <= 126 && !inputOverflow) {
    if (inputLen < sizeof(input) - 1) input[inputLen++] = ch;
    else inputOverflow = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT); digitalWrite(EN_PIN, HIGH);
  EEPROM.get(1, cfg);
  byte valid; EEPROM.get(0, valid);
  if (!goodConfig()) { defaults(); dirty = true; }
  else savedValid = valid == VALID_MARK;
  trusted = savedValid && cfg.trusted && cfg.enabled;
  j2.setCurrentPosition(cfg.pos2); j3.setCurrentPosition(cfg.pos3);
  claw.setCurrentPosition(cfg.posC); encCounts = encOrigin = cfg.posC;
  j2.setPinsInverted(cfg.inv2, false, false); j3.setPinsInverted(cfg.inv3, false, false);
  j2.setAcceleration(cfg.accel); j3.setAcceleration(cfg.accel);
  j2.setMaxSpeed(cfg.maxSteps * cfg.speedPercent / 100.0f);
  j3.setMaxSpeed(cfg.maxSteps * cfg.speedPercent / 100.0f);
  claw.setAcceleration(CLAW_ACCEL); stopClaw();
  for (byte i = 0; i < 5; ++i) pinMode(limitPins[i], INPUT_PULLUP);
  pinMode(CLAW_SWITCH_PIN, INPUT_PULLUP);
  Wire.begin(); Wire.setClock(400000); Wire.setWireTimeout(3000, true);
  // The I2C probe also gives the input pullups time to settle, without delay().
  int status = as5600Read(0x0B, 1);
  if (status >= 0 && (status & 0x20)) {
    int raw = as5600Read(0x0C, 2);
    if (raw >= 0) { hasEncoder = true; lastRaw = raw & 0x0FFF; }
  }
  useSwitch = digitalRead(CLAW_SWITCH_PIN) == LOW;
  clawUncertain = !hasEncoder && claw.currentPosition() > 0;
  for (byte i = 0; i < 5; ++i) fitted[i] = digitalRead(limitPins[i]) == LOW;
  driversOn = true; digitalWrite(EN_PIN, LOW);
  idleSince = lastFeed = lastStatus = millis();
  JsonLine hello; hello.print(F("{\"t\":\"hello\",\"fw\":\"ArmDashboard\",\"ver\":1}")); queueLine(hello);
  printConfig(); printStatus();
}

void loop() {
  bool stepsReady = prepareSteps();
  if (stepsReady) {
    if (driversOn) { j2.run(); j3.run(); }
    claw.run();
  }
  if (deferredCommand && !armMoving() && !pending && inputLen == 0) {
    memcpy(input, deferredInput, sizeof(input));
    deferredCommand = false; command();
  }
  readInput();
  if ((jogging || clawJogging) && millis() - lastFeed >= 400) {
    watchdogTripped = true; stopAll(false);
  }
  armLoop(); clawLoop();
  if (millis() - lastStatus >= 100) { lastStatus = millis(); printStatus(); }
  persistLoop(); pumpOutput();
}
