// Mega differential arm and claw. USB lines follow PROTOCOL.md, version 1.
#include <AccelStepper.h>
#include <Servo.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const byte EN_PIN = 43, CLAW_SERVO_PIN = 11;  // Servo signal on J27 IN4
const byte limitPins[5] = {22, 24, 26, 28, 30};
const int CLAW_MIN_US = 500, CLAW_MAX_US = 2500;
const float CLAW_RANGE = 300.0f, CLAW_SPEED = 120.0f;
const unsigned long CLAW_INTERVAL_MS = 10;
const long FAR_TARGET = 1000000000L;
const unsigned long MAGIC = 0x41524D31UL;
const uint16_t VERSION = 2;
const byte VALID_MARK = 0xA5;

AccelStepper j2(AccelStepper::DRIVER, 27, 29);
AccelStepper j3(AccelStepper::DRIVER, 31, 33);
Servo clawServo;

struct Saved {
  unsigned long magic;
  uint16_t version;
  float gear, maxSteps, accel;
  float limits[4];
  long pos2, pos3;
  float clawPos, open, closeLimit;
  int8_t dir;
  byte speedPercent, limitsOn, inv2, inv3, trusted, enabled;
};
Saved cfg, snapshot;
bool driversOn = true, trusted = false;
bool fitted[5], switchFault = false;
float clawPos = 0, clawTarget = 0, clawRate = 0;
bool clawJogging = false;
int clawPulse = 0;
unsigned long lastClawUpdate = 0;
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
bool moving() { return armMoving() || clawPos != clawTarget || pending; }

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
  out.print(F("],\"cmax\":")); out.number(cfg.closeLimit, 1);
  out.print(F(",\"copen\":")); out.number(cfg.open, 1);
  out.print(F(",\"cdir\":")); out.print((int)cfg.dir);
  out.print('}'); queueLine(out);
}

void printStatus() {
  JsonLine out;
  out.print(F("{\"t\":\"st\",\"j2\":")); out.print(j2.currentPosition());
  out.print(F(",\"j3\":")); out.print(j3.currentPosition());
  out.print(F(",\"p\":")); out.number(pitchSteps() / stepsPerDegree(), 1);
  out.print(F(",\"r\":")); out.number(rollSteps() / stepsPerDegree(), 1);
  out.print(F(",\"c\":")); out.print(clawPos, 1);
  out.print(F(",\"mv\":")); out.print(moving() ? 1 : 0);
  out.print(F(",\"en\":")); out.print(driversOn ? 1 : 0);
  out.print(F(",\"tr\":")); out.print(trusted ? 1 : 0);
  out.print(F(",\"spd\":")); out.print(cfg.speedPercent);
  out.print(F(",\"wd\":")); out.print(watchdogTripped ? 1 : 0);
  out.print(F(",\"lim\":\""));
  for (byte i = 0; i < 5; ++i) out.print(!fitted[i] ? '-' : (digitalRead(limitPins[i]) == HIGH ? '1' : '0'));
  out.print(F("\",\"sw\":-1,\"enc\":0,\"cu\":0}")); queueLine(out);
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
  cfg.open = 150.0f; cfg.dir = 1; cfg.closeLimit = 90.0f;
}

bool goodConfig() {
  if (cfg.magic != MAGIC || cfg.version != VERSION) return false;
  if (!isfinite(cfg.gear) || cfg.gear <= 0 || !isfinite(1600.0f * cfg.gear / 360.0f) || 1600.0f * cfg.gear / 360.0f <= 0) return false;
  if (!isfinite(cfg.maxSteps) || cfg.maxSteps <= 0) return false;
  if (!isfinite(cfg.accel) || cfg.accel <= 0) return false;
  if (cfg.speedPercent < 10 || cfg.speedPercent > 100) return false;
  if (!isfinite(cfg.open) || cfg.open < 0 || cfg.open > CLAW_RANGE || (cfg.dir != 1 && cfg.dir != -1)) return false;
  if (!isfinite(cfg.closeLimit) || cfg.closeLimit <= 0 || cfg.closeLimit > CLAW_RANGE) return false;
  if (!isfinite(cfg.clawPos) || cfg.clawPos < 0 || cfg.clawPos > cfg.closeLimit) return false;
  for (byte i = 0; i < 4; ++i) if (!isfinite(cfg.limits[i])) return false;
  return cfg.limits[0] <= cfg.limits[1] && cfg.limits[2] <= cfg.limits[3];
}

void persistLoop() {
  if (moving()) { idleSince = millis(); return; }
  if (!dirty || invalidatePending || invalidWriting || millis() - idleSince < 1000 || !eeprom_is_ready()) return;
  if (!saveActive) {
    snapshot = cfg;
    snapshot.pos2 = j2.currentPosition(); snapshot.pos3 = j3.currentPosition();
    snapshot.clawPos = clawPos;
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
  clawTarget = clawPos;
  clawJogging = false;
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

void stopAll(bool quick) {
  jogging = false; gotoActive = false; wanted2 = wanted3 = 0;
  stopArm(quick);
  stopClaw();
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

float clawServoAngle() {
  return constrain(cfg.open + clawPos * cfg.dir, 0.0f, CLAW_RANGE);
}

int clawMicroseconds() {
  return lroundf(CLAW_MIN_US + clawServoAngle() * (CLAW_MAX_US - CLAW_MIN_US) / CLAW_RANGE);
}

void moveClaw(float target, float rate, bool jog) {
  if (target == clawPos) { stopClaw(); return; }
  if (clawPos == clawTarget) changed();
  if (target != clawTarget || rate != clawRate || jog != clawJogging) lastClawUpdate = millis();
  clawTarget = target;
  clawRate = rate;
  clawJogging = jog;
}

void jogClaw(int velocity) {
  if (velocity == 0) { stopClaw(); return; }
  moveClaw(velocity > 0 ? cfg.closeLimit : 0.0f, CLAW_SPEED * abs(velocity) / 100.0f, true);
}

void clawLoop() {
  if (clawPos == clawTarget) return;
  unsigned long now = millis();
  unsigned long elapsed = now - lastClawUpdate;
  if (elapsed < CLAW_INTERVAL_MS) return;
  lastClawUpdate = now;
  float travel = clawRate * (elapsed / 1000.0f);
  float remaining = clawTarget - clawPos;
  if (fabs(remaining) <= travel) {
    clawPos = clawTarget;
    clawJogging = false;
  } else {
    clawPos += remaining > 0 ? travel : -travel;
  }
  int pulse = clawMicroseconds();
  if (pulse != clawPulse) {
    clawServo.writeMicroseconds(pulse);
    clawPulse = pulse;
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
    if (a < 0 || a > cfg.closeLimit) { message(F("Claw target outside limits"), true); return; }
    moveClaw(a, CLAW_SPEED, false);
  } else if (!strcmp(cmd, "HC") && count == 1) {
    moveClaw(0, CLAW_SPEED, false);
  } else if (!strcmp(cmd, "ZC") && count == 1) {
    cfg.open = clawServoAngle();
    clawPos = 0; stopClaw(); configChanged();
    message(F("Claw zero marked"));
  } else if (!strcmp(cmd, "KC") && count == 1) {
    if (clawPos <= 0) { message(F("Claw close limit must be positive"), true); return; }
    stopClaw();
    cfg.closeLimit = clawPos; configChanged();
  } else if (!strcmp(cmd, "CDIR") && count == 1) {
    if (clawPos != 0) { message(F("Open the claw first"), true); return; }
    stopClaw();
    cfg.dir = -cfg.dir; configChanged();
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
  j2.setPinsInverted(cfg.inv2, false, false); j3.setPinsInverted(cfg.inv3, false, false);
  j2.setAcceleration(cfg.accel); j3.setAcceleration(cfg.accel);
  j2.setMaxSpeed(cfg.maxSteps * cfg.speedPercent / 100.0f);
  j3.setMaxSpeed(cfg.maxSteps * cfg.speedPercent / 100.0f);
  for (byte i = 0; i < 5; ++i) pinMode(limitPins[i], INPUT_PULLUP);
  clawPos = clawTarget = cfg.clawPos;
  clawPulse = clawMicroseconds();
  clawServo.writeMicroseconds(clawPulse);
  // Apply the full pulse range before the timer can send the first pulse.
  noInterrupts();
  clawServo.attach(CLAW_SERVO_PIN, CLAW_MIN_US, CLAW_MAX_US);
  if (clawServo.readMicroseconds() != clawPulse) clawServo.writeMicroseconds(clawPulse);
  interrupts();
  for (byte i = 0; i < 5; ++i) fitted[i] = digitalRead(limitPins[i]) == LOW;
  driversOn = true; digitalWrite(EN_PIN, LOW);
  idleSince = lastFeed = lastStatus = millis();
  JsonLine hello; hello.print(F("{\"t\":\"hello\",\"fw\":\"ArmDashboard\",\"ver\":1}")); queueLine(hello);
  printConfig(); printStatus();
}

void loop() {
  bool stepsReady = prepareSteps();
  if (stepsReady && driversOn) { j2.run(); j3.run(); }
  if (deferredCommand && !armMoving() && !pending && inputLen == 0) {
    memcpy(input, deferredInput, sizeof(input));
    deferredCommand = false; command();
  }
  readInput();
  if ((jogging || clawJogging) && millis() - lastFeed >= 400) {
    watchdogTripped = true; stopAll(false);
  }
  armLoop();
  if (stepsReady && !invalidatePending) clawLoop();
  if (millis() - lastStatus >= 100) { lastStatus = millis(); printStatus(); }
  persistLoop(); pumpOutput();
}
