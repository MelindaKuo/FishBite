#include <Arduino.h>
#include <Wire.h>

#define ENABLE_BLE   1
#define ENABLE_NTFY  0
#define DEBUG_ORIENT 1

#if ENABLE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#endif

#define MPU_ADDR   0x68
#define BUZZER_PIN 25

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

const float GYRO_LSB_PER_DPS = 65.5f;
const float ACC_LSB_PER_G    = 8192.0f;

const uint32_t SAMPLE_INTERVAL_MS = 10;
const uint32_t MUTE_MS            = 3000;

const float   JERK_SMOOTH_ALPHA = 0.30f;
const float   NOISE_ALPHA       = 0.01f;
const float   NOISE_UPDATE_MAX  = 3.0f;
const float   GYR_JERK_FLOOR    = 0.30f;
const float   ACC_JERK_FLOOR    = 0.02f;
const float   TRIG_SCORE        = 9.0f;
const float   STRONG_SCORE      = 40.0f;
const uint8_t PERSIST_N         = 3;

const uint32_t RECAL_MIN_MS     = 1500;
const uint32_t RECAL_MAX_MS     = 10000;
const float    RECAL_ALPHA      = 0.20f;
const float    RECAL_STABLE_PCT = 0.05f;

const float    ACC_SMOOTH_ALPHA = 0.02f;
const float    TILT_LIMIT_DEG   = 5.0f;
const float    COS_TILT_LIMIT   = 0.9962f;
const float    GYR_SETTLE_SCORE = 4.0f;
const uint32_t ORIENT_HOLD_MS   = 2000;

const uint8_t  I2C_FAIL_LIMIT   = 5;

const float    FIGHT_TILT_DEG      = 45.0f;
const float    COS_FIGHT_TILT      = 0.7071f;
const uint32_t FIGHT_START_HOLD_MS = 8000;
const uint32_t FIGHT_END_HOLD_MS   = 5000;

struct JerkChannel {
  float prevMag;
  float jerk;
  float noise;
  float floorVal;
  float score;
  bool  init;
};

JerkChannel gyrCh = {0, 0, GYR_JERK_FLOOR, GYR_JERK_FLOOR, 0, false};
JerkChannel accCh = {0, 0, ACC_JERK_FLOOR, ACC_JERK_FLOOR, 0, false};

struct RecalState {
  bool     active;
  uint32_t start;
  float    lastNoiseCheck;
  uint32_t nextNoiseCheck;
};

struct Orientation {
  float    sax, say, saz;
  bool     accInit;
  float    refX, refY, refZ;
  bool     refValid;
  uint32_t offSince;
};

struct FightState {
  bool     active;
  uint32_t aboveSince;
  uint32_t belowSince;
  uint32_t startMs;
};

RecalState  recal  = {false, 0, 0, 0};
Orientation orient = {0, 0, 0, false, 0, 0, 0, false, 0};
FightState  fight  = {false, 0, 0, 0};

uint32_t lastSample = 0;
uint32_t alertUntil = 0;
uint8_t  overCount  = 0;
uint8_t  i2cFails   = 0;

uint8_t  beepsPending = 0;
bool     buzzerOn     = false;
uint32_t beepToggleAt = 0;
uint32_t beepOnMs = 0, beepOffMs = 0;

#if ENABLE_BLE
BLECharacteristic *bleTx = nullptr;
volatile bool bleConnected = false;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *)     override { bleConnected = true; }
  void onDisconnect(BLEServer *s) override { bleConnected = false; s->getAdvertising()->start(); }
};

void bleInit() {
  BLEDevice::init("FishBite");
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  BLEService *svc = srv->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  bleTx = svc->createCharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
                                    BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  svc->start();
  srv->getAdvertising()->addServiceUUID(svc->getUUID());
  srv->getAdvertising()->start();
}

void bleNotify(const char *msg) {
  if (bleConnected && bleTx) {
    bleTx->setValue((uint8_t *)msg, strlen(msg));
    bleTx->notify();
  }
}
#else
void bleInit() {}
void bleNotify(const char *) {}
#endif

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_WHO_AM_I);
  bool ok = (Wire.endTransmission(false) == 0) &&
            (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) == 1);
  if (ok) {
    uint8_t who = Wire.read();
    if (who != 0x68) {
      Serial.printf("WHO_AM_I=0x%02X (clone/6500? continuing)\n", who);
    }
  }
  ok &= mpuWriteReg(REG_PWR_MGMT_1,   0x01);
  delay(50);
  ok &= mpuWriteReg(REG_CONFIG,       0x03);
  ok &= mpuWriteReg(REG_SMPLRT_DIV,   9);
  ok &= mpuWriteReg(REG_GYRO_CONFIG,  0x08);
  ok &= mpuWriteReg(REG_ACCEL_CONFIG, 0x08);
  return ok;
}

bool readIMU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) {
    return false;
  }

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();
  return true;
}

void updateChannel(JerkChannel &c, float mag, bool muted, bool fastAdapt) {
  if (!c.init) {
    c.prevMag = mag;
    c.jerk = 0;
    c.score = 0;
    c.init = true;
    return;
  }
  float j = fabsf(mag - c.prevMag);
  c.prevMag = mag;
  c.jerk += JERK_SMOOTH_ALPHA * (j - c.jerk);

  float nAlpha = fastAdapt ? RECAL_ALPHA : NOISE_ALPHA;
  if (fastAdapt || (!muted && c.jerk < NOISE_UPDATE_MAX * c.noise)) {
    c.noise += nAlpha * (c.jerk - c.noise);
    if (c.noise < c.floorVal) {
      c.noise = c.floorVal;
    }
  }
  c.score = c.jerk / c.noise;
}

void recalibrate(uint32_t now, const char *reason) {
  gyrCh.init = false;  gyrCh.noise = GYR_JERK_FLOOR;
  accCh.init = false;  accCh.noise = ACC_JERK_FLOOR;
  overCount = 0;

  recal.active         = true;
  recal.start          = now;
  recal.lastNoiseCheck = 0;
  recal.nextNoiseCheck = now + 500;

  orient.offSince = 0;
  alertUntil      = now + RECAL_MAX_MS;

  fight.active     = false;
  fight.aboveSince = 0;
  fight.belowSince = 0;

  Serial.printf("RECAL start (%s)\n", reason);
}

void captureOrientationRef() {
  float m = sqrtf(orient.sax*orient.sax + orient.say*orient.say + orient.saz*orient.saz);
  if (m < 0.5f) {
    return;
  }
  orient.refX = orient.sax / m;
  orient.refY = orient.say / m;
  orient.refZ = orient.saz / m;
  orient.refValid = true;
}

void updateOrientationSmoothing(float axg, float ayg, float azg) {
  float aAlpha = recal.active ? RECAL_ALPHA : ACC_SMOOTH_ALPHA;
  if (!orient.accInit) {
    orient.sax = axg; orient.say = ayg; orient.saz = azg;
    orient.accInit = true;
  } else {
    orient.sax += aAlpha * (axg - orient.sax);
    orient.say += aAlpha * (ayg - orient.say);
    orient.saz += aAlpha * (azg - orient.saz);
  }
}

void checkRecalConvergence(uint32_t now) {
  if (now >= recal.nextNoiseCheck) {
    float delta  = fabsf(gyrCh.noise - recal.lastNoiseCheck);
    bool  stable = (recal.lastNoiseCheck > 0) &&
                   (delta < RECAL_STABLE_PCT * recal.lastNoiseCheck);
    recal.lastNoiseCheck = gyrCh.noise;
    recal.nextNoiseCheck = now + 500;

    if (stable && (now - recal.start) >= RECAL_MIN_MS) {
      recal.active = false;
      alertUntil   = now;
      captureOrientationRef();
      Serial.printf("RECAL done  %lums  gyrNoise=%.2fdps  accNoise=%.3fg\n",
                    now - recal.start, gyrCh.noise, accCh.noise);
      bleNotify("READY");
    }
  }
  if (recal.active && (now - recal.start) >= RECAL_MAX_MS) {
    recal.active = false;
    alertUntil   = now;
    captureOrientationRef();
    Serial.printf("RECAL timeout  gyrNoise=%.2f\n", gyrCh.noise);
    bleNotify("READY");
  }
}

bool checkRepositioning(uint32_t now, float tiltDeg, bool tilted, bool settled) {
  if (tilted && settled) {
    if (orient.offSince == 0) {
      orient.offSince = now;
    } else if (now - orient.offSince >= ORIENT_HOLD_MS) {
      Serial.printf("Rod moved %.1f deg - auto-recal\n", tiltDeg);
      recalibrate(now, "orientation");
      return true;
    }
  } else {
    orient.offSince = 0;
  }
  return false;
}

void updateFightState(uint32_t now, float dot) {
  bool fightTilted = (dot < COS_FIGHT_TILT);
  if (fightTilted) {
    if (fight.aboveSince == 0) {
      fight.aboveSince = now;
    }
    fight.belowSince = 0;
    if (!fight.active && (now - fight.aboveSince) >= FIGHT_START_HOLD_MS) {
      fight.active  = true;
      fight.startMs = fight.aboveSince;
      bleNotify("FIGHT START");
      Serial.println("FIGHT START");
    }
    return;
  }

  fight.aboveSince = 0;
  if (!fight.active) {
    return;
  }

  if (fight.belowSince == 0) {
    fight.belowSince = now;
  }
  if (now - fight.belowSince >= FIGHT_END_HOLD_MS) {
    fight.active = false;
    uint32_t durationS = (fight.belowSince - fight.startMs) / 1000;
    char fmsg[32];
    snprintf(fmsg, sizeof fmsg, "FIGHT END dur=%lus", (unsigned long)durationS);
    bleNotify(fmsg);
    Serial.println(fmsg);
    fight.belowSince = 0;
  }
}

bool updateOrientationAndFight(uint32_t now) {
  if (!orient.refValid) {
    return false;
  }

  float m = sqrtf(orient.sax*orient.sax + orient.say*orient.say + orient.saz*orient.saz);
  if (m <= 0.5f) {
    return false;
  }

  float dot = (orient.sax/m)*orient.refX + (orient.say/m)*orient.refY + (orient.saz/m)*orient.refZ;
  if (dot > 1.0f) {
    dot = 1.0f;
  }
  if (dot < -1.0f) {
    dot = -1.0f;
  }

  float tiltDeg = acosf(dot) * 57.2958f;
  bool  tilted  = (dot < COS_TILT_LIMIT);
  bool  settled = (gyrCh.score < GYR_SETTLE_SCORE);

#if DEBUG_ORIENT
  static uint32_t lastDbg = 0;
  if (now - lastDbg >= 500) {
    lastDbg = now;
    Serial.printf("tilt=%5.1f/%.1fdeg  settled=%d  gyrS=%4.1f  accS=%4.1f  held=%lums  fighting=%d\n",
                  tiltDeg, TILT_LIMIT_DEG, settled ? 1 : 0,
                  gyrCh.score, accCh.score,
                  orient.offSince ? (now - orient.offSince) : 0,
                  fight.active ? 1 : 0);
  }
#endif

  if (checkRepositioning(now, tiltDeg, tilted, settled)) {
    return true;
  }

  updateFightState(now, dot);
  return false;
}

void startBeeps(uint8_t n, uint32_t onMs, uint32_t offMs) {
  beepOnMs = onMs;
  beepOffMs = offMs;
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOn = true;
  beepToggleAt = millis() + onMs;
  beepsPending = n - 1;
}

void serviceBeeps(uint32_t now) {
  if (!buzzerOn && beepsPending == 0) {
    return;
  }
  if ((int32_t)(now - beepToggleAt) < 0) {
    return;
  }
  if (buzzerOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
    beepToggleAt = now + beepOffMs;
  } else if (beepsPending > 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzerOn = true;
    beepToggleAt = now + beepOnMs;
    beepsPending--;
  }
}

void setup() {
  Serial.begin(9600);
  delay(300);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(5);

  if (!mpuInit()) {
    Serial.println("WARN: MPU6050 init incomplete - check wiring");
  }

  bleInit();

  lastSample = millis();
  recalibrate(lastSample, "boot");
  Serial.println("running  ('r'=recal  'b'=test beep+notify)");
}

void loop() {
  uint32_t now = millis();
  serviceBeeps(now);

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      recalibrate(now, "manual");
    }
    if (c == 'b' || c == 'B') {
      startBeeps(2, 100, 100);
      bleNotify("TEST");
    }
  }

  if ((int32_t)(now - lastSample) < (int32_t)SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSample += SAMPLE_INTERVAL_MS;
  if ((int32_t)(now - lastSample) > 250) {
    lastSample = now;
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!readIMU(ax, ay, az, gx, gy, gz)) {
    if (++i2cFails >= I2C_FAIL_LIMIT) {
      Serial.println("I2C fail - reinitializing sensor");
      Wire.begin();
      Wire.setClock(400000);
      Wire.setTimeOut(5);
      mpuInit();
      recalibrate(now, "i2c recovery");
      i2cFails = 0;
    }
    return;
  }
  i2cFails = 0;

  float gmag = sqrtf((float)gx*gx + (float)gy*gy + (float)gz*gz) / GYRO_LSB_PER_DPS;
  float axg = ax / ACC_LSB_PER_G;
  float ayg = ay / ACC_LSB_PER_G;
  float azg = az / ACC_LSB_PER_G;
  float amag = sqrtf(axg*axg + ayg*ayg + azg*azg);

  updateOrientationSmoothing(axg, ayg, azg);

  bool muted = (now < alertUntil);
  updateChannel(gyrCh, gmag, muted, recal.active);
  updateChannel(accCh, amag, muted, recal.active);

  if (recal.active) {
    checkRecalConvergence(now);
    return;
  }

  if (updateOrientationAndFight(now)) {
    return;
  }

  if (muted) {
    overCount = 0;
    return;
  }

  float combined = gyrCh.score + accCh.score;
  overCount = (combined > TRIG_SCORE) ? (uint8_t)(overCount + 1) : 0;

  if (overCount >= PERSIST_N) {
    overCount  = 0;
    alertUntil = now + MUTE_MS;
    bool strong = (combined > STRONG_SCORE);
    if (strong) {
      startBeeps(3, 120, 100);
    } else {
      startBeeps(1, 350, 0);
    }

    char msg[48];
    snprintf(msg, sizeof msg, "BITE %s score=%.1f",
             strong ? "STRONG" : "nibble", combined);
    bleNotify(msg);
    Serial.printf("%s  gyrS=%.1f accS=%.1f  t=%lu\n",
                  msg, gyrCh.score, accCh.score, now);
  }
}
