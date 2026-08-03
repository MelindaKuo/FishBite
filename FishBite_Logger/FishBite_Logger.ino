#include <Arduino.h>
#include <Wire.h>
#include <string.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

constexpr uint32_t SAMPLE_HZ = 100;
constexpr uint32_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_HZ;

constexpr uint8_t MPU_ADDR = 0x68;

constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr float ACC_LSB_PER_G    = 8192.0f;

constexpr char BLE_DEVICE_NAME[]      = "FishBiteLogger";
constexpr char BLE_SERVICE_UUID[]     = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char BLE_MARKER_CHAR_UUID[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr uint32_t MARKER_LOOKBACK_HINT_MS = 15000;

enum MarkerType : uint8_t {
  MARKER_NONE          = 0,
  MARKER_BITE          = 1,
  MARKER_FALSE_TRIGGER = 2,
};

#pragma pack(push, 1)
struct LogRecord {
  uint32_t t_ms;
  int16_t  ax, ay, az;
  int16_t  gx, gy, gz;
  uint8_t  marker;
};
#pragma pack(pop)
static_assert(sizeof(LogRecord) == 17, "LogRecord layout changed - update analyze.py parser too");

constexpr size_t recordSize() { return sizeof(LogRecord); }

static bool g_loggingActive = false;
static uint32_t g_lastSampleMs = 0;

static volatile uint8_t  g_pendingMarkerType  = MARKER_NONE;
static volatile uint32_t g_pendingMarkerAtMs  = 0;
static volatile bool     g_bleConnected       = false;

class LoggerServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    g_bleConnected = true;
  }
  void onDisconnect(BLEServer *server) override {
    g_bleConnected = false;
    server->getAdvertising()->start();
  }
};

class MarkerWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    std::string v = chr->getValue();
    if (v.empty()) {
      return;
    }
    uint8_t b = static_cast<uint8_t>(v[0]);
    if (b == 1 || b == '1') {
      g_pendingMarkerType = MARKER_BITE;
      g_pendingMarkerAtMs = millis();
    } else if (b == 2 || b == '2') {
      g_pendingMarkerType = MARKER_FALSE_TRIGGER;
      g_pendingMarkerAtMs = millis();
    }
  }
};

void bleInit() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new LoggerServerCallbacks());

  BLEService *service = server->createService(BLE_SERVICE_UUID);
  BLECharacteristic *markerChar = service->createCharacteristic(
      BLE_MARKER_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  markerChar->setCallbacks(new MarkerWriteCallbacks());
  markerChar->addDescriptor(new BLE2902());

  service->start();
  server->getAdvertising()->addServiceUUID(service->getUUID());
  server->getAdvertising()->start();
}

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
  ok &= mpuWriteReg(REG_PWR_MGMT_1, 0x01);
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
  Wire.write(REG_ACCEL_XOUT_H);
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

void printCsvHeader(uint32_t t0Ms) {
  Serial.printf("# FishBite log v1 sample_hz=%lu record_bytes=%u t0_ms=%lu\n",
                (unsigned long)SAMPLE_HZ, (unsigned)recordSize(), (unsigned long)t0Ms);
}

void printCsvRow(const LogRecord &r) {
  Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%u\n",
                (unsigned long)r.t_ms, r.ax, r.ay, r.az, r.gx, r.gy, r.gz,
                (unsigned)r.marker);
}

void handleSerialCommands() {
  static char buf[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        if (strcmp(buf, "start") == 0) {
          g_loggingActive = true;
          printCsvHeader(millis());
        } else if (strcmp(buf, "stop") == 0) {
          g_loggingActive = false;
          Serial.println("# stopped");
        } else {
          Serial.printf("# unknown command: %s\n", buf);
        }
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeOut(5);

  if (!mpuInit()) {
    Serial.println("WARN: MPU6050 init incomplete - check wiring");
  }

  bleInit();

  g_lastSampleMs = millis();

  Serial.println("FishBite_Logger idle.");
  Serial.println("Serial commands: 'start' begins a session, 'stop' ends it.");
  Serial.println("Marker input over BLE: connect to \"FishBiteLogger\" and write");
  Serial.println("1 = bite, 2 = false trigger to the marker characteristic, any time");
  Serial.println("after noticing the event.");
}

void loop() {
  handleSerialCommands();

  uint32_t now = millis();

  if (g_pendingMarkerType != MARKER_NONE) {
    uint8_t markerType = g_pendingMarkerType;
    uint32_t markerAtMs = g_pendingMarkerAtMs;
    g_pendingMarkerType = MARKER_NONE;

    if (g_loggingActive) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readIMU(ax, ay, az, gx, gy, gz)) {
        LogRecord rec{markerAtMs, ax, ay, az, gx, gy, gz, markerType};
        printCsvRow(rec);
      }
    }
  }

  if ((int32_t)(now - g_lastSampleMs) < (int32_t)SAMPLE_INTERVAL_MS) {
    return;
  }
  g_lastSampleMs += SAMPLE_INTERVAL_MS;
  if ((int32_t)(now - g_lastSampleMs) > 250) {
    g_lastSampleMs = now;
  }

  if (!g_loggingActive) {
    return;
  }

  int16_t ax, ay, az, gx, gy, gz;
  if (!readIMU(ax, ay, az, gx, gy, gz)) {
    return;
  }

  LogRecord rec{now, ax, ay, az, gx, gy, gz, MARKER_NONE};
  printCsvRow(rec);
}
