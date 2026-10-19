// Minimal QMI8658C driver (accelerometer + gyro) over I2C for the
// LOLIN S3 Mini Pro. Register map per the QMI8658 datasheet.
#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace imu {

constexpr int PIN_SDA = 12;
constexpr int PIN_SCL = 11;

constexpr uint8_t REG_WHO_AM_I = 0x00;   // reads 0x05
constexpr uint8_t REG_CTRL1    = 0x02;
constexpr uint8_t REG_CTRL2    = 0x03;   // accel range / ODR
constexpr uint8_t REG_CTRL3    = 0x04;   // gyro range / ODR
constexpr uint8_t REG_CTRL5    = 0x06;   // low-pass filters
constexpr uint8_t REG_CTRL7    = 0x08;   // sensor enable
constexpr uint8_t REG_STATUS0  = 0x2E;
constexpr uint8_t REG_AX_L     = 0x35;   // AX..AZ, GX..GZ, 12 bytes, little-endian
constexpr uint8_t REG_RESET    = 0x60;

constexpr float ACCEL_LSB_PER_G   = 8192.f;  // +/-4 g range
constexpr float GYRO_LSB_PER_DPS  = 64.f;    // +/-512 dps range

static uint8_t addr_ = 0;

static inline bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static inline bool readRegs(uint8_t reg, uint8_t* buf, size_t len) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr_, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

// If a slave was left mid-transaction by a reset it can hold SDA low; clock
// SCL until it releases, then issue a STOP.
static void busRecover() {
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN); pinMode(PIN_SDA, INPUT_PULLUP);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; ++i) {
    digitalWrite(PIN_SCL, LOW); delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SDA, LOW); delayMicroseconds(5);
  digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);     // STOP
}

static bool probe() {
  const uint8_t candidates[2] = {0x6B, 0x6A};   // SA0 high / low
  for (uint8_t a : candidates) {
    addr_ = a;
    uint8_t who = 0;
    if (readRegs(REG_WHO_AM_I, &who, 1) && who == 0x05) return true;
  }
  addr_ = 0;
  return false;
}

// Returns true if the IMU was found and configured. Retries for ~1 s with a
// bus recovery in between, since a soft reset can leave the chip mid-transfer.
inline bool begin() {
  bool found = false;
  for (int attempt = 0; attempt < 10 && !found; ++attempt) {
    if (attempt > 0) { Wire.end(); busRecover(); delay(50); }
    Wire.begin(PIN_SDA, PIN_SCL, 400000);
    Wire.setTimeOut(50);
    found = probe();
  }
  if (!found) return false;

  writeReg(REG_RESET, 0xB0); delay(150);
  writeReg(REG_CTRL1, 0x40);          // address auto-increment
  writeReg(REG_CTRL2, 0x15);          // accel +/-4 g, ODR ~235 Hz
  writeReg(REG_CTRL3, 0x45);          // gyro +/-512 dps, ODR ~235 Hz
  writeReg(REG_CTRL5, 0x77);          // LPF on both, widest bandwidth (13% ODR)
  writeReg(REG_CTRL7, 0x03);          // enable accel + gyro
  delay(50);
  return true;
}

inline uint8_t address() { return addr_; }

// Accel in g, gyro in deg/s (chip axes). Returns false on bus error.
inline bool read(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
  uint8_t b[12];
  if (!readRegs(REG_AX_L, b, 12)) return false;
  auto s16 = [&](int i) { return (int16_t)((uint16_t)b[i] | ((uint16_t)b[i + 1] << 8)); };
  ax = s16(0) / ACCEL_LSB_PER_G;  ay = s16(2) / ACCEL_LSB_PER_G;  az = s16(4) / ACCEL_LSB_PER_G;
  gx = s16(6) / GYRO_LSB_PER_DPS; gy = s16(8) / GYRO_LSB_PER_DPS; gz = s16(10) / GYRO_LSB_PER_DPS;
  return true;
}

} // namespace imu
