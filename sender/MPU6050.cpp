#include "MPU6050.h"

HungVo_IMU::HungVo_IMU(TwoWire &w) { _wire = &w; }

bool HungVo_IMU::begin(int sda, int scl) {
  _wire->begin(sda, scl);
  _wire->setClock(400000);

  writeReg(0x6B, 0x00); // Wake up
  writeReg(0x1A, 0x05); // DLPF 10Hz - L?c c?c m?nh cho motor 8520
  writeReg(0x1B, 0x08); // Gyro +/- 500 dps (H? s? 65.5)
  writeReg(0x1C, 0x10); // Accel +/- 8g (H? s? 4096)
  return true; 
}

void HungVo_IMU::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(0x68);
  _wire->write(reg);
  _wire->write(val);
  _wire->endTransmission();
}

void HungVo_IMU::readBurst() {
  _wire->beginTransmission(0x68);
  _wire->write(0x3B);
  _wire->endTransmission(false);
  _wire->requestFrom(0x68, 14);
  _ax = _wire->read() << 8 | _wire->read();
  _ay = _wire->read() << 8 | _wire->read();
  _az = _wire->read() << 8 | _wire->read();
  _wire->read(); _wire->read(); // Skip Temp
  _gx = _wire->read() << 8 | _wire->read();
  _gy = _wire->read() << 8 | _wire->read();
  _gz = _wire->read() << 8 | _wire->read();
}

void HungVo_IMU::calibrate(int samples) {
  float sumAX=0, sumAY=0, sumAZ=0, sumGX=0, sumGY=0, sumGZ=0;
  for(int i=0; i<samples; i++) {
    readBurst();
    sumAX += (float)_ax/4096.0; sumAY += (float)_ay/4096.0; sumAZ += (float)_az/4096.0;
    sumGX += (float)_gx/65.5;  sumGY += (float)_gy/65.5;  sumGZ += (float)_gz/65.5;
    delay(1);
  }
  _offAX = sumAX/samples; _offAY = sumAY/samples; 
  _offAZ = (sumAZ/samples) - 1.0; // Hi?u chu?n tr?ng l?c 1g
  _offGX = sumGX/samples; _offGY = sumGY/samples; _offGZ = sumGZ/samples;
}

void HungVo_IMU::update() { readBurst(); }

float HungVo_IMU::getRawRollAngle() {
  float ay = ((float)_ay/4096.0) - _offAY;
  float az = ((float)_az/4096.0) - _offAZ;
  return atan2(ay, az) * 57.2958;
}

float HungVo_IMU::getRawPitchAngle() {
  float ax = ((float)_ax/4096.0) - _offAX;
  float ay = ((float)_ay/4096.0) - _offAY;
  float az = ((float)_az/4096.0) - _offAZ;
  return atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
}

float HungVo_IMU::getGyroX() { return ((float)_gx/65.5) - _offGX; }
float HungVo_IMU::getGyroY() { return ((float)_gy/65.5) - _offGY; }
float HungVo_IMU::getGyroZ() { return ((float)_gz/65.5) - _offGZ; }
