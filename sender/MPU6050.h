#ifndef HUNGVO_IMU_H
#define HUNGVO_IMU_H

#include <Arduino.h>
#include <Wire.h>

class HungVo_IMU {
  public:
    HungVo_IMU(TwoWire &w);
    bool begin(int sda, int scl);
    void calibrate(int samples = 2000); 
    void update();

    // C�c h�m l?y gi� tr? th� d? n?p v�o Kalman
    float getRawRollAngle();  
    float getRawPitchAngle(); 
    float getGyroX();         
    float getGyroY();         
    float getGyroZ();         

  private:
    TwoWire *_wire;
    int16_t _ax, _ay, _az, _gx, _gy, _gz;
    float _offAX, _offAY, _offAZ, _offGX, _offGY, _offGZ;
    
    void writeReg(uint8_t reg, uint8_t val);
    void readBurst(); 
};

#endif
