#ifndef CONFIG_H
#define CONFIG_H

/*
 * Config.h - Khai bao chung cho toan bo sketch DRONE (sender)
 *   - Gom cac #include thu vien
 *   - Dinh nghia struct ESP-NOW + chan (pin)
 *   - Khai bao "extern" TAT CA bien global (dinh nghia that nam o Globals.cpp)
 * Moi module .cpp chi can #include "Config.h" la thay dung cung mot bien global.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Kalman.h>
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "Bitcraze_PMW3901.h"
#include "MPU6050.h"   // class HungVo_IMU (file doi ten tu HungVo_IMU)

// =====================================================
// MACRO TEST: 1 = bay binh thuong | 0 = TAT HAN ESP-NOW
// =====================================================
#define EN_ESPNOW 1

// =====================================================
// ESP-NOW: 1 goi tin gop tat ca trang thai
// =====================================================
extern uint8_t receiverAddress[];

typedef struct {
  float roll, pitch, yaw;
  float alt;
  float targetHeight;
  float verticalSpeed;
  float pos_x, pos_y;
  float V_real_x, V_real_y;
  float pwm_x, pwm_y;
  int16_t deltaX, deltaY;
  float v_bat;
  uint8_t armed;
} TelemetryPacket;

extern TelemetryPacket tele;
extern esp_now_peer_info_t peerInfo;

// =====================================================
// PINS
// =====================================================
#define PMW_CS_PIN    10
#define PMW_MOSI_PIN  13
#define PMW_SCK_PIN   12
#define PMW_MISO_PIN  11
#define PIN_ADC 7
#define PIN_RGB 48

extern const int motorPins[]; // RR, FR, RL, FL
extern const int rcPins[];

// =====================================================
// SENSOR OBJECTS
// =====================================================
extern TwoWire I2C_IMU;
extern TwoWire I2C_TOF;
extern HungVo_IMU myIMU;
extern VL53L1X tof;
extern Bitcraze_PMW3901 flow;
extern Kalman kalmanR, kalmanP;
extern Adafruit_NeoPixel pixels;

// =====================================================
// INTER-CORE SHARED
// =====================================================
extern TaskHandle_t NavigationTaskHandle;
extern TaskHandle_t TelemetryTaskHandle;

extern volatile float shared_altitudeCorrection;
extern volatile float shared_smoothCorrection;
extern volatile float shared_pwm_opt_x;
extern volatile float shared_pwm_opt_y;
extern volatile bool  shared_pos_hold_active;

// =====================================================
// FLIGHT STATE (CORE 1)
// =====================================================
extern volatile unsigned long pulseStart[6];
extern volatile int rcValue[6];
extern volatile int16_t rc4_min;

extern bool isArmed;
extern bool isReadyToFly;
extern unsigned long armStartTime;
extern const int idleSpeed;
extern const float dt;

extern float filtered_voltage;
extern float pitchOffset, rollOffset, yawGyroOffset;
extern float gyroXOffset, gyroYOffset;

extern float dynamicYawBias;
extern float fRoll, fPitch, fYaw;
extern float gz_filtered;

extern float PAngle;
extern float PRate, IRate, DRate, PRateYaw;
extern float iTermRoll, iTermPitch, lastErrorRoll, lastErrorPitch, iTermYaw, lastYawError;

// =====================================================
// NAVIGATION STATE
// =====================================================
extern bool altitudeHold;
extern bool lastAltHoldState;
extern String altMode;
extern float tofHeight, filteredHeight, lastHeight;
extern float verticalSpeed, targetHeight, altitudeCorrection;
extern float P_alt, I_alt, D_alt, iTermAlt;

extern float V_real_x, V_real_y;
extern volatile float pos_x, pos_y;
extern volatile int16_t deltaX, deltaY;
extern bool pmwInitOk;
extern unsigned long lastFlowTime;

#endif
