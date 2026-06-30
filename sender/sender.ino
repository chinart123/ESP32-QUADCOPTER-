/*
 * 1S DRONE FLIGHT CONTROLLER (DUAL-CORE) - ESP32-S3 SUPERMINI
 * CORE 1 (FLIGHT-CRITICAL): MPU6050, VL53L1X, PMW3901, PID & Motor Mix
 * CORE 0 (NETWORK): ESP-NOW telemetry
 * BAN MODULE HOA: tach MPU6050 / VL53L1X / PMW3901 / ESP-NOW ra file rieng.
 *   main.cpp chi giu phan dieu khien bay loi + goi cac module.
 * HUNG VO | 2026
 */

#include "Config.h"
#include "VL53L1X_Sensor.h"
#include "PMW3901_Sensor.h"
#include "ESPNow_Telemetry.h"

// =====================================================
// RC INTERRUPTS (IRAM)
// =====================================================
void IRAM_ATTR isr0() { if (digitalRead(rcPins[0])) pulseStart[0]=micros(); else rcValue[0]=micros()-pulseStart[0]; }
void IRAM_ATTR isr1() { if (digitalRead(rcPins[1])) pulseStart[1]=micros(); else rcValue[1]=micros()-pulseStart[1]; }
void IRAM_ATTR isr2() { if (digitalRead(rcPins[2])) pulseStart[2]=micros(); else rcValue[2]=micros()-pulseStart[2]; }
void IRAM_ATTR isr3() { if (digitalRead(rcPins[3])) pulseStart[3]=micros(); else rcValue[3]=micros()-pulseStart[3]; }
void IRAM_ATTR isr4() { if (digitalRead(rcPins[4])) pulseStart[4]=micros(); else rcValue[4]=micros()-pulseStart[4]; }
void IRAM_ATTR isr5() { if (digitalRead(rcPins[5])) pulseStart[5]=micros(); else rcValue[5]=micros()-pulseStart[5]; }

// =====================================================
// SMART CALIBRATE
// =====================================================
void smartCalibrate() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 255)); pixels.show(); delay(3000);
  int validSamples = 0;
  float sumP = 0, sumR = 0, sumGZ = 0, sumGX = 0, sumGY = 0;
  const float gyroThreshold = 15.0f;

  while (validSamples < 2000) {
    myIMU.update();
    if (abs(myIMU.getGyroX()) > gyroThreshold || abs(myIMU.getGyroY()) > gyroThreshold) {
      validSamples = 0; sumP = 0; sumR = 0; sumGZ = 0; sumGX = 0; sumGY = 0;
      pixels.setPixelColor(0, pixels.Color(255, 0, 255)); pixels.show();
    } else {
      validSamples++;
      sumP  += -myIMU.getRawRollAngle();
      sumR  += myIMU.getRawPitchAngle();
      sumGZ += myIMU.getGyroZ();
      sumGX += myIMU.getGyroX();
      sumGY += myIMU.getGyroY();
      pixels.setPixelColor(0, pixels.Color(255, 255, 255)); pixels.show();
    }
    delay(2);
  }
  pitchOffset   = sumP / 2000.0f;
  rollOffset    = sumR / 2000.0f;
  yawGyroOffset = sumGZ / 2000.0f;
  gyroXOffset   = sumGX / 2000.0f;
  gyroYOffset   = sumGY / 2000.0f;

  kalmanP.setAngle(0); kalmanR.setAngle(0); fYaw = 0; isReadyToFly = true;
}

// =====================================================
// CORE 1: NAVIGATION TASK (SENSORS + PID)
// Chay tren Core 1 cung voi IMU. An toan tuyet doi khoi nhieu Wi-Fi.
// =====================================================
void NavigationCore1Task(void * pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 4 / portTICK_PERIOD_MS;

  for(;;) {
    updateToF();   // ToF + Alt-Hold PID  (VL53L1X_Sensor.cpp)
    updatePMW();   // Optical flow + Pos-Hold PID (PMW3901_Sensor.cpp)
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// =====================================================
// SETUP (CORE 1)
// =====================================================
void setup() {
  Serial.begin(921600);
  pixels.begin(); pixels.setBrightness(100);

  setupEspNow();   // ESPNow_Telemetry.cpp

  setupPMW();      // PMW3901_Sensor.cpp

  I2C_IMU.begin(5, 6, 400000);
  if (!myIMU.begin(5, 6)) { Serial.println("IMU INIT FAIL"); while (1); }

  setupToF();      // VL53L1X_Sensor.cpp

  kalmanR.setRmeasure(0.15f); kalmanP.setRmeasure(0.15f);
  kalmanR.setQangle(0.003f);  kalmanP.setQangle(0.003f);
  kalmanR.setQbias(0.003f);   kalmanP.setQbias(0.003f);

  smartCalibrate();

  for (int i = 0; i < 6; i++) {
    pinMode(rcPins[i], INPUT_PULLDOWN);
    if (i==0) attachInterrupt(rcPins[0], isr0, CHANGE);
    if (i==1) attachInterrupt(rcPins[1], isr1, CHANGE);
    if (i==2) attachInterrupt(rcPins[2], isr2, CHANGE);
    if (i==3) attachInterrupt(rcPins[3], isr3, CHANGE);
    if (i==4) attachInterrupt(rcPins[4], isr4, CHANGE);
    if (i==5) attachInterrupt(rcPins[5], isr5, CHANGE);
  }
  for (int i = 0; i < 4; i++) { ledcAttach(motorPins[i], 8000, 10); ledcWrite(motorPins[i], 0); }

  xTaskCreatePinnedToCore(NavigationCore1Task, "NavTask", 4096, NULL, 2, &NavigationTaskHandle, 1);
  xTaskCreatePinnedToCore(TelemetryTask, "TeleTask", 2048, NULL, 1, &TelemetryTaskHandle, 0);
}

// =====================================================
// MAIN LOOP (CORE 1)
// =====================================================
void loop() {
  unsigned long startLoop = micros();
  myIMU.update();

  if (rcValue[4] < rc4_min) rc4_min = rcValue[4];

  float targetR = map(rcValue[0], 1000, 2000, -25, 25);
  float targetP = map(rcValue[1], 1000, 2000, -25, 25);
  int throttle  = map(rcValue[2], 1000, 2000, 40, 850);
  float stickYawRate = map(rcValue[3], 1000, 2000, 150, -150);
  if (abs(stickYawRate) < 25) stickYawRate = 0;
  if (abs(rcValue[0] - 1500) < 15) targetR = 0;
  if (abs(rcValue[1] - 1500) < 15) targetP = 0;

  if (shared_pos_hold_active) {
    targetR += shared_pwm_opt_x;
    targetP += shared_pwm_opt_y;
    targetR = constrain(targetR, -12.0f, 12.0f);
    targetP = constrain(targetP, -12.0f, 12.0f);
  }

  float rawP = (-myIMU.getRawRollAngle()) - pitchOffset;
  float rawR = (myIMU.getRawPitchAngle()) - rollOffset;
  float rawGyroZ = myIMU.getGyroZ() - yawGyroOffset;
  if (abs(stickYawRate) < 8 && abs(fRoll) < 6 && abs(fPitch) < 6 && abs(verticalSpeed) < 40 && throttle > 180) {
    dynamicYawBias = (dynamicYawBias * 0.999999f) + (rawGyroZ * 0.000001f);
  }
  float gz_clean = rawGyroZ - dynamicYawBias;
  gz_filtered = (gz_filtered * 0.89f) + (gz_clean * 0.11f);
  fPitch = kalmanP.getAngle(rawP, -myIMU.getGyroX(), dt);
  fPitch = fPitch - 2.0f;
  fRoll  = kalmanR.getAngle(rawR, myIMU.getGyroY(), dt);
  fYaw  -= gz_filtered * dt;

  if (isArmed && (abs(fRoll) > 40 || abs(fPitch) > 40)) isArmed = false;
  if (!isArmed && rcValue[4] > 1550 && rcValue[2] < 1100) {
    isArmed = true; armStartTime = millis();
    fYaw = 0; iTermRoll = 0; iTermPitch = 0; iTermYaw = 0; lastYawError = 0;
    pos_x = 0; pos_y = 0;
  }
  if (rcValue[4] < 1450) isArmed = false;

  bool mode_AltHold = (rcValue[5] > 1400 && rcValue[5] < 2100);
  bool mode_PosHold = (rcValue[5] > 1900 && rcValue[5] < 2100);
  bool switchActive = (mode_AltHold || mode_PosHold);

  static int centerThrottlePosition = 1500;
  static float hoverThrottle = 0;
  if (switchActive && !lastAltHoldState) {
    if (rcValue[2] >= 1100 && rcValue[2] <= 1900) {
      altitudeHold = true; targetHeight = filteredHeight;
      hoverThrottle = throttle; centerThrottlePosition = rcValue[2]; iTermAlt = 0;
    }
  } else if (!switchActive) {
    altitudeHold = false; altMode = "OFF"; shared_altitudeCorrection = 0;
  }
  lastAltHoldState = switchActive;

  if (altitudeHold) {
    int throttleOffset = rcValue[2] - centerThrottlePosition;
    int deadband = 110;
    if (throttleOffset > deadband) {
      if (filteredHeight >= 2200.0f) { altMode = "HOLD"; targetHeight = 2200.0f; }
      else { altMode = "UP"; targetHeight = filteredHeight; iTermAlt = 0;
        shared_altitudeCorrection = map(rcValue[2], centerThrottlePosition + deadband, 2000, 0, 150); }
    } else if (throttleOffset < -deadband) {
      altMode = "DOWN"; targetHeight = filteredHeight; iTermAlt = 0;
      shared_altitudeCorrection = map(rcValue[2], 1000, centerThrottlePosition - deadband, -50, 0);
    } else { altMode = "HOLD"; }

    float tiltComp = 1.0f;
    if (abs(fRoll) >= 5 || abs(fPitch) >= 5)
      tiltComp = constrain(1.0f / (cosf(radians(fRoll)) * cosf(radians(fPitch))), 1.0f, 1.15f);

    if (altMode == "HOLD") throttle = (hoverThrottle * tiltComp) + shared_smoothCorrection;
    else throttle = (hoverThrottle * tiltComp) + shared_altitudeCorrection;
    throttle = constrain(throttle, 120, 800);
  }

  float tarRateR = PAngle * (targetR - fRoll);
  float errR = tarRateR - myIMU.getGyroY();
  if (throttle < 50) iTermRoll = 0; else iTermRoll = constrain(iTermRoll + IRate * errR * dt, -200, 200);
  float outRoll = (PRate * errR) + iTermRoll + DRate * (errR - lastErrorRoll) / dt;
  lastErrorRoll = errR;

  float tarRateP = PAngle * (targetP - fPitch);
  float errP = tarRateP - (-myIMU.getGyroX());
  if (throttle < 50) iTermPitch = 0; else iTermPitch = constrain(iTermPitch + IRate * errP * dt, -200, 200);
  float outPitch = (PRate * errP) + iTermPitch + DRate * (errP - lastErrorPitch) / dt;
  lastErrorPitch = errP;

  float yawRateError = stickYawRate - gz_filtered;
  if (abs(stickYawRate) < 10) iTermYaw = constrain(iTermYaw + yawRateError * 0.7f * dt, -220, 220);
  else iTermYaw *= 0.985f;
  float dYaw = (yawRateError - lastYawError) / dt;
  lastYawError = yawRateError;
  float outYaw = (PRateYaw * yawRateError) + iTermYaw + (0.0028f * dYaw);

  if (isArmed) {
    int baseThr = (throttle < 25) ? idleSpeed : throttle;
    ledcWrite(3, constrain(baseThr - outPitch + outRoll + outYaw, 30, 1023)); // FL
    ledcWrite(4, constrain(baseThr - outPitch - outRoll - outYaw, 30, 1023)); // FR
    ledcWrite(2, constrain(baseThr + outPitch + outRoll - outYaw, 30, 1023)); // RL
    ledcWrite(1, constrain(baseThr + outPitch - outRoll + outYaw, 30, 1023)); // RR
  } else {
    for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], 0);
  }

  float v = (analogRead(PIN_ADC) / 4095.0f) * 3.1f * 2.0f * 1.1455f;
  filtered_voltage = filtered_voltage * 0.95f + v * 0.05f;
  if (!isArmed) pixels.setPixelColor(0, pixels.Color(255, 100, 0));
  else if (filtered_voltage > 3.85f) pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  else if (filtered_voltage > 3.65f) pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  else                               pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  while (micros() - startLoop < 4000);
}
