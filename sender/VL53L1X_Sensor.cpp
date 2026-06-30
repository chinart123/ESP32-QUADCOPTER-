#include "VL53L1X_Sensor.h"

// =====================================================
// SETUP ToF (goi trong setup() - Core 1)
// =====================================================
void setupToF() {
  I2C_TOF.begin(8, 9, 400000);
  tof.setBus(&I2C_TOF);
  if (!tof.init()) { Serial.println("TOF INIT FAIL"); while (1); }
  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(20);
}

// =====================================================
// UPDATE ToF + ALT PID (goi trong NavigationCore1Task)
// =====================================================
void updateToF() {
  static unsigned long lastTofTime = micros();

  // ---------------- TOF + ALT PID ----------------
  if (tof.dataReady()) {
    float rawTof = constrain(tof.read(false), 25, 2250);
    float trueVerticalHeight = rawTof * cosf(radians(fRoll)) * cosf(radians(fPitch));
    tofHeight = trueVerticalHeight;
    filteredHeight = (filteredHeight * 0.3f) + (tofHeight * 0.7f);

    unsigned long currentTofTime = micros();
    float dt_tof = (currentTofTime - lastTofTime) / 1000000.0f;

    if (dt_tof > 0.005f) {
      float rawVS = (filteredHeight - lastHeight) / dt_tof;
      verticalSpeed = (verticalSpeed * 0.4f) + (rawVS * 0.6f);
    }
    lastHeight = filteredHeight;
    lastTofTime = currentTofTime;
  }

  if (altitudeHold && altMode == "HOLD") {
    float altError = targetHeight - filteredHeight;
    float vung_chet = 2.0f;
    if (abs(altError) < vung_chet) altError = 0;
    else altError = (altError > 0) ? altError - vung_chet : altError + vung_chet;

    // PHUC HOI: Toc do leo dua tren dien ap pin
    float max_climb_speed = (filtered_voltage < 3.85f) ? 0.3f : 0.6f;
    float targetVelocity = (altError < 0) ? -0.2f : constrain(altError * 0.2f, 0, max_climb_speed);
    float velocityError = targetVelocity - verticalSpeed;

    if (rcValue[2] < 1100) iTermAlt = 0;
    else {
      iTermAlt += (I_alt * 1.5f) * velocityError * 0.004f;

      // PHUC HOI: Bu v_moi khan cap khi dang rot nhanh gan muc tieu
      if (abs(targetHeight - filteredHeight) < 45.0f && verticalSpeed < -0.5f) {
        float v_moi = (filtered_voltage < 3.85f) ? 25.0f : 10.0f;
        if (iTermAlt < v_moi) iTermAlt = v_moi;
      }

      // PHUC HOI: Gioi han I-term theo pin, nhung mo rong tran len 250/200 PWM
      iTermAlt = constrain(iTermAlt, -100.0f, (filtered_voltage < 3.8f) ? 250.0f : 200.0f);
    }

    float heSoLamMem = constrain(abs(targetHeight - filteredHeight) / 250.0f, 0.15f, 1.0f);
    float active_D = (abs(targetHeight - filteredHeight) < 45.0f && verticalSpeed < 0) ? D_alt * 1.3f : D_alt;

    altitudeCorrection = (P_alt * 1.5f * heSoLamMem * velocityError) + iTermAlt - (active_D * 0.05f * verticalSpeed);

    // PHUC HOI: Tach biet constrain khi bu am va bu duong
    if ((targetHeight - filteredHeight) < 0) {
      altitudeCorrection = constrain(altitudeCorrection, -40, 40);
    } else {
      // Anh xa gioi han luc day theo pin: Toi da 280 PWM khi pin yeu, va 100 PWM khi pin day
      int maxThrustLimit = constrain(map(filtered_voltage * 100, 380, 420, 280, 100), 150, 250);
      altitudeCorrection = constrain(altitudeCorrection, -120, maxThrustLimit);
    }

    shared_altitudeCorrection = altitudeCorrection;
  }
  shared_smoothCorrection = (shared_smoothCorrection * 0.3f) + (shared_altitudeCorrection * 0.7f);
}
