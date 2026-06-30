#include "PMW3901_Sensor.h"

// =====================================================
// SETUP PMW3901 (goi trong setup())
// =====================================================
void setupPMW() {
  pinMode(PMW_MISO_PIN, INPUT_PULLUP);
  SPI.begin(PMW_SCK_PIN, PMW_MISO_PIN, PMW_MOSI_PIN, PMW_CS_PIN);
  delay(100);
  pmwInitOk = flow.begin();
  Serial.println(pmwInitOk ? "PMW INIT OK" : "PMW INIT FAIL");
}

// =====================================================
// UPDATE PMW3901 + POS HOLD (goi trong NavigationCore1Task)
// =====================================================
void updatePMW() {
  // ---------------- PMW3901 SENSOR FUSION + POS HOLD ----------------
  unsigned long currentFlowTime = micros();
  if (pmwInitOk && (currentFlowTime - lastFlowTime >= 10000)) { // 10ms
    float dt_flow = (currentFlowTime - lastFlowTime) / 1000000.0f;
    int16_t raw_dx, raw_dy;
    flow.readMotionCount(&raw_dx, &raw_dy);
    lastFlowTime = currentFlowTime;

    deltaX = raw_dx;
    deltaY = -raw_dy;

    static float last_fPitch_flow = 0, last_fRoll_flow = 0;
    float delta_pitch_rad = radians(fPitch - last_fPitch_flow);
    float delta_roll_rad  = radians(fRoll - last_fRoll_flow);
    last_fPitch_flow = fPitch;
    last_fRoll_flow  = fRoll;

    const float PIXEL_TO_RAD = 0.00186f;
    float flow_angle_x = deltaX * PIXEL_TO_RAD;
    float flow_angle_y = deltaY * PIXEL_TO_RAD;

    float GYRO_COMP = 0.83f;
    float real_angle_x = flow_angle_x + (delta_roll_rad * GYRO_COMP);
    float real_angle_y = flow_angle_y + (delta_pitch_rad * GYRO_COMP);
    float height_cm = constrain(filteredHeight / 10.0f, 5.0f, 200.0f);

    float dx_cm = real_angle_x * height_cm;
    float dy_cm = real_angle_y * height_cm;
    float raw_v_x = dx_cm / dt_flow;
    float raw_v_y = dy_cm / dt_flow;
    V_real_x = (V_real_x * 0.001f) + (raw_v_x * 0.999f);
    V_real_y = (V_real_y * 0.001f) + (raw_v_y * 0.999f);

    pos_x += dx_cm; pos_y += dy_cm;
    pos_x = constrain(pos_x, -200.0f, 200.0f);
    pos_y = constrain(pos_y, -200.0f, 200.0f);

    bool wantPosHold   = (rcValue[5] > 1900 && rcValue[5] < 2100);
    bool stickOverride = (rcValue[0] < 1430 || rcValue[0] > 1560 || rcValue[1] < 1400 || rcValue[1] > 1600);
    bool isBouncing    = (abs(verticalSpeed) > 150.0f);

    if (wantPosHold && !stickOverride && !isBouncing && filteredHeight > 80) {
      float BASE_HEIGHT_CM = 50.0f;
      float alt_factor = constrain(BASE_HEIGHT_CM / height_cm, 0.65f, 1.3f);

      float Kp_pos = 0.6f * alt_factor;
      float Ki_pos = 0.008f;
      float raw_error_x = 0.0f - pos_x;
      float raw_error_y = 0.0f - pos_y;
      float error_pos_x = 0, error_pos_y = 0;
      static float iTerm_pos_x = 0, iTerm_pos_y = 0;
      float DEAD_ZONE_CM = 0.4f;

      if (abs(raw_error_x) < DEAD_ZONE_CM) { error_pos_x = 0; iTerm_pos_x *= 0.9f; }
      else error_pos_x = (raw_error_x > 0) ? raw_error_x - DEAD_ZONE_CM : raw_error_x + DEAD_ZONE_CM;
      if (abs(raw_error_y) < DEAD_ZONE_CM) { error_pos_y = 0; iTerm_pos_y *= 0.9f; }
      else error_pos_y = (raw_error_y > 0) ? raw_error_y - DEAD_ZONE_CM : raw_error_y + DEAD_ZONE_CM;

      iTerm_pos_x = constrain(iTerm_pos_x + error_pos_x * dt_flow, -10.0f, 10.0f);
      iTerm_pos_y = constrain(iTerm_pos_y + error_pos_y * dt_flow, -10.0f, 10.0f);
      float target_V_x = constrain((error_pos_x * Kp_pos) + iTerm_pos_x * Ki_pos, -10.0f, 10.0f);
      float target_V_y = constrain((error_pos_y * Kp_pos) + iTerm_pos_y * Ki_pos, -10.0f, 10.0f);

      float Kp_vel = 0.25f * alt_factor;
      float Kd_vel = 0.37f * alt_factor;
      float error_vel_x = target_V_x - V_real_x;
      float error_vel_y = target_V_y - V_real_y;
      static float last_error_vel_x = 0, last_error_vel_y = 0;
      static float filtered_d_x = 0, filtered_d_y = 0;
      float raw_d_x = error_vel_x - last_error_vel_x;
      float raw_d_y = error_vel_y - last_error_vel_y;
      filtered_d_x = (filtered_d_x * 0.55f) + (raw_d_x * 0.45f);
      filtered_d_y = (filtered_d_y * 0.55f) + (raw_d_y * 0.45f);
      last_error_vel_x = error_vel_x;
      last_error_vel_y = error_vel_y;

      float raw_angle_x = (error_vel_x * Kp_vel) + (filtered_d_x * Kd_vel);
      float raw_angle_y = (error_vel_y * Kp_vel) + (filtered_d_y * Kd_vel);
      shared_pwm_opt_x = constrain(raw_angle_x, -15.0f, 15.0f);
      shared_pwm_opt_y = constrain(raw_angle_y, -15.0f, 15.0f);
      shared_pos_hold_active = true;
    } else {
      pos_x = 0; pos_y = 0;
      shared_pwm_opt_x = 0; shared_pwm_opt_y = 0;
      shared_pos_hold_active = false;
    }
  }
}
