#include "Processing.h"

// =====================================================
// LUON in cho Processing (digital twin)
// roll,pitch,yaw,alt,posX,posY,pwmX,pwmY,dX,dY,armed,rc4_min,targetH,vSpeed,Vx,Vy,dt_flow_ms,loop_us
// =====================================================
void printProcessing() {
  Serial.printf("SIM,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
    tele.roll, tele.pitch, tele.yaw, tele.alt,
    tele.pos_x, tele.pos_y, tele.pwm_x, tele.pwm_y,
    tele.deltaX, tele.deltaY,
    tele.armed, tele.rc4_min,
    tele.targetHeight, tele.verticalSpeed, tele.V_real_x, tele.V_real_y,
    tele.dt_flow_ms, tele.loop_us);
}

// =====================================================
// Teleplot cho VSCode (tuy chon)
// =====================================================
void printTeleplot() {
#if EN_TELEPLOT
  // Alt Hold (Table 4.2)
  Serial.printf(">Z_target:%.2f\n",   tele.targetHeight);
  Serial.printf(">Z_filtered:%.2f\n", tele.alt);
  Serial.printf(">Z_error:%.2f\n",    tele.targetHeight - tele.alt);
  Serial.printf(">Vspeed:%.2f\n",     tele.verticalSpeed);
  // Pos Hold (Table 4.3)
  Serial.printf(">pos_x:%.2f\n",      tele.pos_x);
  Serial.printf(">pos_y:%.2f\n",      tele.pos_y);
  Serial.printf(">Vx:%.2f\n",         tele.V_real_x);
  Serial.printf(">Vy:%.2f\n",         tele.V_real_y);
  Serial.printf(">pwm_x:%.2f\n",      tele.pwm_x);
  Serial.printf(">pwm_y:%.2f\n",      tele.pwm_y);
  // Chan doan
  Serial.printf(">rc4_min:%d\n",      tele.rc4_min);
  Serial.printf(">armed:%d\n",        tele.armed);
  Serial.printf(">dt_flow_ms:%d\n",   tele.dt_flow_ms);
  Serial.printf(">loop_us:%d\n",      tele.loop_us);
#endif
}
