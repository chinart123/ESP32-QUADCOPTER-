#ifndef ESPNOW_RECEIVER_H
#define ESPNOW_RECEIVER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

/*
 * ESPNow_Receiver - Ground station nhan goi ESP-NOW tu drone.
 *   - Dinh nghia struct TelemetryPacket (PHAI KHOP 100% voi ben sender).
 *   - setupEspNowReceiver(): init ESP-NOW + dang ky callback nhan.
 *   - Callback OnDataRecv (trong .cpp) goi printProcessing()/printTeleplot().
 */

// Struct PHAI KHOP 100% voi ben drone (sender)
typedef struct {
  float roll, pitch, yaw;
  float alt;
  float targetHeight;
  float verticalSpeed;
  float pos_x, pos_y;
  float V_real_x, V_real_y;
  float pwm_x, pwm_y;
  int16_t deltaX, deltaY;
  int16_t rc4_min;
  uint8_t armed;
  int16_t dt_flow_ms;   // CHAN DOAN: nhip doc optical flow (ms)
  int16_t loop_us;      // CHAN DOAN: chu ky vong Core 0 (us)
} TelemetryPacket;

extern TelemetryPacket tele;

void setupEspNowReceiver();

#endif
