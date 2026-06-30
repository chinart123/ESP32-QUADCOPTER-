#ifndef ESPNOW_TELEMETRY_H
#define ESPNOW_TELEMETRY_H

#include "Config.h"

/*
 * ESPNow_Telemetry - Khoi truyen thong khong day (Core 0).
 *   setupEspNow()  : init WiFi STA + ESP-NOW + add peer (goi trong setup()).
 *   TelemetryTask(): gop trang thai vao 'tele' va gui 10Hz.
 *                    Chay rieng tren Core 0 (tranh nhieu Wi-Fi vao vong bay Core 1).
 */
void setupEspNow();
void TelemetryTask(void *pvParameters);

#endif
