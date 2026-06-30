#include "ESPNow_Telemetry.h"

// =====================================================
// SETUP ESP-NOW (goi trong setup() - Core 1)
// =====================================================
void setupEspNow() {
#if EN_ESPNOW
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(34); // 8.5 dBm

  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW INIT FAIL"); while(1); }
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0; peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("PEER ADD FAIL"); while(1); }
#endif
}

// =====================================================
// CORE 0: TELEMETRY TASK (WI-FI ONLY)
// Chi doc bien va gui di. Core 0 danh rieng cho song vo tuyen.
// =====================================================
void TelemetryTask(void * pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS; // 10Hz

  for(;;) {
#if EN_ESPNOW
    tele.roll = fRoll;   tele.pitch = fPitch;  tele.yaw = fYaw;   tele.alt = filteredHeight;
    tele.targetHeight = targetHeight;          tele.verticalSpeed = verticalSpeed;
    tele.pos_x = pos_x;  tele.pos_y = pos_y;
    tele.V_real_x = V_real_x; tele.V_real_y = V_real_y;
    tele.pwm_x = shared_pwm_opt_x; tele.pwm_y = shared_pwm_opt_y;
    tele.deltaX = deltaX; tele.deltaY = deltaY;
    tele.v_bat = filtered_voltage; tele.armed = isArmed ? 1 : 0;

    esp_now_send(receiverAddress, (uint8_t *)&tele, sizeof(tele));
    rc4_min = 2000;
#endif
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
