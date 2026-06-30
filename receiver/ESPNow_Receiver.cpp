#include "ESPNow_Receiver.h"
#include "Processing.h"

TelemetryPacket tele;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(tele)) return;
  memcpy(&tele, incomingData, sizeof(tele));

  printProcessing();   // ---- LUON in cho Processing (digital twin) ----
  printTeleplot();     // ---- Teleplot cho VSCode (chi in khi EN_TELEPLOT=1) ----
}

void setupEspNowReceiver() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW INIT ERROR!"); return; }
  esp_now_register_recv_cb(OnDataRecv);
}
