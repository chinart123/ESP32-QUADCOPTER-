/*
 * GROUND STATION RECEIVER - ESP32
 * Ban MODULE HOA: ESPNow_Receiver (nhan goi) + Processing (xuat Serial).
 *   receiver.ino chi giu setup()/loop().
 */

#include "ESPNow_Receiver.h"
#include "Processing.h"

void setup() {
  Serial.begin(921600);
  setupEspNowReceiver();
}

void loop() {
  delay(1);
}
