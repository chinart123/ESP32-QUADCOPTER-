#ifndef PROCESSING_H
#define PROCESSING_H

#include "ESPNow_Receiver.h"  // de thay struct TelemetryPacket + bien 'tele'

/*
 * Processing - Xuat du lieu telemetry ra Serial.
 *   printProcessing(): LUON in dong "SIM,..." -> Processing doc (digital twin 3D).
 *   printTeleplot()  : khoi ">ten:gia_tri" cho VSCode Teleplot (chi khi EN_TELEPLOT=1).
 *   Hai cai song song, khong xung dot (Teleplot dong '>', Processing dong 'SIM,').
 *   Tat Teleplot = doi EN_TELEPLOT thanh 0.
 */
#define EN_TELEPLOT 1   // 1 = in them Teleplot cho VSCode | 0 = tat

void printProcessing();
void printTeleplot();

#endif
