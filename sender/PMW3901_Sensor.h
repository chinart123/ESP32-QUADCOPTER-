#ifndef PMW3901_SENSOR_H
#define PMW3901_SENSOR_H

#include "Config.h"

/*
 * PMW3901_Sensor - Khoi cam bien optical flow + Pos-Hold PID.
 *   setupPMW()  : khoi tao SPI + PMW3901 (goi trong setup()).
 *   updatePMW() : doc flow, bu gyro, doi pixel->cm, tich phan vi tri,
 *                 chay PID giu vi tri -> shared_pwm_opt_x/y.
 *                 Goi trong NavigationCore1Task (tu chot nhip 10ms ben trong).
 */
void setupPMW();
void updatePMW();

#endif
