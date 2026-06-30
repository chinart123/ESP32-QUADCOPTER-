#ifndef VL53L1X_SENSOR_H
#define VL53L1X_SENSOR_H

#include "Config.h"

/*
 * VL53L1X_Sensor - Khoi cam bien khoang cach (ToF) + Alt-Hold PID.
 *   setupToF()  : khoi tao I2C bus 0 + VL53L1X (goi trong setup()).
 *   updateToF() : doc ToF, bu nghieng, loc, tinh verticalSpeed
 *                 va chay PID giu do cao -> shared_altitudeCorrection.
 *                 Goi trong NavigationCore1Task (chu ky 4ms).
 */
void setupToF();
void updateToF();

#endif
