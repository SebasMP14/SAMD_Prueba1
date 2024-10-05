#ifndef TMP100_DRIVER_H
#define TMP100_DRIVER_H

#define DEBUG_MODE

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define TMP100_ADDRESS 0b01001001 /* Para ADD0 flotante */
#define resolution 12 // 0.0625 ºC de resolución "equivale" a 1 mV de presición para polarización

bool start_tmp100(void);
float read_tmp100(void);

#endif