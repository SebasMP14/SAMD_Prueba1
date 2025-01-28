/**
 * tmp100_driver.h
 * Funciones de configuración y lectura del sensor de temperatura TMP100 para la misión M.U.A. de FIUNA 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - 
 */
#ifndef TMP100_DRIVER_H
#define TMP100_DRIVER_H

#define DEBUG_TMP

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// #define TMP100_ADDRESS 0b01001001 /* Para ADD0 flotante */
#define TMP100_ADDRESS 0b01001000 /* Para ADD0  */
#define resolution 12 // 0.0625 ºC de resolución "equivale" a 1 mV de presición para polarización

bool start_tmp100(void);
float read_tmp100(void);

#endif