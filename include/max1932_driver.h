/**
 * max1932_driver.h
 * Funciones de control del max1932, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Agregar en "write" función con respecto a la tensión deseada
 */
#ifndef MAX1932_DRIVER_H
#define MAX1932_DRIVER_H

#include <SPI.h>
#include "hardware_pins.h"

#define DEBUG_MODE

void start_max1932(void);
bool write_max_reg(uint8_t command);

#endif