/**
 * dac8551_driver.cpp
 * Funciones de control del dac8551, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2025> (github)
 * 
 * TODO:
 * - 
 */

#include "dac8551_driver.h"
#include <Arduino.h>

void start_dac8551(void) {
  pinMode(PIN_SPI_SS, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH);
  SPI.begin();
}

void write_dac8551_reg(uint16_t command) {
  SPI.beginTransaction(SPISettings(DAC_CLK_SPEED, MSBFIRST, SPI_MODE1)); 
  digitalWrite(SPI_CS_MAX1, LOW);  // selección
  SPI.transfer(0x00);
  SPI.transfer16(command);
  digitalWrite(SPI_CS_MAX1, HIGH);
  SPI.endTransaction();
}

void end_dac8551(void) {
  SPI.beginTransaction(SPISettings(DAC_CLK_SPEED, MSBFIRST, SPI_MODE1)); 
  digitalWrite(SPI_CS_MAX1, LOW);  // selección
  SPI.transfer(0x11);
  SPI.transfer16(0x0000);
  digitalWrite(SPI_CS_MAX1, HIGH);
  SPI.endTransaction();
}