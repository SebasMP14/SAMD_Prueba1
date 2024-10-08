/**
 * main.cpp
 * Este código es una prueba del control del MAX1932 por SPI
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

#include <Arduino.h>
#include <SPI.h>

#include "max1932_driver.h"
#include "hardware_pins.h"

#define DEBUG_MODE

#define P PB08

void setup() {
  delay(4000);

  Serial.begin(115200);

  pinMode(SPI_CS_MAX, OUTPUT);
  digitalWrite(SPI_CS_MAX, HIGH);

  Serial.println("SPI begin");
  start_max1932();

  if ( !write_max_reg(0xFF) ) {
    Serial.println("Comando max incorrecto");
  }

  delay(5000);

  if ( !write_max_reg(0xC8) ) {
    Serial.println("Comando max incorrecto");
  }

  delay(5000);

  if ( !write_max_reg(0x64) ) {
    Serial.println("Comando max incorrecto");
  }

  delay(5000);

  if ( !write_max_reg(0x05) ) {
    Serial.println("Comando max incorrecto");
  }

  Serial.println("Setup finalizado...");
}

void loop() {  
  digitalWrite(P, LOW);
  delay(1000);
  digitalWrite(P, HIGH);
  delay(1000);

  // for (uint8_t i = 0; i < 256; i++) { 
  //   SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 2 MHz, LSBFirst, SPI_MODE
  //   digitalWrite(SPI_CS_MAX, LOW);
  //   SPI.transfer(i);
  //   digitalWrite(SPI_CS_MAX, HIGH);
  //   SPI.endTransaction();
  //   delay(100);
  // }

}



/* NOTAS: Potenciometro digital I2C
#define MCP4561 0b00101110
 escribirPotenciometro(128);
int escribirPotenciometro(uint16_t valor)
{
  if(valor>256) {valor=256;}
  Wire.beginTransmission(int(MCP4561)); //Dirección del MCP4561
  byte DH = valor/256;
  byte D1 = 0b00000000 or DH;
  byte D2 = byte(valor);
  Serial.println(D1,BIN);
  Serial.println(D2,BIN);
  Wire.write(D1);
  Wire.write(D2);
  Wire.endTransmission();
  return 0;
}



*/