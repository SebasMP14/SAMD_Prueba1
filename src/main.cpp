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
  // Serial1.begin(115200);
  // Serial2.begin(115200);

  pinMode(SPI_CS_MAX, OUTPUT);
  digitalWrite(SPI_CS_MAX, HIGH);

  Serial.println("SPI begin");
  start_max1932();

  Serial.println("Setup finalizado...");
}

void loop() {
  // digitalWrite(P, LOW);
  // delay(1000);
  // digitalWrite(P, HIGH);
  // delay(1000);
  
  delay(1000);
  // Serial1.write(0x0A); // OBC reading
  // Serial1.write(0xDA);

  // if ( Serial1.available() ) { // MUA receiving
  //   Serial.print("Serial1: 0x");
  //   Serial.println(Serial1.read(), HEX);
  // }

  // if ( Serial2.available() ) { // MUA receiving
  //   Serial.print("Serial2: 0x");
  //   Serial.println(Serial2.read(), HEX);
  // }
}




  // pinMode(SPI_CS_MAX, OUTPUT);
  // digitalWrite(SPI_CS_MAX, HIGH);

  // Serial.println("SPI begin");
  // start_max1932();

  // if ( !write_max_reg(0xC8) ) {
  //   Serial.println("Comando max incorrecto");
  // }

  // delay(5000);

  // if ( !write_max_reg(0x64) ) {
  //   Serial.println("Comando max incorrecto");
  // }

  // delay(5000);

  // if ( !write_max_reg(0x05) ) {
  //   Serial.println("Comando max incorrecto");
  // }
