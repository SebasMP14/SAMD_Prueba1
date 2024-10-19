/**
 * main.cpp
 * Este código es una prueba de la comunicación serial entre DOS SAMD51
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

#include <Arduino.h>
// #include <SPI.h>


// #include "max1932_driver.h"
#include "hardware_pins.h"

#define DEBUG_MAIN
#define RTC_GCLK_TD 0
#define P PA20

uint32_t date;

void setup() {
  delay(4000);

  Serial.begin(115200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(P, OUTPUT);

  Serial.println("Setup Finalizado");
}

void loop() {
  digitalWrite(P, HIGH);
  delay(500);
  digitalWrite(P, LOW);
  delay(500);

  if ( Serial1.available() > 0 ) {
    uint8_t read1;
    Serial1.readBytes(&read1, 1);
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> Recibido Serial1: 0x");
    Serial.println(read1, HEX);
    #endif
    if ( read1 == 0xAA ) {
      Serial1.write(0xA1); // COUNT_MODE
    }
  }

  delay(1000);
  
  if ( Serial2.available() > 0 ) {
    uint8_t read2;
    Serial2.readBytes(&read2, 1);
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> Recibido Serial2: 0x");
    Serial.println(read2, HEX); 
    #endif
    if ( read2 == 0xBB ) { // REQUEST_TIMESTAMP
      date = millis() / 1000;
      Serial2.write((uint8_t *)&date, sizeof(date));
    }
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> timestamp enviado: ");
    Serial.println(date);
    #endif
  }
}

  // Serial1.begin(115200);
  // Serial2.begin(115200);

  // pinMode(SPI_CS_MAX, OUTPUT);
  // digitalWrite(SPI_CS_MAX, HIGH);

  // Serial.println("SPI begin");
  // start_max1932();

  /////////////////////////////////////

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

//////////////////////////////////////
  // digitalWrite(P, LOW);
  // delay(1000);
  // digitalWrite(P, HIGH);
  // delay(1000);
  
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