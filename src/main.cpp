/**
 * main.cpp
 * Este codigo es una prueba de la memoria FLASH
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * - Lucas Cho
 * 
 * TODO:
 * 
 * 
 */

#include <Arduino.h>

#include "hardware_pins.h"
#include "flash_driver.h"

#define DEBUG_MODE

#define P PB08

void setup() {
  delay(4000);

  uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};  /* Datos de ejemplo de escritura */
  bool flash_status = true;

  #ifdef DEBUG_MODE
  Serial.begin(115200);
  Serial.println("DEBUG (setup): Puerto serial iniciado.");
  #endif

  pinMode(PB08, OUTPUT);

  /* Inicializar el flash QSPI */
  /* TODO: error handling */
  flash_status = start_flash();
  if ( flash_status == true ) {
    #ifdef DEBUG_MODE
    Serial.println("Comunicar al OBC que el flash no conecta.");
    #endif
    flash_status = true;
  }

  flash_status = write_mem((uint8_t *)&data, sizeof(data));
  if ( flash_status == true ) {
    #ifdef DEBUG_MODE
    Serial.println("Fallo en la escritura.");
    #endif
    flash_status = true;
  }

  read_all();

  // erase_all();

  Serial.println("FIN del proceso...");
}

void loop() {  
  digitalWrite(PB08, LOW);
  delay(1000);
  digitalWrite(PB08, HIGH);
  delay(1000);
}
