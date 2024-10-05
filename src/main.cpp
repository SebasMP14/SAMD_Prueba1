/**
 * main.cpp
 * Este codigo es una prueba del sensor de temperatura TMP100, protocolo I2C
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

#include <Arduino.h>

#include "hardware_pins.h"
#include "tmp100_driver.h"

#define DEBUG_MODE

#define P PB08

uint8_t status = 0;
float temperature = 0.0;

void setup() {
  delay(4000);

  Serial.begin(115200);

  // Configuración del TMP100
  if ( !start_tmp100() ) {
    #ifdef DEBUG_MODE
    Serial.println("Inicialización de TMP100 fallida");
    #endif
  }

  
  Serial.println("Setup finalizado...");
}

void loop() {  
  digitalWrite(PB08, LOW);
  delay(1000);
  digitalWrite(PB08, HIGH);
  temperature = read_tmp100();
  if (isnan(temperature)) {
    #ifdef DEBUG_MODE
    Serial.println("Error al leer la temperatura.");
    #endif
  } else {
    Serial.print("TMP: ");
    Serial.print(temperature);
    Serial.println(" ºC");
  }

  delay(1000);  // Leer cada segundo
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