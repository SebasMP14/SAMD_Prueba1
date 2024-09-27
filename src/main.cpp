#include <Arduino.h>
#include <Adafruit_SPIFlash.h>
#include <Adafruit_FlashTransport_QSPI.h>

#include "hardware_pins.h"

Adafruit_FlashTransport_QSPI Flash_QSPI;
// Adafruit_SPIFlash flash(&Flash_QSPI);

#define P PB08

void setup() {

  delay(4000);

  Serial.begin(115200);
  Serial.println("Iniciado.");


  pinMode(P, OUTPUT);


}

void loop() {  

}
