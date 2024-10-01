#include <Arduino.h>
#include <Adafruit_SPIFlash.h>



// #include "flash_config"
#include "hardware_pins.h"

Adafruit_FlashTransport_QSPI flashTransport(QSPI_SCK, QSPI_CS, QSPI_D0,
                                   QSPI_D1, QSPI_D2, QSPI_D3);
Adafruit_SPIFlash Flash_QSPI(&flashTransport);

#define P PB08

void setup() {

  delay(3000);

  Serial.begin(115200);
  Serial.println("Iniciado.");

  pinMode(PB08, OUTPUT);
  // pinMode(QSPI_CS, OUTPUT); // Pin con resistencia pullup
  // delay(500);
  // digitalWrite(QSPI_CS, LOW);

  // Inicializar el flash QSPI
  flashTransport.begin();
  if (flashTransport.supportQuadMode()) {
    Serial.println("Soporta QSPI");
  } else {
      Serial.println("No soporta QSPI");
  }
  
  // Flash_QSPI.begin();
  while (Flash_QSPI.begin() == false) {
    Serial.println("Error al inicializar la memoria");
    delay(1000);  // Detener si falla la inicialización
  }
  if (Flash_QSPI.isReady()) {
    Serial.println("La mem flash Está lista");
    Serial.print("ID JEDEC de la memoria flash: 0x");
    Serial.println(Flash_QSPI.getJEDECID(), HEX);  // Mostrar el ID en formato hexadecimal
  }

  Serial.println("Memoria flash_QSPI inicializada correctamente");

}

void loop() {  
  // digitalWrite(QSPI_CS, LOW);
  digitalWrite(PB08, LOW);
  delay(1000);
  digitalWrite(PB08, HIGH);
  // digitalWrite(QSPI_CS, HIGH);
  delay(1000);
}
