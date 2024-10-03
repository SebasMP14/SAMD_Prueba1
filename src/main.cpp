#include <Arduino.h>
#include <Adafruit_SPIFlash.h>

#include "hardware_pins.h"

Adafruit_FlashTransport_QSPI flashTransport(QSPI_SCK, QSPI_CS, QSPI_D0, QSPI_D1, QSPI_D2, QSPI_D3);
Adafruit_SPIFlash Flash_QSPI(&flashTransport);

#define P PB08

#define PAGE_SIZE 256                           // De la memoria flash
#define SECTOR_SIZE 4096
#define EXT_FLASH_SIZE (16UL * 1024 * 1024)         // 16 MB o 128 Mb
#define LAST_PAGE_ADDR (EXT_FLASH_SIZE - PAGE_SIZE) // Contiene la última posición escrita

void setup() {

  delay(3000);

  Serial.begin(115200);
  Serial.println("Iniciado.");

  pinMode(PB08, OUTPUT);

  // Inicializar el flash QSPI
  flashTransport.begin();
  if (flashTransport.supportQuadMode()) {
    Serial.println("Soporta QSPI");
  } else {
    Serial.println("No soporta QSPI");
  }
  
  while (Flash_QSPI.begin() == false) {
    Serial.println("Error al inicializar la memoria");
    delay(1000);
  }
  if (Flash_QSPI.isReady()) {
    Serial.println("La mem flash Está lista");
    Serial.print("ID JEDEC de la memoria flash: 0x");
    Serial.println(Flash_QSPI.getJEDECID(), HEX);  
  }

  Serial.println("Memoria flash_QSPI inicializada correctamente");

  uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF};  // Datos de ejemplo
  // uint32_t address = 0x00000001;  // Dirección de la memoria para escribir (inicio)

  uint32_t last_write_address = 0;
  Flash_QSPI.readBuffer(LAST_PAGE_ADDR, (uint8_t*)&last_write_address, sizeof(last_write_address));

  Serial.print("Última posición escrita leída de la flash: 0x");
  Serial.println(last_write_address, HEX);

  if (last_write_address == 0xFFFFFFFF) { // si no hay una dir guardada
    last_write_address = 0x00000000;
    // Flash_QSPI.eraseChip();
  }
  uint32_t new_write_address = 0;
  if (last_write_address == 0x00000000) {
    new_write_address = last_write_address;
  } else {
    new_write_address = last_write_address + 0x01;
    if (new_write_address >= EXT_FLASH_SIZE - PAGE_SIZE) {
      Serial.println("Sin espacio disponible");
    }
  }

  // // Escribir datos en la memoria
  // Flash_QSPI.eraseSector(address);  // Borrar antes de escribir
  // if (Flash_QSPI.writeBuffer(address, data, sizeof(data))) {
  //   Serial.println("Datos escritos correctamente.");
  // } else {
  //   Serial.println("Error al escribir en la memoria.");
  // }

  if (Flash_QSPI.writeBuffer(new_write_address, data, sizeof(data))) {
    Serial.println("Datos escritos correctamente.");
  } else {
    Serial.println("Error al escribir en la memoria.");
  }

  // Actualizar la dirección de la última posición de escritura en la última página de la memoria
  Flash_QSPI.erasePage(LAST_PAGE_ADDR);  // Borrar la última página antes de escribir
  if (Flash_QSPI.writeBuffer(LAST_PAGE_ADDR, (uint8_t*)&new_write_address, sizeof(new_write_address))) {
    Serial.print("Nueva posición de escritura guardada: 0x");
    Serial.println(new_write_address, HEX);
  } else {
    Serial.println("Error al actualizar la dirección en la memoria.");
  }

  // Leer los datos escritos para verificar
  // uint8_t read_data[sizeof(data)];
  // if (Flash_QSPI.readBuffer(address, read_data, sizeof(read_data))) {
  //   Serial.print("Datos leídos: ");
  //   for (uint8_t i = 0; i < sizeof(read_data); i++) {
  //     Serial.print("0x");
  //     Serial.print(read_data[i], HEX);
  //     Serial.print(" ");
  //   }
  //   Serial.println();
  // } else {
  //   Serial.println("Error al leer de la memoria.");
  // }

  uint8_t read_data = 0;
  Serial.print("Leyendo memoria: ");
  for (uint32_t address = 0x01; address < (new_write_address + sizeof(data)); address++) {
    if (Flash_QSPI.readBuffer(address, &read_data, sizeof(read_data))) {
      Serial.print("0x");
      Serial.print(read_data, HEX);
      Serial.print(" ");
    } else {
      Serial.println("Error al leer de la memoria.");
    }
  }
  Serial.println();

}

void loop() {  
  digitalWrite(PB08, LOW);
  delay(1000);
  digitalWrite(PB08, HIGH);
  delay(1000);
}
