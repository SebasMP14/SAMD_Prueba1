/**
 * max1932_driver.cpp
 * Funciones de control del max1932, comunicación por SPI 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Est. Sebas Monje <2024> (github)
 * 
 * TODO:
 * - Agregar en "write" función con respecto a la tensión deseada
 */
#include "max1932_driver.h"

/************************************************************************************************************
 * @fn      start_max1932
 * @brief   Inicializa el puerto SPI
 * @param   NONE
 * @return  NONE
 */
void start_max1932(void) {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
}

/************************************************************************************************************
 * @fn      write_max_reg
 * @brief   Envía el comando de tensión
 * @param   command: valores de cero a 255, siendo 255 la instrucción que genera la menor tensión de salida
 *                   en el max1932 y 1 el mayor valor, 0 apaga el convertidor
 * @return  true exitoso - false fallido
 */
bool write_max_reg(uint8_t command) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // 2 MHz, LSBFirst, SPI_MODE
  #ifdef DEBUG_MODE
  Serial.print("0x");
  Serial.println(command, HEX);
  #endif

  digitalWrite(SPI_CS_MAX, LOW);  // selección
  delayMicroseconds(1);
  SPI.transfer(command);     // Envio de comando
  delayMicroseconds(1);
  digitalWrite(SPI_CS_MAX, HIGH);
  delayMicroseconds(1);

  SPI.endTransaction();

  return true;
}