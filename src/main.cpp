/**
 * main.cpp
 * Este código es una prueba de la comunicación OBC - MUA
 * Este sistema simula al OBC, mientras MUA prueba la FSM desarrollada
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024> (github)
 * 
 * TODO:
 * 
 */

//// Obtencion de puntos de la curva IV ////
////////////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include <SPI.h>

#include "hardware_pins.h"
#include "ads1260_driver.h"
#include "max1932_driver.h"
#include "dac8551_driver.h"
#include "tmp100_driver.h"
#include "calculos.h"

#define DEBUG_MAIN
#define MAX_ITER 10
#define Elementos 400                     // Cantidad de muestras
#define OverVoltage 1                     // Sobrevoltaje aplicado para la polarización de los SiPMs
#define Switching_Time_MAX 4              // Microseconds

float inverseVoltage[Elementos];          // Tensión inversa aplicada al SiPM para obtener "inverseCurrent_I"
uint16_t inverseVoltage_command[Elementos];
float inverseVCurrent[Elementos];        // Corriente inversa, convertida de "inverseCurrent_V[]"
float Filtered_voltage[Elementos];
float Filtered_current[Elementos];
float temperatureArray [Elementos];

ADS1260 ads1260(&SPI1, SPI_CS_ADC);

unsigned long time_ini;
float temperature = 0.0;
float firstCurrent = 0.0;

void obtain_Curve_inverseVI(float Temperature);

void setup() {
  delay(5000);

  Serial.begin(115200);
  Serial.println("Serial iniciado");
  
  pinMode(SPI_CS_DAC2, OUTPUT);
  pinMode(SPI_CS_MAX2, OUTPUT);
  pinMode(INTERFACE_EN, OUTPUT); // 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_SiPM1, OUTPUT);
  pinMode(LED_SiPM2, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_SiPM1, LOW);
  digitalWrite(LED_SiPM2, LOW);
  digitalWrite(INTERFACE_EN, HIGH); //
  delay(START_UP_TIME_ADS);                                       // Habilitación del ADC
  delay(1000);

  SPI.begin();                                                    // Descomentado en start_dac8551 y start_max1932
  Serial.println("SPI iniciado");
  start_dac8551(SPI_CS_DAC2);                                     // Solo el canal N
  digitalWrite(SPI_CS_DAC1, HIGH);
  start_max1932(SPI_CS_MAX2);
  digitalWrite(SPI_CS_MAX1, HIGH);
  Serial.println("CSs en HIGH");
  if ( start_tmp100() ) {
    Serial.println("TMP iniciado");
  }
  Serial.println((float)read_tmp100(), 4);
  

  // ADC Configuration
  ads1260.begin();
  Serial.print("MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   // 00100100
  ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100);           // 40 KSPS - FIR (Page 30)
  // ads1260.writeRegisterData(ADS1260_MODE0, 0b01101100);           // 14400 SPS
  delay(50);
  Serial.print("MODE0: ");
  Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   
  Serial.print("PGA: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
  ads1260.writeRegisterData(ADS1260_PGA, 0b10000000);             // BYPASS MODE
  Serial.print("PGA BYPASS MODE: ");
  Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
  ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000);           // STATENB
  // ads1260.writeRegisterData(ADS1260_REF, 0b00010000);             // REF 2.498V ENABLE
  
  delay(500);
  // Inicializar Vout1
  write_dac8551_reg(0x7FFF, SPI_CS_DAC2);
  write_max_reg(0x01, SPI_CS_MAX2);

  time_ini = millis();
  Serial.println("Setup finalizado");
  // while(1);
}

void loop() {
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  if ( millis() - time_ini >= 3000 ) {
    Serial.println("Datos obtenidos: ");
    Serial.print("Temperatura: ");

    temperature = read_tmp100();
    Serial.println(temperature, 4);
    Serial.println("Obtención de curva ");
    // delay(5000);
    obtain_Curve_inverseVI(temperature);

    time_ini = millis();
    Serial.println("i, Voltage, VCorriente, (Voltage*12)-3.8, (VCorriente-firstCurrent)/2000");
    for ( uint16_t i = 1; i < Elementos; i++ ) {
      Serial.print(i);
      Serial.print(",");
      Serial.print(inverseVoltage[i], 7);
      Serial.print(",");
      Serial.print(inverseVCurrent[i], 7);
      Serial.print(",");
      Serial.print(temperatureArray[i]);
      Serial.print(",");
      Serial.print((inverseVoltage[i]*12)-3.8, 7);
      Serial.print(",");
      Serial.println((inverseVCurrent[i]-firstCurrent)/2000, 10);
    }
    // Serial.print("Breakdown Voltage sin filtrar: ");
    // Serial.println((obtain_Vbd(inverseVCurrent, inverseVoltage, Elementos) * 12) - 3.8, 6);
    sliding_moving_average(inverseVoltage, Elementos, 5, Filtered_voltage);
    sliding_moving_average(inverseVCurrent, Elementos, 5, Filtered_current);
    Serial.print("Voltage y Corriente Filtrados empieza en 5 seg");
    delay(5000);
    for ( uint16_t i = 0; i < Elementos; i++ ) {
      Serial.print(i);
      Serial.println(",");
      Serial.print(Filtered_voltage[i], 6);
      Serial.print(",");
      Serial.println(Filtered_current[i], 6);
    }
    // Serial.print("Breakdown Voltage Filtrado: ");
    // Serial.println((obtain_Vbd(Filtered_current, Filtered_voltage, Elementos) * 12) - 3.8, 6);
  }
  
}

/************************************************************************************************************
 * @fn      obtain_Curve_inverseVI
 * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
 *          ADC.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @return  ---todo
 * TODO: - Se necesita un algoritmo para setear el Vbias correctamente
 * 
 */
void obtain_Curve_inverseVI(float Temperature) {
  float Vbd_Teo = Vbd_teorical(Temperature);
  #ifdef DEBUG_MAIN
  Serial.print("Vbd teorico: ");
  Serial.println(Vbd_Teo);
  #endif
  // float Vlimite_inferior = max(24.588, Vbd_Teo - 2);      // 24.588 es el minimo valor a la salida del MAX
  // float Vlimite_superior = min(36, Vbd_Teo + 2);      // 36 es el máximo valor a la salida del MAX
  uint16_t Vlim_sup = 0x3000;//VDAC_command(Vlimite_inferior); // Comandos 
  uint16_t Vlim_inf = 0x7FFF;//VDAC_command(Vlimite_superior); // Comandos 
  uint16_t paso = (uint16_t)(Vlim_inf - Vlim_sup) / Elementos;
  unsigned long start_time, total_time;

  #ifdef DEBUG_MAIN
  Serial.print("Limites (dec, hex): ");
  Serial.print(out_voltage(Vlim_inf), 2);
  Serial.print(", ");
  Serial.print(out_voltage(Vlim_sup), 2);
  Serial.print(", ");
  Serial.print(Vlim_inf, HEX);
  Serial.print(", ");
  Serial.print(Vlim_sup, HEX);
  Serial.print(", paso: ");
  Serial.println(paso);
  #endif

  for ( uint16_t i = 0; i < Elementos; i++ ) {          // Comandos para el DAC
    inverseVoltage_command[i] = Vlim_inf - i * paso;
  }
  Serial.println("Comandos calculados");
  start_time = micros();
  for ( uint16_t i = 0; i < Elementos; i++ ) {
    // if (i % 100 == 0 && i != 0) {
    //   Serial.println("DEBUG (obtain_Curve_inverseVI) -> waiting");
    //   // delay(5000);
    // }
    write_dac8551_reg(inverseVoltage_command[i], SPI_CS_DAC2);
    // delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    delay(10); // Settling time of the MAX
    // Aquí se debe validar la tensión seteada en la salida del max con el ADC
    inverseVoltage[i] = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN1, ADS1260_MUXN_AINCOM));
    inverseVCurrent[i] = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM));
    temperatureArray[i] = read_tmp100();
    // uint32_t aux = 0x00;
    // ads1260.connectMUX(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM);
    // for ( uint8_t j = 0; j < 10; j++ ) {
    //   aux += ads1260.readConversion();
    //   delayMicroseconds(100);
    // }
    // inverseVoltage[i] = ads1260.computeVolts((uint32_t)(aux/10));
    // aux = 0x00;
    // ads1260.connectMUX(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM);
    // for ( uint8_t j = 0; j < 10; j++ ) {
    //   aux += ads1260.readConversion();
    //   delayMicroseconds(100);
    // }
    // inverseVCurrent[i] = ads1260.computeVolts((uint32_t)(aux/10));
  }
  total_time = micros() - start_time;
  firstCurrent = inverseVCurrent[0];
  Serial.print("First VCurrent: ");
  Serial.println(firstCurrent, 7);
  Serial.print("Tiempo promedio de muestreo [ms]: ");
  float Ts = (float)total_time / (Elementos * 1000);
  Serial.println(Ts, 3);
}


//////// Pruebas ADS1260
////////////////////////////////////////////////////////////////////
// #include <Arduino.h>
// #include "ads1260_driver.h"
// #include "hardware_pins.h"
// #include <SPI.h>

// ADS1260 ads1260(&SPI1, SPI_CS_ADC);

// void setup() {
//   delay(4000);

//   Serial.begin(115200);
//   Serial.println("PLACA FINAL V1...");

//   pinMode(INTERFACE_EN, OUTPUT); // 
//   digitalWrite(INTERFACE_EN, HIGH); // 
//   delay(START_UP_TIME_ADS); // Habilitación del ADC

//   ads1260.begin();

//   pinMode(LED_BUILTIN, OUTPUT);
//   digitalWrite(LED_BUILTIN, LOW);

//   ads1260.noOperation();

//   Serial.println("Setup finalizado...");

//   Serial.print("ID: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_ID), BIN);      // 10100001
//   Serial.print("STATUS: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_STATUS), BIN);  // 00000101
//   Serial.print("MODE0: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   // 00100100
//   ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100); // 40 KSPS - FIR (Page 30)
//   delay(50);
//   Serial.print("MODE0: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   // 11111100
//   Serial.print("MODE1: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE1), BIN);   // 00000001
//   Serial.print("MODE2: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE2), BIN);   // 0
//   Serial.print("MODE3: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE3), BIN);   // 0
//   Serial.print("REF: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_REF), BIN);     // 00000101
//   Serial.print("PGA: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);     // 0
//   Serial.print("INPMUX: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_INPMUX), BIN);  // 11111111
//   Serial.print("INPBIAS: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_INPBIAS), BIN); // 0

//   ads1260.writeRegisterData(ADS1260_PGA, 0b10000000);
//   Serial.print("PGA: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);

//   uint32_t offsetCal = (ads1260.readRegisterData(ADS1260_OFCAL2) << 16) |
//                      (ads1260.readRegisterData(ADS1260_OFCAL1) << 8) |
//                       ads1260.readRegisterData(ADS1260_OFCAL0);

//   uint32_t fullScaleCal = (ads1260.readRegisterData(ADS1260_FSCAL2) << 16) |
//                           (ads1260.readRegisterData(ADS1260_FSCAL1) << 8) |
//                           ads1260.readRegisterData(ADS1260_FSCAL0);

//   Serial.print("Offset Calibration: 0x");
//   Serial.println(offsetCal, HEX);
//   Serial.print("Full-Scale Calibration: 0x");
//   Serial.println(fullScaleCal, HEX);

//   ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000); // STATENB

//   // ads1260.selfOffsetCalibration();
//   // offsetCal = (ads1260.readRegisterData(ADS1260_OFCAL2) << 16) |
//   //                    (ads1260.readRegisterData(ADS1260_OFCAL1) << 8) |
//   //                     ads1260.readRegisterData(ADS1260_OFCAL0);
//   // Serial.print("Offset Calibration: 0x");
//   // Serial.println(offsetCal, HEX);

//   ads1260.writeRegisterData(ADS1260_REF, 0b00010000);
// }

// void loop() {
//   delay(500);
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);

//   uint32_t value = ads1260.readData(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM);
//   int32_t signedValue = value;
//   if (signedValue & 0x800000) { // Verifica si el valor es negativo
//     signedValue |= 0xFF000000; // Extiende el signo
//   }
//   float converted1 = (signedValue * 2.498) / 8388607.0; // /-1+2^23
//   float converted2 = (value * 5.155) / 8388607.0;       // /-1+2^23
//   Serial.print("Valor hex, voltage: 0x");
//   Serial.print(value, HEX);
//   Serial.print(", ");
//   Serial.print(converted1, 7);
//   Serial.print(", ");
//   Serial.println(converted2, 7);

//   Serial.println(ads1260.readRegisterData(ADS1260_STATUS), BIN);
//   delay(1000);

// }

// #include <Arduino.h>

// #include "hardware_pins.h"
// #include "dac8551_driver.h"
// // #include "tmp100_driver.h"
// #include <SPI.h>

// #define DEBUG_MAIN

// void setup() {
//   delay(4000);

//   Serial.begin(115200);
//   Serial.println("Serial iniciado");

//   // start_dac8551();
//   pinMode(SPI_CS_ADC, OUTPUT);
//   digitalWrite(SPI_CS_ADC, HIGH);
//   SPI1.begin();
//   // start_tmp100();

//   // Serial.print("Temperatura: ");
//   // Serial.println(read_tmp100(), 4);

//   Serial.println("Setup finalizado");
// }

// void loop() {
//   delay(1000);
//   // Serial.print("Temperatura: ");
//   // Serial.println(read_tmp100(), 4);

//   for (uint8_t i = 0; i < 256; i++) {
//     // write_dac8551_reg(i*256);
//     SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
//     digitalWrite(SPI_CS_ADC, LOW);  // selección
//     SPI1.transfer(0x00);
//     SPI1.transfer16(i*256);
//     digitalWrite(SPI_CS_ADC, HIGH);
//     SPI1.endTransaction();
//     delay(80);
//     Serial.print(", ");
//     Serial.print(i);
//   }
//   Serial.println("end");
// }

/////////////////////////////////////////////////////////////////////////////////////////////////
/* Prueba de comunicación UART con RTC, el programa recibe los datos */

// #include <Arduino.h>

// #include "hardware_pins.h"
// #include "RTC_SAMD51.h"
// #include "DateTime.h"

// #define DEBUG_MAIN
// #define TRAMA_SIZE 44

// unsigned long date;
// bool state = false;
// bool state_recibir = false;
// uint32_t Time;
// uint8_t ID_COUNT_MODE         = 0x01;
// uint8_t ID_TRANSFER_DATA_MODE = 0x02;
// uint8_t ACK_OBC_TO_MUA        = 0x04;
// uint8_t ACK_MUA_TO_OBC        = 0x07; // ACK MUA to OBC

// RTC_SAMD51 rtc;

// uint16_t calcularCRC(const uint8_t *data, size_t length);
// uint16_t calcularCRC1(const uint8_t *data, size_t length);
// uint16_t mk_CRC(uint8_t *data, uint8_t data_number); // Para Probar del OBC

// void setup() {
//   delay(4000);

//   Serial.begin(115200);
//   Serial.println("Serial iniciado");
//   while ( !rtc.begin() ) {
//     delay(1000);
//     Serial.println("No rtc");
//   }
//   Serial.println("RTC iniciado.");
//   DateTime now = DateTime(F(__DATE__), F(__TIME__));
//   rtc.adjust(now);
//   Serial1.begin(115200);
//   Serial.println("Serial1 Iniciado");
//   Serial2.begin(115200);
//   Serial.println("Serial2 Iniciado");

//   DateTime currentTime = rtc.now();

//   Serial.print("Fecha: ");
//   Serial.print(currentTime.year());
//   Serial.print("-");
//   Serial.print(currentTime.month());
//   Serial.print("-");
//   Serial.print(currentTime.day());
//   Serial.print(" Hora: ");
//   Serial.print(currentTime.hour());
//   Serial.print(":");
//   Serial.print(currentTime.minute());
//   Serial.print(":");
//   Serial.println(currentTime.second());

//   Time = millis();

//   Serial.println("Setup Finalizado");

//   // uint8_t trama_prueba[30] = {0x4A, 0x47, 0x36, 0x59, 0x42, 0x57, 0x30, 0x4A, 0x47, 0x36,
//   //                            0x59, 0x50, 0x59, 0x30, 0x3E, 0xF0, 0xAA, 0x03, 0x12, 0x34, 
//   //                            0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78};
//   // uint16_t crc1 = calcularCRC(trama_prueba, 30);
//   // uint16_t crc2 = mk_CRC(trama_prueba, 30);

// }

// void loop() {
//   if ( millis() - Time > 5000 && !state ) { // Establecer COUNT MODE
//     uint8_t recibido;
    
//     Serial1.write(ID_COUNT_MODE);
//     while ( Serial1.available() > 1 );
    
//     Serial1.readBytes(&recibido, 1);
//     Serial.print("Recibido de Serial1: 0x");
//     Serial.println(recibido, HEX);
    
//     if ( recibido == ACK_MUA_TO_OBC ) {
//       state = true;
//       Serial.println("Estado (COUNT) establecido exitosamente");
//     } else {
//       Serial.print("Estado fallido");
//     }

//     Time = millis();
//   }
  
//   if ( Serial2.available() > 0 ) { // Para transmitir unixtime
//     uint8_t read2;
//     Serial2.readBytes(&read2, 1);

//     #ifdef DEBUG_MAIN
//     Serial.print("DEBUG (loop) -> Recibido Serial2: 0x");
//     Serial.println(read2, HEX);
//     #endif
    
//     if ( read2 == 0xBB ) { // REQUEST_TIMESTAMP
//       date = rtc.now().unixtime();
//       Serial2.write((uint8_t *)&date, sizeof(date));
//       #ifdef DEBUG_MAIN
//       Serial.print("DEBUG (loop) -> timestamp enviado: ");
//       Serial.println(date);
//       #endif
//     }
//   }

//   if ( millis() - Time > 10000 && !state ) {
//     Serial.print("Date: ");
//     Serial.print(rtc.now().year());
//     Serial.print("-");
//     Serial.print(rtc.now().month());
//     Serial.print("-");
//     Serial.print(rtc.now().day());
//     Serial.print(" Hora: ");
//     Serial.print(rtc.now().hour());
//     Serial.print(":");
//     Serial.print(rtc.now().minute());
//     Serial.print(":");
//     Serial.println(rtc.now().second());
//     Serial.print(", ");
//     Serial.print(rtc.now().unixtime());
//     Serial.print(", ");
//     Serial.println(rtc.now().unixtime(), HEX);
//     Time = millis();
//   }

//   if ( millis() - Time > 40000 && state ) { // Establecer TRANSFER MODE
//     Serial.println("Preparando para establecer TRANSFER MODE");
    
//     Serial1.write(ID_TRANSFER_DATA_MODE);

//     Serial.println("Estado enviado");
//     state_recibir = true;
//     // delay(100);
//     // if ( Serial1.available() ) {
//     //   state = true;
//     //   Serial.println("Estado (TRANSFER_DATA) establecido exitosamente");
//     //   state_recibir = true;
//     // } else {
//     //   Serial.print("Estado fallido");
//     // }

//     Time = millis();
//   }

//   if ( state_recibir ) {
//     uint8_t recibido[44];
//     uint16_t CRC;
//     Serial.println("Esperando datos");
//     delay(200);
//     while ( Serial1.available() < TRAMA_SIZE );

//     Serial1.readBytes(recibido, TRAMA_SIZE);
//     Serial.print("Recibido de Serial1 (DATOS)");
//     for (int i = 0; i < TRAMA_SIZE; i++) {
//       Serial.print("0x");
//       Serial.print(recibido[i], HEX);
//       Serial.print(" ");
//     }
//     Serial.println();

//     CRC = calcularCRC(recibido, 42);
//     uint16_t CRC_MUA = (recibido[42] << 8) | recibido[43];
//     if ( CRC == CRC_MUA && recibido[0] == 0x03) {
//       state = true;
//       Serial.println("Datos recibidos correctamente");
//       Serial1.write(ACK_OBC_TO_MUA);
//     } else {
//       Serial.println("error en la recepción");
//     }
//   }
//   delay(1000);
// }

// uint16_t calcularCRC(const uint8_t *data, size_t length) {
//   uint32_t crc = 0xFFFF;                // Valor inicial
//   uint32_t pol = 0x8408;                // Polinomio
//   uint8_t temp_data[length];
//   uint8_t temp;

//   memcpy(temp_data, data, length);

//   for ( uint8_t i = 0; i < length; i++ ) {
//     for ( uint8_t j = 0; j < 8; j++ ) {
//       temp = (crc ^ temp_data[i]) & 0x0001;
//       crc = crc >> 1;
//       if ( temp == 1 ) {
//         crc = crc ^ pol;
//       }
//       temp_data[i] = temp_data[i] >> 1;
//     }
//   }
  
//   return crc ^ 0xFFFF;
// }

// uint16_t calcularCRC1(const uint8_t *data, size_t length) {
//   uint16_t crc = 0xFFFF;                // Valor inicial
//   for (size_t i = 0; i < length; i++) {
//     crc ^= (data[i] << 8);
//     for (uint8_t bit = 0; bit < 8; bit++) {
//       if (crc & 0x8000) {
//         crc = (crc << 1) ^ 0x8408;      // Polinomio
//       } else {
//         crc <<= 1;
//       }
//     }
//   }
//   return crc ^ 0xFFFF;
// }

// // Función CRC basada en mk_CRC
// uint16_t mk_CRC(uint8_t *data, uint8_t data_number) {
//   uint32_t crcReg = 0xFFFF;
//   uint32_t calc = 0x8408;
//   uint8_t w;

//   // Crear una copia de los datos para trabajar con ellos sin modificar el original
//   uint8_t cal_data[data_number];
//   memcpy(cal_data, data, data_number);

//   for (int32_t k = 0; k < data_number; k++) {
//     for (int32_t i = 0; i < 8; i++) {
//       w = (crcReg ^ cal_data[k]) & 0x0001;
//       crcReg = crcReg >> 1;
//       if (w == 1) {
//         crcReg = crcReg ^ calc;
//       }
//       cal_data[k] = cal_data[k] >> 1; // Solo modificamos la copia
//     }
//   }
//   return crcReg ^ 0xFFFF;
// }

// for (uint8_t j = 0; j < data_number; j++) {
//   Serial.print(" 0x");
//   Serial.print(data[j], HEX);
//   Serial.print(",");
// }

//// Obtencion de puntos de la curva IV (Primeras pruebas) ////
//////////////////////////////////////////////////////////////////////////////////////////////////
// #include <Arduino.h>
// // #include <SPI.h>
// #include <Wire.h>
// #include <Adafruit_ADS1X15.h>

// #include "max1932_driver.h"
// #include "dac8551_driver.h"
// #include "tmp100_driver.h"
// #include "hardware_pins.h"
// #include "calculos.h"

// #define DEBUG_MAIN
// #define MAX_ITER 10
// #define Elementos 100                     // Cantidad de muestras
// #define OverVoltage 3                     // Sobrevoltaje aplicado para la polarización de los SiPMs
// #define Switching_Time_MAX 4              // Microseconds

// #define ADS_ADDRESS 0x48 //48
// #define P PA20

// float inverseVoltage[Elementos];          // Tensión inversa aplicada al SiPM para obtener "inverseCurrent_I"
// uint8_t inverseVoltage_command[Elementos];
// float inverseCurrent_I[Elementos];        // Corriente inversa, convertida de "inverseCurrent_V[]"

// Adafruit_ADS1115 ads;

// unsigned long time_ini;
// float temperature = 0.0;

// void obtain_Curve_inverseVI(float Temperature);

// void setup() {
//   delay(4000);

//   Serial.begin(115200);
//   Serial.println("Serial iniciado");

//   start_dac8551();
//   for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
//     if ( start_tmp100() ) { // Configuración del TMP100, HACER EN VARIOS INTENTOS
//       #ifdef DEBUG_MAIN
//       Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 exitosa.");
//       #endif
//       break;
//     } else {
//       #ifdef DEBUG_MAIN
//       Serial.print("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida: ");
//       Serial.println(iter_counter);
//       #endif
//       delay(10);
//     }
//   }
//   if ( ads.begin() ) { // ADS_ADDRESS, &Wire1
//     ads.setDataRate(RATE_ADS1115_860SPS);
//     Serial.print("ADS iniciado, DataRate: ");
//     Serial.println(ads.getDataRate());
//   }

//   time_ini = millis();
//   Serial.println("Setup finalizado");

//   analogWriteResolution(12);
//   pinMode(A0, OUTPUT);
//   Serial.println("i, VoltageT, VCorriente");
//   for ( int i = 0; i < 4095; i += 4 ) {
//     Serial.print(i);
//     Serial.print(",");
//     Serial.print((3.3/4095)*i, 4);
//     Serial.print(",");
//     analogWrite(A0, i);
//     delay(4);
//     Serial.println(ads.computeVolts(ads.readADC_SingleEnded(0)), 6);
//   }
//   while ( 1 );
// }

// void loop() {
//   if ( millis() - time_ini >= 5000 ) {
//     Serial.println("Datos obtenidos: ");
//     Serial.print("Temperatura: ");

//     temperature = 28.0;//read_tmp100();
//     // Serial.println(temperature, 4);
//     obtain_Curve_inverseVI(temperature);

//     time_ini = millis();
//     init_butterworth();
//     Serial.println("i, Voltage, Corriente, Voltage*10.97, Corriente/11000");
//     for ( uint8_t i = 0; i < Elementos; i++ ) {
//       Serial.print(i);
//       Serial.print(",");
//       Serial.print(inverseVoltage[i], 6);
//       Serial.print(",");
//       Serial.print(inverseCurrent_I[i], 6);
//       Serial.print(",");
//       Serial.print(inverseVoltage[i]*10.97, 6);
//       Serial.print(",");
//       Serial.println(inverseCurrent_I[i]/11000, 10);
//     }
//     Serial.print("Breakdown Voltage sin filtrar: ");
//     Serial.println((obtain_Vbd(inverseCurrent_I, inverseVoltage, Elementos) * 10.97) - 3.581, 6);
//     float* Filtered_voltage = apply_butterworth(inverseVoltage, Elementos);
//     float* Filtered_current = apply_butterworth(inverseCurrent_I, Elementos);
//     Serial.print("Voltage y Corriente Filtrados");
//     for ( uint8_t i = 0; i < Elementos; i++ ) {
//       Serial.println(Filtered_voltage[i], 6);
//       Serial.print(", ");
//       Serial.print(Filtered_current[i], 6);
//     }
//     Serial.print("Breakdown Voltage Filtrado: ");
//     Serial.println((obtain_Vbd(Filtered_current, Filtered_voltage, Elementos) * 10.97) - 3.581, 6);
//   }
  
//   Serial.println("Siguiente toma de muestras");
//   delay(5000);
// }

// /************************************************************************************************************
//  * @fn      obtain_Curve_inverseVI
//  * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
//  *          ADC.
//  * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
//  * @return  ---todo
//  * TODO: - Se necesita un algoritmo para setear el Vbias correctamente
//  * fny <= fs/2
//  */
// void obtain_Curve_inverseVI(float Temperature) {
//   // float Vbd_Teo = Vbd_teorical(Temperature);
//   float Vlimite_inferior = 26.2;//max(21.2, Vbd_Teo - 2); // 21.2 es el minimo valor a la salida del MAX
//   float Vlimite_superior = 30.2;//min(33.2, Vbd_Teo + 2); // 33.2 es el máximo valor a la salida del MAX
//   uint8_t Vlim_sup = VMax_command(Vlimite_inferior);
//   uint8_t Vlim_inf = VMax_command(Vlimite_superior);
//   uint8_t paso = (uint8_t)(Vlim_sup - Vlim_inf) / Elementos;
//   unsigned long start_time, total_time;

//   #ifdef DEBUG_MAIN
//   Serial.print("Limites: ");
//   Serial.print(Vlimite_inferior);
//   Serial.print(", ");
//   Serial.print(Vlimite_superior);
//   Serial.print(", ");
//   Serial.print(Vlim_inf, HEX);
//   Serial.print(", ");
//   Serial.print(Vlim_sup, HEX);
//   Serial.print(", paso: ");
//   Serial.print(paso);
//   #endif

//   for ( uint8_t i = 0; i < Elementos; i++ ) {
//     inverseVoltage_command[i] = Vlim_inf + i * paso;
//   }

//   start_time = micros();
//   for ( uint8_t i = 0; i < Elementos; i++ ) {
//     write_max_reg(inverseVoltage_command[i]);
//     // delayMicroseconds(Switching_Time_MAX); // 4 microseconds
//     delay(4); // por la frecuencia de muestreo del ADS1115 que es 860 SPS (muestreo c/ 1.16ms)
//     // Aquí se debe validar la tensión seteada en la salida del max con el ADC
//     inverseVoltage[i] = ads.computeVolts(ads.readADC_SingleEnded(0));
//     inverseCurrent_I[i] = ads.computeVolts(ads.readADC_SingleEnded(1)); // Microamperios
//     // Serial.print(i);
//     // Serial.print(",");
//     // Serial.print(inverseVoltage[i], 6);
//     // Serial.print(",");
//     // Serial.println(inverseCurrent_I[i], 6);

//     // delay(5000);
//   }
//   total_time = micros() - start_time;
//   Serial.print("Tiempo promedio de muestreo [ms]: ");
//   float Ts = (float)total_time / (Elementos * 1000);
//   Serial.println(Ts);
//   Fs = 1 / Ts;
//   Fc = 0.1 * Fs / 2;  
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