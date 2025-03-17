/**
 * main.cpp
 * Este código es una prueba de la comunicación OBC - MUA
 * Este sistema simula al OBC, mientras MUA prueba la FSM desarrollada
 * 
 * -> GuaraníSat2 -> MUA_Control -> FIUNA -> LME
 * 
 * Made by:
 * - Sebas Monje <2024-2025> (github)
 * 
 * TODO:
 * 
 */
#include <Arduino.h>

#include "hardware_pins.h"
#include "flash_driver.h"
#include "interrupts.h"
// #include ""

void setup() {
  delay(6000);

  Serial.begin(115200);                 // Puerto USB
  Serial.println("DEBUG (setup) -> Serial Iniciado");

  pinMode(INTERFACE_EN, OUTPUT);
  digitalWrite(INTERFACE_EN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  activeInterrupt2();

  /* Inicialización de memoria Flash */
  if ( !start_flash() ) {              // Se utiliza en ambos modos de operación
    // break;
    Serial.println("DEBUG (setup) -> Flash con problemss");
  }
  delay(2000);

  if (erase_all()) {
    Serial.print("comando de borrado enviado, status: 0b");
    Serial.println(Flash_QSPI.readStatus(), BIN);
    // Flash_QSPI
  } else {
    Serial.println("error, no se completó el borrado");
  }
  uint8_t escribir[5] = {0x01, 0xA3, 0xF5, 0x12, 0x14};
  write_mem(escribir, 5);
  uint16_t len = 200;
  uint8_t leer[len] = {};
  read_until(leer, len);
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(" 0x");
    Serial.print(leer[i], HEX);
  }
  Serial.println();
  delay(1000);
  // read_all();

  while(1) {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void loop() {

}

//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// SIMULACION OBC
/*
#include <Arduino.h>

#include "hardware_pins.h"
#include "RTC_SAMD51.h"
#include "DateTime.h"
#include "power_manager.h"

#define DEBUG_MAIN
#define TRAMA_SIZE 44
#define TRAMA_COMM 6
#define STOP_BYTE  0x0A

unsigned long date;
bool state = false;
bool state_recibir = false;
uint32_t Time;
uint8_t MISSION_ID            = 0x26;
uint8_t ID_COUNT_MODE         = 0x01;
uint8_t ID_TRANSFER_DATA_MODE = 0x02;
uint8_t ACK_OBC_TO_MUA        = 0x04;
uint8_t ACK_MUA_TO_OBC        = 0x07; // ACK MUA to OBC

const uint16_t crc_table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

RTC_SAMD51 rtc;

uint16_t crc_calculate(uint8_t *data);

void setup() {
  delay(4000);

  Serial.begin(115200);
  Serial.println("Serial iniciado");
  while ( !rtc.begin() ) {
    delay(1000);
    Serial.println("No rtc");
  }
  Serial.println("RTC iniciado.");
  DateTime now = DateTime(F(__DATE__), F(__TIME__));
  rtc.adjust(now);
  Serial1.begin(115200);
  Serial.println("Serial1 Iniciado");
  Serial2.begin(115200);
  Serial.println("Serial2 Iniciado");

  DateTime currentTime = rtc.now();

  Serial.print("Fecha: ");
  Serial.print(currentTime.year());
  Serial.print("-");
  Serial.print(currentTime.month());
  Serial.print("-");
  Serial.print(currentTime.day());
  Serial.print(" Hora: ");
  Serial.print(currentTime.hour());
  Serial.print(":");
  Serial.print(currentTime.minute());
  Serial.print(":");
  Serial.println(currentTime.second());

  Time = millis();

  while ( Serial2.available() ) {
    Serial2.read();
  }

  Serial.println("Setup Finalizado");

  // uint8_t trama_prueba[30] = {0x4A, 0x47, 0x36, 0x59, 0x42, 0x57, 0x30, 0x4A, 0x47, 0x36,
  //                            0x59, 0x50, 0x59, 0x30, 0x3E, 0xF0, 0xAA, 0x03, 0x12, 0x34, 
  //                            0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78};
  // uint16_t crc1 = calcularCRC(trama_prueba, 30);
  // uint16_t crc2 = mk_CRC(trama_prueba, 30);

}

void loop() {
  if ( millis() - Time > 5000 && !state ) { // Establecer COUNT MODE
    uint8_t recibido[TRAMA_COMM];
    uint8_t trama[TRAMA_COMM] = {MISSION_ID, ID_COUNT_MODE, 0x00};
    uint16_t CRC = crc_calculate(trama);
    Serial.print(" CRC calculado: 0x");
    Serial.println(CRC, HEX);
    trama[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
    trama[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
    trama[TRAMA_COMM-1] = STOP_BYTE;

    while (Serial1.available()){  // flush serial
      Serial1.read();
    }
    Serial1.write(trama, TRAMA_COMM); 
    
    Serial.print("Enviado por Serial1: 0x");
    for (uint8_t i = 0; i < TRAMA_COMM; i++) {
      Serial.print(trama[i], HEX);
      Serial.print(", 0x");
    }
    Serial.println();

    while ( Serial1.available() < TRAMA_COMM );
    while ( Serial2.available() ) {
      Serial2.read();
    }    
    Serial1.readBytes(recibido, TRAMA_COMM);
    Serial.print("Recibido de Serial1: 0x");
    for (uint8_t i = 0; i < TRAMA_COMM; i++) {
      Serial.print(recibido[i], HEX);
      Serial.print(", 0x");
    }
    Serial.println();
    
    if ( recibido[1] == ACK_MUA_TO_OBC ) {
      state = true;
      Serial.println("Estado (COUNT) establecido exitosamente");
    } else {
      Serial.print("Estado fallido");
    }

    Time = millis();
  }
  
  if ( Serial2.available() > 0 ) { // Para transmitir unixtime
    uint8_t read2;
    Serial2.readBytes(&read2, 1);

    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> Recibido Serial2: 0x");
    Serial.println(read2, HEX);
    #endif
    
    if ( read2 == 0xBB ) { // REQUEST_TIMESTAMP
      date = rtc.now().unixtime();
      Serial2.write((uint8_t *)&date, sizeof(date));
      #ifdef DEBUG_MAIN
      Serial.print("DEBUG (loop) -> timestamp enviado: ");
      Serial.println(date);
      #endif
    }
  }

  if ( millis() - Time > 10000 && !state ) {
    Serial.print("Date: ");
    Serial.print(rtc.now().year());
    Serial.print("-");
    Serial.print(rtc.now().month());
    Serial.print("-");
    Serial.print(rtc.now().day());
    Serial.print(" Hora: ");
    Serial.print(rtc.now().hour());
    Serial.print(":");
    Serial.print(rtc.now().minute());
    Serial.print(":");
    Serial.println(rtc.now().second());
    Serial.print(", ");
    Serial.print(rtc.now().unixtime());
    Serial.print(", ");
    Serial.println(rtc.now().unixtime(), HEX);
    Time = millis();
  }

  if ( millis() - Time > 40000 && state ) { // Establecer TRANSFER MODE
    Serial.println("Preparando para establecer TRANSFER MODE");
    
    uint8_t recibido[TRAMA_COMM];
    uint8_t trama[TRAMA_COMM] = {MISSION_ID, ID_TRANSFER_DATA_MODE, 0x00};
    uint16_t CRC = crc_calculate(trama);
    Serial.print(" CRC calculado: 0x");
    Serial.println(CRC, HEX);
    trama[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
    trama[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
    trama[TRAMA_COMM-1] = STOP_BYTE;

    while (Serial1.available()){  // flush serial
      Serial1.read();
    }
    Serial1.write(trama, TRAMA_COMM); 
    Serial.print("Enviado por Serial1:");
    for (uint8_t i = 0; i < TRAMA_COMM; i++) {
      Serial.print(" 0x");
      Serial.print(trama[i], HEX);
    }
    Serial.println();

    while ( Serial1.available() < TRAMA_COMM ); // esperar ack

    Serial1.readBytes(recibido, TRAMA_COMM);
    Serial.print("Recibido de Serial1: 0x");
    for (uint8_t i = 0; i < TRAMA_COMM; i++) {
      Serial.print(recibido[i], HEX);
      Serial.print(", 0x");
    }
    Serial.println();
    
    if ( recibido[1] == ACK_MUA_TO_OBC ) {
      state = true;
      Serial.println("Estado (TRANSFER) establecido exitosamente");
      state_recibir = true;
    } else {
      Serial.print("Estado fallido");
    }

    Time = millis();
  }

  if ( state_recibir ) {
    uint8_t trama_size = 36;
    uint8_t recibido[trama_size];
    uint8_t enviar[TRAMA_COMM] = {0x26, ACK_OBC_TO_MUA, 0x00};
    uint16_t CRC;
    Serial.println("Esperando datos");
    delay(100);
    while ( Serial1.available() < trama_size );

    Serial1.readBytes(recibido, trama_size);
    Serial.print("Recibido de Serial1 (DATOS): ");
    for (uint8_t i = 0; i < trama_size; i++) {
      Serial.print(" 0x");
      Serial.print(recibido[i], HEX);
    }
    Serial.println();

    CRC = crc_calculate(recibido);
    Serial.print(" CRC calculado: 0x");
    Serial.println(CRC, HEX);
    uint16_t CRC_MUA = (recibido[trama_size-3] << 8) | recibido[trama_size-2];
    if ( CRC == CRC_MUA && recibido[0] == 0x26) {
      // state = true;
      Serial.println("Datos recibidos correctamente");
      Serial.print(" CRC calculado: 0x");
      CRC = crc_calculate(enviar);
      Serial.println(CRC, HEX);
      enviar[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
      enviar[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
      enviar[TRAMA_COMM-1] = STOP_BYTE;

      while( Serial1.available() ) { // flush serial1
        Serial1.read();
      }
      Serial1.write(enviar, TRAMA_COMM);
      Serial.println("ACK enviado");
    } else {
      Serial.println("error en la recepción");
    }
  }
  delay(1000);
}



**
 * @fn                crc_calculate
 * @brief             Calculate the crc16 value
 * @param[in] data    Pointer to a buffer of up to 45 data_len bytes.
 * @return            The calculated crc value.
 *
uint16_t crc_calculate(uint8_t *data) {
  uint16_t crc = 0x1d0f;     //initial crc value
  uint16_t tbl_idx;
  
  // get the amount of appended data according to Data size
  uint8_t data_len = data[2];
  
  // operate only over data[1:data_len] to calculate the checksum 
  for ( uint8_t i = 1; i <= data_len + 2; i++ ) {
    tbl_idx = (((crc >> 8) ^ (*(data + i))) & 0xFF);
    crc = (crc_table[tbl_idx] ^ (crc << 8)) & 0xFFFF;
  }
  
  return (crc & 0xFFFF);
}
*/
//////////////////////////////////////////////////////////////////////////////////////////////////


//// Obtencion de puntos de la curva IV ////
////////////////////////////////////////////////////////////////////////////////////////////////
// #include <Arduino.h>
// #include <SPI.h>

// #include "hardware_pins.h"
// #include "ads1260_driver.h"
// #include "max1932_driver.h"
// #include "dac8551_driver.h"
// #include "tmp100_driver.h"
// #include "calculos.h"

// #define DEBUG_MAIN
// #define MAX_ITER 10
// #define Elementos 400                     // Cantidad de muestras
// #define OverVoltage 1                     // Sobrevoltaje aplicado para la polarización de los SiPMs
// #define Switching_Time_MAX 4              // Microseconds

// float inverseVoltage[Elementos];          // Tensión inversa aplicada al SiPM para obtener "inverseCurrent_I"
// uint16_t inverseVoltage_command[Elementos];
// float inverseVCurrent[Elementos];        // Corriente inversa, convertida de "inverseCurrent_V[]"
// float Filtered_voltage[Elementos];
// float Filtered_current[Elementos];
// float temperatureArray [Elementos];

// ADS1260 ads1260(&SPI1, SPI_CS_ADC);

// unsigned long time_ini;
// float temperature = 0.0;
// float firstCurrent = 0.0;

// void obtain_Curve_inverseVI(float Temperature);

// void setup() {
//   delay(5000);

//   Serial.begin(115200);
//   Serial.println("Serial iniciado");
  
//   pinMode(SPI_CS_DAC2, OUTPUT);
//   pinMode(SPI_CS_MAX2, OUTPUT);
//   pinMode(INTERFACE_EN, OUTPUT); // 
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(LED_SiPM1, OUTPUT);
//   pinMode(LED_SiPM2, OUTPUT);

//   digitalWrite(LED_BUILTIN, LOW);
//   digitalWrite(LED_SiPM1, LOW);
//   digitalWrite(LED_SiPM2, LOW);
//   digitalWrite(INTERFACE_EN, HIGH); //
//   delay(START_UP_TIME_ADS);                                       // Habilitación del ADC
//   delay(1000);

//   SPI.begin();                                                    // Descomentado en start_dac8551 y start_max1932
//   Serial.println("SPI iniciado");
//   start_dac8551(SPI_CS_DAC1);                                     // Solo el canal N
//   digitalWrite(SPI_CS_DAC2, HIGH);
//   start_max1932(SPI_CS_MAX1);
//   digitalWrite(SPI_CS_MAX2, HIGH);
//   Serial.println("CSs en HIGH");
//   if ( start_tmp100() ) {
//     Serial.println("TMP iniciado");
//   }
//   Serial.println((float)read_tmp100(), 4);
  

//   // ADC Configuration
//   ads1260.begin();
//   Serial.print("MODE0: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   // 00100100
//   ads1260.writeRegisterData(ADS1260_MODE0, 0b11111100);           // 40 KSPS - FIR (Page 30)
//   // ads1260.writeRegisterData(ADS1260_MODE0, 0b01101100);           // 14400 SPS
//   delay(50);
//   Serial.print("MODE0: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_MODE0), BIN);   
//   Serial.print("PGA: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
//   ads1260.writeRegisterData(ADS1260_PGA, 0b10000000);             // BYPASS MODE
//   Serial.print("PGA BYPASS MODE: ");
//   Serial.println(ads1260.readRegisterData(ADS1260_PGA), BIN);
//   ads1260.writeRegisterData(ADS1260_MODE3, 0b01000000);           // STATENB
//   // ads1260.writeRegisterData(ADS1260_REF, 0b00010000);             // REF 2.498V ENABLE
  
//   delay(500);
//   // Inicializar Vout1
//   write_dac8551_reg(0x7FFF, SPI_CS_DAC1);
//   write_max_reg(0x01, SPI_CS_MAX1);

//   time_ini = millis();
//   Serial.println("Setup finalizado");
//   // while(1);
// }

// void loop() {
//   delay(500);
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(500);
//   digitalWrite(LED_BUILTIN, LOW);

//   if ( millis() - time_ini >= 3000 ) {
//     Serial.println("Datos obtenidos: ");
//     Serial.print("Temperatura: ");

//     temperature = read_tmp100();
//     Serial.println(temperature, 4);
//     Serial.println("Obtención de curva ");
//     // delay(5000);
//     obtain_Curve_inverseVI(temperature);

//     time_ini = millis();
//     Serial.println("i, Voltage, VCorriente, (Voltage*12)-3.8-(VCorriente-firstCurrent), (VCorriente-firstCurrent)/2000");
//     for ( uint16_t i = 1; i < Elementos; i++ ) {
//       Serial.print(i);
//       Serial.print(",");
//       Serial.print(inverseVoltage[i], 7);
//       Serial.print(",");
//       Serial.print(inverseVCurrent[i], 7);
//       Serial.print(",");
//       Serial.print(temperatureArray[i]);
//       Serial.print(",");
//       Serial.print((inverseVoltage[i]*12)-3.8-(inverseVCurrent[i]-firstCurrent), 7);
//       Serial.print(",");
//       Serial.println((inverseVCurrent[i]-firstCurrent)/2000, 10);
//     }
//     // Serial.print("Breakdown Voltage sin filtrar: ");
//     // Serial.println((obtain_Vbd(inverseVCurrent, inverseVoltage, Elementos) * 12) - 3.8, 6);
//     sliding_moving_average(inverseVoltage, Elementos, 5, Filtered_voltage);
    
//     sliding_moving_average(inverseVCurrent, Elementos, 5, Filtered_current);
//     Serial.print("Voltage y Corriente Filtrados empieza en 5 seg");
//     delay(5000);
//     for ( uint16_t i = 0; i < Elementos; i++ ) {
//       Serial.print(i);
//       Serial.print(",");
//       Serial.print(Filtered_voltage[i], 6);
//       Serial.print(",");
//       Serial.println(Filtered_current[i], 6);
//     }
//     // Serial.print("Breakdown Voltage Filtrado: ");
//     // Serial.println((obtain_Vbd(Filtered_current, Filtered_voltage, Elementos) * 12) - 3.8, 6);
//   }
  
// }

// /************************************************************************************************************
//  * @fn      obtain_Curve_inverseVI
//  * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
//  *          ADC.
//  * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
//  * @return  ---todo
//  * TODO: - Se necesita un algoritmo para setear el Vbias correctamente
//  * 
//  */
// void obtain_Curve_inverseVI(float Temperature) {
//   float Vbd_Teo = Vbd_teorical(Temperature);
//   #ifdef DEBUG_MAIN
//   Serial.print("Vbd teorico: ");
//   Serial.println(Vbd_Teo);
//   #endif
//   // float Vlimite_inferior = max(24.588, Vbd_Teo - 2);      // 24.588 es el minimo valor a la salida del MAX
//   // float Vlimite_superior = min(36, Vbd_Teo + 2);      // 36 es el máximo valor a la salida del MAX
//   // uint16_t Vlim_sup = 0x3000;//VDAC_command(Vlimite_inferior); // Comandos 
//   // uint16_t Vlim_inf = 0x7FFF;//VDAC_command(Vlimite_superior); // Comandos 
//   // uint16_t paso = (uint16_t)(Vlim_inf - Vlim_sup) / Elementos;
//   // unsigned long start_time, total_time;
//   float Vlimite_inferior = max(24.588, Vbd_Teo + 3.8 - 1.5);      // 24.588 es el minimo valor a la salida del MAX
//   float Vlimite_superior = min(36, Vbd_Teo + 3.8 + 1.5);      // 36 es el máximo valor a la salida del MAX
//   uint16_t Vlim_inf = VDAC_command(Vlimite_inferior); // Comando
//   uint16_t Vlim_sup = VDAC_command(Vlimite_superior); // Comando
//   uint16_t paso = (uint16_t)(Vlim_inf - Vlim_sup) / Elementos;
//   unsigned long start_time, total_time;

//   #ifdef DEBUG_MAIN
//   Serial.print("Limites (dec, hex): ");
//   Serial.print(out_voltage(Vlim_inf), 2);
//   Serial.print(", ");
//   Serial.print(out_voltage(Vlim_sup), 2);
//   Serial.print(", ");
//   Serial.print(Vlim_inf, HEX);
//   Serial.print(", ");
//   Serial.print(Vlim_sup, HEX);
//   Serial.print(", paso: ");
//   Serial.println(paso);
//   #endif

//   for ( uint16_t i = 0; i < Elementos; i++ ) {          // Comandos para el DAC
//     inverseVoltage_command[i] = Vlim_inf - i * paso;
//   }
//   Serial.println("Comandos calculados");
//   start_time = micros();
//   for ( uint16_t i = 0; i < Elementos; i++ ) {
//     // if (i % 100 == 0 && i != 0) {
//     //   Serial.println("DEBUG (obtain_Curve_inverseVI) -> waiting");
//     //   // delay(5000);
//     // }
//     write_dac8551_reg(inverseVoltage_command[i], SPI_CS_DAC1);
//     // delayMicroseconds(Switching_Time_MAX); // 4 microseconds
//     delay(10); // Settling time of the MAX
//     // Aquí se debe validar la tensión seteada en la salida del max con el ADC
//     inverseVoltage[i] = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM));
//     inverseVCurrent[i] = ads1260.computeVolts(ads1260.readData(ADS1260_MUXP_AIN2, ADS1260_MUXN_AINCOM));
//     temperatureArray[i] = read_tmp100();
//     // uint32_t aux = 0x00;
//     // ads1260.connectMUX(ADS1260_MUXP_AIN0, ADS1260_MUXN_AINCOM);
//     // for ( uint8_t j = 0; j < 10; j++ ) {
//     //   aux += ads1260.readConversion();
//     //   delayMicroseconds(100);
//     // }
//     // inverseVoltage[i] = ads1260.computeVolts((uint32_t)(aux/10));
//     // aux = 0x00;
//     // ads1260.connectMUX(ADS1260_MUXP_AIN3, ADS1260_MUXN_AINCOM);
//     // for ( uint8_t j = 0; j < 10; j++ ) {
//     //   aux += ads1260.readConversion();
//     //   delayMicroseconds(100);
//     // }
//     // inverseVCurrent[i] = ads1260.computeVolts((uint32_t)(aux/10));
//   }
//   total_time = micros() - start_time;
//   firstCurrent = inverseVCurrent[0];
//   Serial.print("First VCurrent: ");
//   Serial.println(firstCurrent, 7);
//   Serial.print("Tiempo promedio de muestreo [ms]: ");
//   float Ts = (float)total_time / (Elementos * 1000);
//   Serial.println(Ts, 3);
// }


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


////////////////////////////    
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

//// Obtencion de puntos de la curva IV (Primeras pruebas con placa de pruebas) ////
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