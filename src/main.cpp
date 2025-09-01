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
 * pio device monitor -p COM17
 */

 #include <Arduino.h>

#include "hardware_pins.h"
#include "interrupts.h"
#include "timer_counter.h"
#include "flash_driver.h"
#include "tmp100_driver.h"
#include "mcp4561_driver.h"
#include "max1932_driver.h"
#include "dac8551_driver.h"
#include "ads1260_driver.h"
#include "calculos.h"
#include "obc_comm.h"
#include "power_manager.h"




//////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// SIMULACION OBC

// #include <Arduino.h>

// #include "hardware_pins.h"
// #include "RTC_SAMD51.h"
// #include "DateTime.h"
// #include "power_manager.h"
// #include "flash_driver.h"
// #include "obc_comm.h"

// #define DEBUG_MAIN

// unsigned long date;

// bool state_recibir = false;
// unsigned long tiempo = 0;
 
// uint32_t Time;
// uint8_t ID_TRANSFER_DATA_MODE = 0x02;
// uint8_t ACK_OBC_TO_MUA        = 0x04;

// bool slidingWindowBuffer(uint8_t* buffer);
// bool buildDataFrame(uint8_t* trama, uint8_t ID, uint8_t trama_size, uint32_t address);
// bool verifyOBCResponse(uint8_t* recibido);
// bool sendDataFrame(void);
// uint16_t crc_calculate(uint8_t *data);

// // #define OBC_SIMULATION
// #define MUA_SIMULATION

// #ifdef OBC_SIMULATION
// bool state = false;
// void setup() {
//   delay(6000);

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

//   while ( Serial2.available() ) {
//     Serial2.read();
//   }

//   Serial.println("Setup Finalizado");

//   // uint8_t trama_prueba[30] = {0x4A, 0x47, 0x36, 0x59, 0x42, 0x57, 0x30, 0x4A, 0x47, 0x36,
//   //                            0x59, 0x50, 0x59, 0x30, 0x3E, 0xF0, 0xAA, 0x03, 0x12, 0x34, 
//   //                            0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78};
//   // uint16_t crc1 = calcularCRC(trama_prueba, 30);
//   // uint16_t crc2 = mk_CRC(trama_prueba, 30);

// }

// void loop() {
//   if ( millis() - Time > 5000 && !state ) { // Establecer COUNT MODE
//     uint8_t recibido[TRAMA_COMM];
//     uint8_t trama[TRAMA_COMM] = {MISSION_ID, ID_COUNT_MODE, 0x00};
//     uint16_t CRC = crc_calculate(trama);
//     Serial.print(" CRC calculado: 0x");
//     Serial.println(CRC, HEX);
//     trama[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
//     trama[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
//     trama[TRAMA_COMM-1] = STOP_BYTE;

//     while (Serial1.available()){  // flush serial
//       Serial1.read();
//     }
//     Serial1.write(trama, TRAMA_COMM); 
    
//     Serial.print("Enviado por Serial1:");
//     for (uint8_t i = 0; i < TRAMA_COMM; i++) {
//       Serial.print(" 0x"); Serial.print(trama[i], HEX);
//     }
//     Serial.println();

//     while ( millis() - Time < 10000 ) {
//       if ( Serial1.available() >= TRAMA_COMM ) {
//         break;
//       }
//     }
//     if ( Serial1.available() < TRAMA_COMM ) {
//       Time = millis();
//       return ;
//     }
//     while ( Serial2.available() ) {
//       Serial2.read();
//     }
//     Serial1.readBytes(recibido, TRAMA_COMM);
//     Serial.print("Recibido de Serial1: 0x");
//     for (uint8_t i = 0; i < TRAMA_COMM; i++) {
//       Serial.print(recibido[i], HEX);
//       Serial.print(", 0x");
//     }
//     Serial.println();
    
//     if ( recibido[1] == ID_COUNT_MODE ) {
//       state = true;
//       Serial.println("Estado (COUNT) establecido exitosamente");
//     } else {
//       Serial.print("Estado fallido");
//     }

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

//   if ( millis() - Time > 30000 && state ) { // Establecer TRANSFER MODE
//     Serial.println("Preparando para establecer TRANSFER MODE");
    
//     uint8_t recibido[TRAMA_COMM];
//     uint8_t trama[TRAMA_COMM] = {MISSION_ID, ID_TRANSFER_DATA_MODE, 0x00};
//     uint16_t CRC = crc_calculate(trama);
//     Serial.print(" CRC calculado: 0x");
//     Serial.println(CRC, HEX);
//     trama[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
//     trama[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
//     trama[TRAMA_COMM-1] = STOP_BYTE;

//     while (Serial1.available()){  // flush serial
//       Serial1.read();
//     }
//     Serial1.write(trama, TRAMA_COMM); 
//     Serial.print("Enviado por Serial1:");
//     for (uint8_t i = 0; i < TRAMA_COMM; i++) {
//       Serial.print(" 0x");
//       Serial.print(trama[i], HEX);
//     }
//     Serial.println();

//     while ( Serial1.available() < TRAMA_COMM ); // esperar ack

//     Serial1.readBytes(recibido, TRAMA_COMM);
//     Serial.print("Recibido de Serial1: 0x");
//     for (uint8_t i = 0; i < TRAMA_COMM; i++) {
//       Serial.print(recibido[i], HEX);
//       Serial.print(", 0x");
//     }
//     Serial.println();
    
//     if ( recibido[1] == ID_TRANSFER_DATA_MODE ) {
//       state = true;
//       Serial.println("Estado (TRANSFER) establecido exitosamente");
//       state_recibir = true;
//     } else {
//       Serial.print("Estado fallido");
//     }

//     Time = millis();
//   }

//   if ( state_recibir ) {
//     uint8_t trama_size = 42;
//     uint8_t recibido[trama_size];
//     uint8_t enviar[TRAMA_COMM] = {0x26, ACK_OBC_TO_MUA, 0x00};
//     uint16_t CRC;
//     Serial.println("Esperando datos");
//     delay(100);
//     while ( Serial1.available() < trama_size );

//     Serial1.readBytes(recibido, trama_size);
//     Serial.print("Recibido de Serial1 (DATOS): ");
//     for (uint8_t i = 0; i < trama_size; i++) {
//       Serial.print(" 0x");
//       Serial.print(recibido[i], HEX);
//     }
//     Serial.println();

//     CRC = crc_calculate(recibido);
//     Serial.print(" CRC calculado: 0x");
//     Serial.println(CRC, HEX);
//     uint16_t CRC_MUA = (recibido[trama_size-3] << 8) | recibido[trama_size-2];
//     if ( CRC == CRC_MUA && recibido[0] == 0x26) {
//       state = false;
//       state_recibir = false;
//       Serial.println("Datos recibidos correctamente");
//       Serial.print(" CRC calculado: 0x");
//       // CRC = crc_calculate(enviar);
//       // Serial.println(CRC, HEX);
//       // enviar[TRAMA_COMM-3] = (uint8_t)(CRC >> 8);
//       // enviar[TRAMA_COMM-2] = (uint8_t)(CRC & 0xFF);
//       enviar[1] = recibido[1];
//       enviar[TRAMA_COMM-3] = 0xAA;
//       enviar[TRAMA_COMM-2] = 0xAA;
//       enviar[TRAMA_COMM-1] = STOP_BYTE;

//       while( Serial1.available() ) { // flush serial1
//         Serial1.read();
//       }
//       Serial1.write(enviar, TRAMA_COMM);
//       Serial.println("ACK enviado");
//     } else {
//       Serial.println("error en la recepción");
//     }
//     Time = millis();
//   }
//   delay(1000);
// }
// #endif

// /////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////////////////////////

// #ifdef MUA_SIMULATION
// uint8_t state = 0x00;

// void setupCOUNT(void);
// void loopCOUNT(void);
// void setupTRANSFER(void);
// void loopTRANSFER(void);
// void loopTRANSFERinfo(void);

// void setup() {
//   delay(6000);

//   Serial.begin(115200);                 // Puerto USB
//   #ifdef DEBUG_MAIN
//   Serial.println("DEBUG (setup) -> Serial Iniciado");
//   #endif

//   // Serial.println("PRUEBA DE FLUJO DE PARTICULAS COMPENSANDO EL VBD, ambos canales");

//   Serial1.begin(115200);                // OBC (On Board Computer)
//   #ifdef DEBUG_MAIN
//   Serial.println("DEBUG (setup) -> Serial1 Iniciado");
//   #endif

//   // /* Inicialización de memoria Flash */
//   if ( !start_flash() ) {              // Se utiliza en ambos modos de operación
//     // break;
//     #ifdef DEBUG_MAIN
//     Serial.println("DEBUG (setup) -> Flash con problemss");
//     #endif
//   }

//   // if ( erase_debug() ) {
//   //   #ifdef DEBUG_MAIN
//   //   Serial.println("DEBUG (setup) -> debug borrado");
//   //   #endif
//   // } else {
//   //   #ifdef DEBUG_MAIN
//   //   Serial.println("DEBUG (setup) -> No se pudo borrar debug");
//   //   #endif
//   // }

//   // if ( erase_all() ) {
//   //   Serial.println("Flash borrada");
//   // }

//   // Restaurar último estado guardado en memoria
//   // get_OPstate(&state);


//   pinMode(PA01, OUTPUT);                            // Salida para TC2 (utiliza también PA15)

//   Serial.print("Estado: 0x");
//   Serial.println(state, HEX);

//   switch ( state ) {
//     case 0x00:                                // STAND_BY
//     case 0xFF:
//       currentMode = STAND_BY;
//       requestOperationMode();                 // Espera del modo de operación
//       if ( currentMode == COUNT_MODE ) {
//         setupCOUNT();
//       } else if ( currentMode == TRANSFER_DATA_MODE ) {
//         setupTRANSFER();
//       } else if ( currentMode == TRANSFER_INFO_MODE ) {
//         setupTRANSFER();
//       }
//       break;

//     case 0x01:                                // COUNT_MODE
//       currentMode = COUNT_MODE;
//       #ifdef DEBUG_MAIN
//       Serial.println("DEBUG (setup) -> COUNT_MODE iniciado");
//       #endif
//       setupCOUNT();
//       break;

//     case 0x02:                                // TRANSFER_DATA_MODE
//       currentMode = TRANSFER_DATA_MODE;
//       setupTRANSFER();
//       break;
    
//     case 0x08:
//       currentMode = FINISH;
//       enterOffMode();
//       break;

//     case 0x09:                                // TRANSFER_DATA_MODE
//       currentMode = TRANSFER_INFO_MODE;
//       setupTRANSFER();
//       break;

//     default:
//       /* Modo no seleccionado o incorrecto, manejar... */
//       currentMode = STAND_BY;
//       break;
//   }

//   #ifdef DEBUG_MAIN
//   Serial.println("DEBUG (setup) -> Setup finalizado...");
//   #endif
// }

// void loop() {
//   switch ( currentMode ) {
//     case STAND_BY:
//       requestOperationMode();
//       if (currentMode == COUNT_MODE) {
//         setupCOUNT();
//       } else if (currentMode == TRANSFER_DATA_MODE) {
//         setupTRANSFER();
//       } else if (currentMode == TRANSFER_INFO_MODE) {
//         setupTRANSFER();
//       }
//       break;

//     case COUNT_MODE:
//       loopCOUNT();
//       if ( Serial1.available() ) {
//         requestOperationMode();
//         if (currentMode == TRANSFER_DATA_MODE) {
//           setupTRANSFER();
//         } else if (currentMode == TRANSFER_INFO_MODE) {
//           setupTRANSFER();
//         }
//       }
//       break;

//     case TRANSFER_DATA_MODE:
//       loopTRANSFER();
//       break;

//     case TRANSFER_INFO_MODE:
//       loopTRANSFERinfo();
//       break;
    
//     case FINISH:
//       #ifdef DEBUG_MAIN
//       Serial.println("DEBUG (requestOperationMode) -> FINISH MODE ACTIVATED");
//       Serial.println("Sleep mode in progress: Executing order 66.");
//       #endif
//       write_OPstate(0x00);
//       enterOffMode();
//       break;

//     default:
//       #ifdef DEBUG_MAIN
//       Serial.println("DEBUG (loop) -> UNKNOWN_MODE");
//       #endif
//       delay(2000);
//       requestOperationMode();
//       // if ( !setup_state && currentMode != UNKNOWN_MODE) {
//       //   setupCOUNT();
//       // }
//       /* Modo no seleccionado o incorrecto, manejar... */
//       break;
//   }
// }

// void setupCOUNT() {
//   pinMode(Interface_EN, OUTPUT);
//   getTimestampFromGPS();
//   digitalWrite(Interface_EN, HIGH);
//   tiempo = millis();
//   Serial.println("setupCOUNT finalizado");
// }

// void loopCOUNT() {
//   if ( millis() - tiempo > 5000 ) {
//     Serial.println("loopCOUNT");
//     tiempo = millis();
//   }
// }

// void setupTRANSFER() {
//   digitalWrite(Interface_EN, LOW);
//   Serial.println("Datos de la FLASH: ");
//   read_all();
//   uint32_t start_address = 0x00000000;
//   write_SENT_DATAaddress(&start_address);  // Para transferir desde el inicio
//   Serial.println("setupTRANSFER finalizado");
// }

// void loopTRANSFER() {
//   delay(5000);

//   uint8_t buffer[TRAMA_COMM] = {0};

//   if ( slidingWindowBuffer(buffer, timeOUT) ) {  // Se busca y revisa una trama válida proveniente del OBC
//     if ( verifyOBCResponse(buffer) ) {  // Se verifica el CRC, si es NACK se maneja en la función
//       switch (buffer[1]) {
//         case ID_STANDBY:
//           currentMode = STAND_BY;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> STAND_BY ACTIVATED");
//           #endif
//           write_OPstate(ID_STANDBY);
//           return ;
//           break;
//         case ID_COUNT_MODE:
//           currentMode = COUNT_MODE;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> COUNT MODE ACTIVATED");
//           #endif
//           write_OPstate(ID_COUNT_MODE);
//           return ;
//           break;
//         case ID_TRANSFER_MODE:
//           currentMode = TRANSFER_DATA_MODE;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> TRANSFER MODE ACTIVATED");
//           #endif
//           write_OPstate(ID_TRANSFER_MODE);
//           // return ;
//           break;
//         case ID_FINISH:
//           currentMode = FINISH;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> FINISH MODE ACTIVATED");
//           Serial.println("Sleep mode in progress: Executing order 66.");
//           #endif
//           write_OPstate(ID_FINISH);
//           return ;
//           break;
//         case ID_TRANSFER_SYSINFO_MODE:
//           currentMode = TRANSFER_INFO_MODE;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> TRANSFER SYSINFO MODE ACTIVATED");
//           #endif
//           write_OPstate(ID_TRANSFER_SYSINFO_MODE);
//           return ;
//           break;
//         default:
//           currentMode = STAND_BY;
//           #ifdef DEBUG_MAIN
//           Serial.println("DEBUG (requestOperationMode) -> UNKNOWN MODE");
//           #endif
//           write_OPstate(ID_STANDBY);
//           return ;
//           break;
//       }   // switch (buffer[1])
//     }     // verifyOBCResponse
//   }       // slidingWindowBuffer

//   if ( !sendDataFrame() ) {            // Durante sendDataFrame se puede recibir comandos del OBC
//     #ifdef DEBUG_MAIN
//     Serial.println("DEBUG (loopTRANSFER) -> Falló el envío de trama.");
//     #endif
//     return ;
//   }
// }

// void loopTRANSFERinfo() {

// }

// #endif


// /************************************************************************************************************
//  * @fn      sendDataFrame
//  * @brief   Envía una trama de datos al OBC, recibe ACK o NACK y ejecuta en consecuencia
//  * @param   void
//  * @return  true: Transmisión exitosa, ACK recibido ... 
//  * @return  false: Transmisión fallida, CRC invalide, data frame invalid or timeout
//  * @todo    - Al esperar ACK solo debe ir timeOUT_invalid_frame
//  */
// bool sendDataFrame(void) {
//   uint32_t last_address_written = 0xFFFFFFFF;
//   uint32_t last_sent_address = 0xFFFFFFFF;

//   if (!get_address(&last_address_written)) return false;
//   if (!get_SENT_DATAaddress(&last_sent_address)) return false;

//   if (last_sent_address == 0xFFFFFFFF) last_sent_address = 0x00;  // primer envío de día uno (*festeja*)
//   if (last_address_written == last_sent_address) {                // ya no quedan datos en memoria por enviar
//     #ifdef DEBUG_MAIN
//     Serial.println("DEBUG (sendDataFrame) → last_address_written == last_sent_address");
//     #endif
//     // currentMode = FINISH;
//     return true;
//   }

//   uint8_t trama_size = TRAMA_DATA_SIZE + TRAMA_COMM;
//   uint8_t trama[trama_size];
//   if ( !buildDataFrame(trama, ID_SENT_DATA, TRAMA_DATA_SIZE, last_sent_address) ) return false;

//   Serial1.write(trama, trama_size);

//   #ifdef DEBUG_MAIN
//   Serial.println("DEBUG (sendDataFrame) -> Trama enviada:");
//   for ( uint8_t i = 0; i < trama_size; i++ ) {
//     Serial.print(" 0x"); Serial.print(trama[i], HEX);
//   }
//   Serial.println();
//   #endif

//   // unsigned long tiempo = millis();
//   // while ( Serial1.available() < TRAMA_COMM ) {
//   //   if ( (millis() - tiempo) >= timeOUT ) return false;
//   // }
//   uint8_t recibido[TRAMA_COMM];
//   // Serial1.readBytes(recibido, TRAMA_COMM);
//   if ( !slidingWindowBuffer(recibido, timeOUT_invalid_frame) ) {  // Solo debe ir timeOUT_invalid_frame
//     // delay(timeOUT_invalid_frame);                   // Se tiene que eliminar
//     Serial1.write(nack_IF_MUA_to_OBC, TRAMA_COMM);
//     #ifdef DEBUG_MAIN
//     Serial.println("ERROR (sendDataFrame) → Fallo slidingWindowBuffer");
//     #endif
//     return false;
//   }


//   #ifdef DEBUG_MAIN
//   Serial.println("DEBUG (sendDataFrame) -> Respuesta recibida:");
//   for ( uint8_t i = 0; i < TRAMA_COMM; i++ ) {
//     Serial.print(" 0x"); Serial.print(recibido[i], HEX);
//   }
//   Serial.println();
//   #endif

//   // if ( !verifyOBCResponse(recibido) ) return false;     // REVISAR, se maneja el invalid frame también

//   if ( recibido[1] == ID_SENT_DATA ) {
//     last_sent_address += TRAMA_DATA_SIZE;
//     write_SENT_DATAaddress(&last_sent_address);         // se actualiza la siguiente dirección a enviar
//   } else if (recibido[1] == ID_FINISH) {
//     currentMode = FINISH;
//   } else if (recibido[1] == ID_TRANSFER_SYSINFO_MODE) {
//     currentMode = TRANSFER_INFO_MODE;
//   }

//   return true;
// }



















































































/*
#include <Arduino.h>

#include "hardware_pins.h"
#include "flash_driver.h"
#include "interrupts.h"
#include "power_manager.h"
// #include ""

void setup() {
  delay(6000);

  Serial.begin(115200);                 // Puerto USB
  Serial.println("DEBUG (setup) -> Serial Iniciado");

  pinMode(INTERFACE_EN, OUTPUT);
  digitalWrite(INTERFACE_EN, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // activeInterrupt2();

  // Inicialización de memoria Flash 
  if ( !start_flash() ) {              // Se utiliza en ambos modos de operación
    // break;
    Serial.println("DEBUG (setup) -> Flash con problemss");
  }
  delay(2000);

  // if (erase_all()) {
  //   Serial.print("comando de borrado enviado, status: 0b");
  //   Serial.println(Flash_QSPI.readStatus(), BIN);
  //   // Flash_QSPI
  // } else {
  //   Serial.println("error, no se completó el borrado");
  // }
  uint8_t escribir[36] = { 0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
                          0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
                          0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01, 0xA3, 0xF5, 0x12, 0x14,
                          0x01, 0xA3, 0xF5, 0x12, 0x14, 0x01};
  write_mem(escribir, 36);
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
  uint8_t conteo = 0;
  while(1) {
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    conteo += 1;
    if ( conteo == 10 ) {
      Serial.println("Executing Order 66...");
      enterOffMode();
    }
  }
}

void loop() {

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