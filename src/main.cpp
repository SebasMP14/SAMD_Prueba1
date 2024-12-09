/**
 * main.cpp
 * Este código es una prueba para la obtención de la curva reverse IV del SiPM
 * Se integran el MAX1932, el ADS1115 y el TMP100
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
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "max1932_driver.h"
#include "tmp100_driver.h"
#include "hardware_pins.h"
#include "calculos.h"

#define DEBUG_MAIN
#define MAX_ITER 10
#define Elementos 100                     // Cantidad de muestras
#define OverVoltage 3                     // Sobrevoltaje aplicado para la polarización de los SiPMs
#define Switching_Time_MAX 4              // Microseconds

#define ADS_ADDRESS 0x48 //48
#define P PA20

float inverseVoltage[Elementos];          // Tensión inversa aplicada al SiPM para obtener "inverseCurrent_I"
uint8_t inverseVoltage_command[Elementos];
float inverseCurrent_I[Elementos];        // Corriente inversa, convertida de "inverseCurrent_V[]"

Adafruit_ADS1115 ads;

unsigned long time_ini;
float temperature = 0.0;

void obtain_Curve_inverseVI(float Temperature);

void setup() {
  delay(4000);

  Serial.begin(115200);
  Serial.println("Serial iniciado");

  // start_max1932();
  // for ( uint8_t iter_counter = 0; iter_counter <= MAX_ITER ; iter_counter ++) {
  //   if ( start_tmp100() ) { // Configuración del TMP100, HACER EN VARIOS INTENTOS
  //     #ifdef DEBUG_MAIN
  //     Serial.println("DEBUG (setupCOUNT) -> Inicialización de TMP100 exitosa.");
  //     #endif
  //     break;
  //   } else {
  //     #ifdef DEBUG_MAIN
  //     Serial.print("DEBUG (setupCOUNT) -> Inicialización de TMP100 fallida: ");
  //     Serial.println(iter_counter);
  //     #endif
  //     delay(10);
  //   }
  // }
  if ( ads.begin() ) { // ADS_ADDRESS, &Wire1
    ads.setDataRate(RATE_ADS1115_860SPS);
    Serial.print("ADS iniciado, DataRate: ");
    Serial.println(ads.getDataRate());
  }

  time_ini = millis();
  Serial.println("Setup finalizado");

  analogWriteResolution(12);
  pinMode(A0, OUTPUT);
  Serial.println("i, VoltageT, VCorriente");
  for (int i = 0; i < 4095; i+=4) {
    Serial.print(i);
    Serial.print(",");
    Serial.print((3.3/4095)*i, 4);
    Serial.print(",");
    analogWrite(A0, i);
    delay(4);
    Serial.println(ads.computeVolts(ads.readADC_SingleEnded(0)), 6);
  }
  while ( 1 );
}

void loop() {
  if ( millis() - time_ini >= 5000 ) {
    Serial.println("Datos obtenidos: ");
    Serial.print("Temperatura: ");

    temperature = 28.0;//read_tmp100();
    // Serial.println(temperature, 4);
    obtain_Curve_inverseVI(temperature);

    time_ini = millis();
    init_butterworth();
    Serial.println("i, Voltage, Corriente, Voltage*10.97, Corriente/11000");
    for ( uint8_t i = 0; i < Elementos; i++ ) {
      Serial.print(i);
      Serial.print(",");
      Serial.print(inverseVoltage[i], 6);
      Serial.print(",");
      Serial.print(inverseCurrent_I[i], 6);
      Serial.print(",");
      Serial.print(inverseVoltage[i]*10.97, 6);
      Serial.print(",");
      Serial.println(inverseCurrent_I[i]/11000, 10);
    }
    Serial.print("Breakdown Voltage sin filtrar: ");
    Serial.println((obtain_Vbd(inverseCurrent_I, inverseVoltage, Elementos) * 10.97) - 3.581, 6);
    float* Filtered_voltage = apply_butterworth(inverseVoltage, Elementos);
    float* Filtered_current = apply_butterworth(inverseCurrent_I, Elementos);
    Serial.print("Voltage y Corriente Filtrados");
    for ( uint8_t i = 0; i < Elementos; i++ ) {
      Serial.println(Filtered_voltage[i], 6);
      Serial.print(", ");
      Serial.print(Filtered_current[i], 6);
    }
    Serial.print("Breakdown Voltage Filtrado: ");
    Serial.println((obtain_Vbd(Filtered_current, Filtered_voltage, Elementos) * 10.97) - 3.581, 6);
  }
  
  Serial.println("Siguiente toma de muestras");
  delay(5000);
}

/************************************************************************************************************
 * @fn      obtain_Curve_inverseVI
 * @brief   Se obtiene la curva I-V inversa del SiPM aplicando un filtro de butterworth a las lecturas del
 *          ADC.
 * @param   Temperature: obtenido del sensor TMP100 para la estimación teorica
 * @return  ---todo
 * TODO: - Se necesita un algoritmo para setear el Vbias correctamente
 * fny <= fs/2
 */
void obtain_Curve_inverseVI(float Temperature) {
  // float Vbd_Teo = Vbd_teorical(Temperature);
  float Vlimite_inferior = 26.2;//max(21.2, Vbd_Teo - 2); // 21.2 es el minimo valor a la salida del MAX
  float Vlimite_superior = 30.2;//min(33.2, Vbd_Teo + 2); // 33.2 es el máximo valor a la salida del MAX
  uint8_t Vlim_sup = VMax_command(Vlimite_inferior);
  uint8_t Vlim_inf = VMax_command(Vlimite_superior);
  uint8_t paso = (uint8_t)(Vlim_sup - Vlim_inf) / Elementos;
  unsigned long start_time, total_time;

  #ifdef DEBUG_MAIN
  Serial.print("Limites: ");
  Serial.print(Vlimite_inferior);
  Serial.print(", ");
  Serial.print(Vlimite_superior);
  Serial.print(", ");
  Serial.print(Vlim_inf, HEX);
  Serial.print(", ");
  Serial.print(Vlim_sup, HEX);
  Serial.print(", paso: ");
  Serial.print(paso);
  #endif

  for ( uint8_t i = 0; i < Elementos; i++ ) {
    inverseVoltage_command[i] = Vlim_inf + i * paso;
  }

  start_time = micros();
  for ( uint8_t i = 0; i < Elementos; i++ ) {
    write_max_reg(inverseVoltage_command[i]);
    // delayMicroseconds(Switching_Time_MAX); // 4 microseconds
    delay(4); // por la frecuencia de muestreo del ADS1115 que es 860 SPS (muestreo c/ 1.16ms)
    // Aquí se debe validar la tensión seteada en la salida del max con el ADC
    inverseVoltage[i] = ads.computeVolts(ads.readADC_SingleEnded(0));
    inverseCurrent_I[i] = ads.computeVolts(ads.readADC_SingleEnded(1)); // Microamperios
    // Serial.print(i);
    // Serial.print(",");
    // Serial.print(inverseVoltage[i], 6);
    // Serial.print(",");
    // Serial.println(inverseCurrent_I[i], 6);

    // delay(5000);
  }
  total_time = micros() - start_time;
  Serial.print("Tiempo promedio de muestreo [ms]: ");
  float Ts = (float)total_time / (Elementos * 1000);
  Serial.println(Ts);
  Fs = 1 / Ts;
  Fc = 0.1 * Fs / 2;  
}


/////////////////////////////////////////////////////////////////////////////////////////////////
/* Prueba de comunicación UART con RTC, el programa recibe los datos

#include <Arduino.h>
// #include <SPI.h>


// #include "max1932_driver.h"
#include "hardware_pins.h"
#include "RTC_SAMD51.h"
#include "DateTime.h"

#define DEBUG_MAIN
#define P PA20

unsigned long date;
bool state = false;
uint32_t Time;

RTC_SAMD51 rtc;

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

  pinMode(P, OUTPUT);

  Time = millis();
  Serial.println("Setup Finalizado");
}

void loop() {
  if ( Serial1.available() > 0 && !state ) {
    uint8_t read1;
    Serial1.readBytes(&read1, 1);
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> Recibido Serial1: 0x");
    Serial.println(read1, HEX);
    #endif
    if ( read1 == 0xAA ) {
      state = true;
      Serial1.write(0xA1); // COUNT_MODE
    }
  }
  
  if ( Serial1.available() > 0 && state) { 
    uint8_t read1;
    Serial1.readBytes(&read1, 1);
    Serial.print(" 0x");
    Serial.print(read1, HEX);
  }
  
  if ( Serial2.available() > 0 ) {
    uint8_t read2;
    Serial2.readBytes(&read2, 1);
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> Recibido Serial2: 0x");
    Serial.println(read2, HEX); 
    #endif
    if ( read2 == 0xBB ) { // REQUEST_TIMESTAMP
      date = rtc.now().unixtime();
      Serial2.write((uint8_t *)&date, sizeof(date));
    }
    #ifdef DEBUG_MAIN
    Serial.print("DEBUG (loop) -> timestamp enviado: ");
    Serial.println(date);
    #endif
  }

  if ( millis() - Time > 5000 ) {
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
}

*/




/////////////////////////////////////////////////////////////////////////////////////////////////
/*


void setup() {
  delay(4000);
  Serial.begin(115200);                                 // Open the serial port at 115200 baud
  while(!Serial);                                       // Wait for the console to open
  Serial.println("Pulse Measurement");   // print out the file name
  delay(2000);

  MCLK->APBBMASK.reg |= MCLK_APBBMASK_EVSYS;         // Switch on the event system peripheral
  
  // Set up the generic clock (GCLK7) used to clock timers 
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(1) |       // Divide the 120MHz clock source by divisor 1: 120MHz/1 = 120MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         //GCLK_GENCTRL_SRC_DFLL;    // Generate from 48MHz DFLL clock source
                         GCLK_GENCTRL_SRC_DPLL0;     // Generate from 120MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DFLL1;   // Generate from 100MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization
  
  GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral TC0
                                   GCLK_PCHCTRL_GEN_GCLK7;     // Connect  120MHz generic clock 7 to TC0
  // GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |         // Enable perhipheral TC4
  //                                  GCLK_PCHCTRL_GEN_GCLK0;     // Connect  120MHz generic clock 0 to TC4

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC4
  while (TC4->COUNT16.SYNCBUSY.bit.SWRST);
                                   
  //----------------------------------------------------------------------------------------------------------------------
  //======================================== setup pulsewidth measurment on TC4 input on pion PB08
  //----------------------------------------------------------------------------------------------------------------------

  // Enable the port multiplexer on analog pin PB08
  PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;

  // Set-up the pin as an EIC (interrupt) peripheral on analog pin PB08
  PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg |= PORT_PMUX_PMUXE(0x05);

  EIC->CTRLA.bit.ENABLE = 0;                        // Disable the EIC peripheral
  while (EIC->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization 
  EIC->CONFIG[0].reg = EIC_CONFIG_SENSE4_HIGH;      // Set event on detecting a HIGH level
  EIC->EVCTRL.reg = 1 << 8;                         // Enable event output on external interrupt 8 
  EIC->INTENCLR.reg = 1 << 8;                       // Clear interrupt on external interrupt 8
  EIC->ASYNCH.reg = 1 << 8;                         // Set-up interrupt as asynchronous input
  EIC->CTRLA.bit.ENABLE = 1;                        // Enable the EIC peripheral
  while (EIC->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
 
  // Select the event system user on channel 0 (USER number = channel number + 1)
  EVSYS->USER[EVSYS_ID_USER_TC4_EVU].reg = EVSYS_USER_CHANNEL(1);                   // Set the event user (receiver) as timer TC4

  // Select the event system generator on channel 0
  EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |              // No event edge detection
                                  EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                 // Set event path as asynchronous
                                  EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_8);   // Set event generator (sender) as external interrupt 8
  
  TC4->COUNT16.EVCTRL.reg = TC_EVCTRL_TCEI |               	// Enable the TCC event input 
                            TC_EVCTRL_EVACT_PPW;            	// Set up the timer for capture: CC0 period, CC1 pulsewidth

  NVIC_SetPriority(TC4_IRQn, 0);      			// Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);           			// Connect the TC4 timer to the Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC1;		// Enable compare channel 1 (CC1) interrupts (pulsewidth)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_CAPTEN1;			// Enable pulse capture on CC1 (pulsewidth)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;		// Set the timer to 16-bit mode
  
  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                        	// Enable the TC4 timer
  while (TC4->COUNT16.SYNCBUSY.bit.ENABLE);                 	// Wait for synchronization
   
}

*/


/////////////////////////////////////////////////////////////////////////////////////////////////////

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