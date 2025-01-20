#include "Arduino.h"
#include "timer_counter.h"
#include "hardware_pins.h"

uint16_t rising_edge_time = 0;
uint16_t falling_edge_time = 0;
uint16_t pulse_width = 0;
uint16_t period = 0;

void setupTC2() {
  // Habilitar el reloj para TC2
  GCLK->PCHCTRL[TC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Usa el generador de reloj 0
  GCLK->PCHCTRL[TC3_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Habilitar reloj para TC3 también (parte del contador de 32 bits)
  
  TC2->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC2
  while (TC2->COUNT32.SYNCBUSY.bit.SWRST);  // Esperar hasta que se complete el reseteo
  TC3->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC3
  while (TC3->COUNT32.SYNCBUSY.bit.SWRST);  // 

  // Configurar el prescaler y modo
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; // Prescaler de 1024
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32; // Modo de 32 bits

  // Configurar la comparación para generar una interrupción
  TC2->COUNT32.CC[0].reg = (120000000 * 10 / 1024); // CC_Value = clk_frec * Tiempo_deseado / prescaler
  while (TC2->COUNT32.SYNCBUSY.bit.CC0);

  // Habilitar interrupciones por comparación
  TC2->COUNT32.INTENSET.reg = TC_INTENSET_MC0; // Habilitar interrupción de desbordamiento

  // Habilitar el TC2
  TC2->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC2->COUNT32.SYNCBUSY.bit.ENABLE); // Esperar a que TC2 se habilite

  // Habilitar la interrupción en el NVIC
  NVIC_EnableIRQ(TC2_IRQn);
}

// Manejador de interrupciones de TC2
void TC2_Handler() {
  if (TC2->COUNT32.INTFLAG.bit.MC0) { // Si ocurre un overflow
    TC2->COUNT32.INTFLAG.bit.MC0 = 1; // Limpiar la bandera de interrupción
    TC2->COUNT32.COUNT.reg = 0x0; // Se resetea el registro
    // digitalWrite(SCL_Sensor, !digitalRead(SCL_Sensor)); // Alternar el estado del LED
    Serial.println("Interrupción TC.");
  }

}

/************************************************************************************************************
 * @fn      setupTC()
 * @brief   Configuración de timer counter de 16 bits 
 * @param   NONE
 * @return  NONE
 */
// void setupTC4(void) {

//   EVSYS->USER[EVSYS_ID_USER_TC4_EVU].reg = EVSYS_USER_CHANNEL(1);

//   /* Select the event system generator on channel 0 */
//   // EVSYS->Channel[0].CHANNEL.reg |= EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;              	/* No event edge detection */
//   EVSYS->Channel[0].CHANNEL.reg |= EVSYS_CHANNEL_PATH_ASYNCHRONOUS;                 	/* Set event path as asynchronous */
//   EVSYS->Channel[0].CHANNEL.reg |= EVSYS_CHANNEL_EDGSEL_BOTH_EDGES;
//   EVSYS->Channel[0].CHANNEL.reg |= EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_8);   	/* Set event generator (sender) as external interrupt 8 */ 

//   // Habilitar el reloj para TC4
//   GCLK->PCHCTRL[TC4_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN; // Usa el generador de reloj 0
//   while (GCLK->SYNCBUSY.bit.GENCTRL0);
  
//   TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // Resetear TC4
//   while (TC4->COUNT16.SYNCBUSY.bit.SWRST);  // Esperar hasta que se complete el reseteo
  
//   // Configurar el prescaler y modo
//   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024; // Prescaler de 1024
//   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Modo de 16 bits

//   TC4->COUNT16.INTENSET.reg |= TC_INTENSET_MC1;						/* Enable compare channel 1 (CC1) interrupts */
//   TC4->COUNT16.INTENSET.reg |= TC_INTENSET_MC0;         	/* Enable compare channel 0 (CC0) interrupts  */
//   TC4->COUNT16.CTRLA.reg    |= TC_CTRLA_CAPTEN1;					/* Enable pulse capture on CC1 */
//   TC4->COUNT16.CTRLA.reg    |= TC_CTRLA_CAPTEN0;					/* Enable pulse capture on CC0 */
//   TC4->COUNT16.EVCTRL.reg   |= TC_EVCTRL_EVACT_PPW; 			/* Enable the TC to PPW capture */



//   // Habilitar el TC4
//   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
//   while (TC4->COUNT16.SYNCBUSY.bit.ENABLE); // Esperar a que TC4 se habilite

//   NVIC_EnableIRQ(TC4_IRQn); // Habilitar la interrupción en el NVIC
// }

// /*
//  * @fn		TC4_Handler
//  * @brief	Interrupt Service Routine (ISR) for timer TC4
//  *
//  */
// void TC4_Handler()
// {      
//   if (TC4->COUNT16.INTFLAG.bit.MC0)                         /* Check for match counter 4 (MC0) interrupt */
//   {   
//     period = TC4->COUNT16.CC[0].reg;                        /* Copy the period */ 
//     TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;            /* Disable compare channel 4 (CC0) interrupts */
//     TC4->COUNT16.INTFLAG.bit.MC0 = 1;                        
//     Serial.println("TC handler1");
//   }
 
//   if (TC4->COUNT16.INTFLAG.bit.MC1)                         /* Check for match counter 1 (MC1) interrupt */
//   {
//     pulse_width = TC4->COUNT16.CC[1].reg;                    /* Copy the pulse width */
//     TC4->COUNT16.INTENCLR.reg = TC_INTENCLR_MC1;            /* Disable compare channel 1 (CC1) interrupts */  
//     TC4->COUNT16.INTFLAG.bit.MC1 = 1;  
//     Serial.println("TC handler2");                            
//   }
  
// }


