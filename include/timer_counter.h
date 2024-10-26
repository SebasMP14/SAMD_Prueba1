#ifndef TIMER_COUNTER_H
#define TIMER_COUNTER_H

extern uint16_t rising_edge_time;
extern uint16_t falling_edge_time;
extern uint16_t pulse_width;
extern uint16_t period;

void setupTC2();        // Interrupción para algoritmo de polarización
void TC2_Handler();     // Aplicación del algoritmo
// void setupTC4();
// void TC4_Handler();

#endif 

// PA16-17 para 32 bits