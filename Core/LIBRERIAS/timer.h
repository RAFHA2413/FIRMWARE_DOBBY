/*
 * timer.h
 *
 *  Created on: Oct 30, 2025
 *      Author: ALCIDES_RAMOS
 */

#ifndef LIBRERIAS_TIMER_H_
#define LIBRERIAS_TIMER_H_

#include "main.h"
//incluya libreria de sensores usados en la interrupcion del timer



//DEFINA TIMER A USAR
#define TIM_INT TIM6
#define htim_INT htim6







extern uint32_t tiempo_imu;
extern TIM_HandleTypeDef htim_INT;

extern  uint32_t tmicros;
extern float tmilis;
//extern float giro_z_filtrado ;
extern volatile float giro_z_filtrado ; //>>>>> CAMBIO: volatile (ISR)

void TIMER_INIT();
void TIMER_MIDE();
void TIMER_PROCEDIMIENTOS();
void Actualiza_Giro_Filtrado();


#endif /* LIBRERIAS_TIMER_H_ */
