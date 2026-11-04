/*
 * DRV8833.c
 *
 *  Created on: Nov 2, 2024
 *      Author: ALCIDES_RAMOS
 *
 *  Tabla del DRV8871:   IN1 IN2
 *                        0   0   libre (coast)
 *                        0   1   gira para un lado
 *                        1   0   gira para el otro lado
 *                        1   1   freno (brake)
 *
 *  Modo de manejo (DRV8871_FRENO en DRV8871.h):
 *   0 = FAST DECAY (coast): un pin con PWM, el otro en 0.
 *       En el tiempo apagado del PWM el motor queda libre y la corriente cae casi a cero:
 *       poca fuerza a PWM bajo, zona muerta grande y velocidad poco lineal.
 *   1 = SLOW DECAY (brake): un pin en 1 fijo, el otro con el PWM INVERTIDO.
 *       En el tiempo apagado el motor queda en corto (freno) y la corriente se mantiene:
 *       velocidad casi proporcional al PWM y zona muerta mucho menor.
 *       OJO: con PWM 0 el motor queda FRENADO (MD_Stop / MI_Stop lo dejan libre).
 */

#include "DRV8871.h"


//inicia los dvr correspondientes, en el caso 2
void DRV8871_Init()
{
	  PWM_init(&PWM1);
	  PWM_init(&PWM2);
	  PWM_init(&PWM3);
	  PWM_init(&PWM4);
	  MD_Stop();
	  MI_Stop();

}

//maneja un puente H con el valor v (0..ARR)
// alto: pin que va en 1 para girar en este sentido (en fast decay lleva el PWM)
// bajo: el otro pin (en slow decay lleva el PWM invertido)
static void DRV8871_Salida(PWMS *alto, PWMS *bajo, uint32_t v)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(alto->htim);
  if (v > arr + 1) v = arr + 1;

#if DRV8871_FRENO
  //slow decay: alto siempre en 1 (CCR = ARR+1), bajo en 0 durante v cuentas de ARR+1
  //  bajo en 0 -> el motor gira     bajo en 1 -> freno (1,1)
  PWM_RAW(alto, arr + 1);
  PWM_RAW(bajo, arr + 1 - v);
#else
  //fast decay: alto con PWM, bajo en 0
  PWM_RAW(alto, v);
  PWM_RAW(bajo, 0);
#endif
}

void MD_Adelante(uint32_t vm1)
{
  DRV8871_Salida(&PWM2, &PWM1, vm1);
}

void MD_Atras(uint32_t vm1)
{
  DRV8871_Salida(&PWM1, &PWM2, vm1);
}

void MI_Adelante(uint32_t vm2)
{
  DRV8871_Salida(&PWM3, &PWM4, vm2);
}

void MI_Atras(uint32_t vm2)
{
  DRV8871_Salida(&PWM4, &PWM3, vm2);
}

//freno activo: los dos pines en 1 (motor en corto), en cualquier modo
void MD_Freno()
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM1.htim);
  PWM_RAW(&PWM1, arr + 1);
  PWM_RAW(&PWM2, arr + 1);
}
void MI_Freno()
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(PWM3.htim);
  PWM_RAW(&PWM3, arr + 1);
  PWM_RAW(&PWM4, arr + 1);
}

//motor libre (coast): los dos pines en 0, en cualquier modo
void MD_Stop()
{
	  PWM_RAW(&PWM1, 0);
	  PWM_RAW(&PWM2,0);

}
void MI_Stop()
{
	  PWM_RAW(&PWM3, 0);
	  PWM_RAW(&PWM4,0);
}
