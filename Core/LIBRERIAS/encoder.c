/*
 * encoder.c
 *
 *  Created on: Mar 18, 2025
 *      Author: ALCIDES_RAMOS
 */

#include  "encoder.h"

//incluir los timer usados
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

//configura encoder a usar
ENCODERS ENCODER1 = {&htim1,TIM1};
ENCODERS ENCODER2 = {&htim2,TIM2};

//define direccion de giro encoders (signo de pulsos y vueltas)
#define ENCODER1_DIR   1
#define ENCODER2_DIR   1

//define signo de la velocidad (rpm)
#define ENCODER1_VEL   1
#define ENCODER2_VEL   1

//filtro pasa bajos de las rpm: 1.0 = sin filtro, mas pequeno = mas suave pero mas lento
#define ENCODER_FILTRO 0.3f

static uint16_t encoder_ms = 0;   //contador de milisegundos para el muestreo
volatile uint8_t encoder_muestra = 0;   //1 = hay una muestra nueva (la borra quien la usa, ej. llanta)


//ppr: pulsos por vuelta del encoder (un canal, en el eje del motor)
//modo: 4 = x4, 2 = x2
//numbits: 16 o 32 bits del timer
void Encoder_init(ENCODERS *encoder, uint16_t ppr, uint8_t modo, uint8_t numbits)
{
  encoder->cpr = 0;   //mientras sea 0, Encoder_velo no toca este encoder

  HAL_TIM_Encoder_Start(encoder->htim, TIM_CHANNEL_ALL);

  encoder->ppr     = ppr;
  encoder->modo    = modo;
  encoder->numbits = numbits;
  encoder->k_rpm   = 60000.0f / ((float)ppr * modo * ENCODER_TS_MS);   //cuentas/ventana -> rpm

  encoder->cnt_ant   = __HAL_TIM_GET_COUNTER(encoder->htim);   //evita un salto en la primera medicion
  encoder->delta     = 0;
  encoder->pulsos    = 0;
  encoder->vueltas   = 0;
  encoder->rpm_cruda = 0;
  encoder->rpm       = 0;

  encoder->cpr = ppr * modo;   //va al final: habilita el encoder en Encoder_velo
}

//pone en cero los pulsos y vueltas acumulados (no afecta las rpm)
void Encoder_reset(ENCODERS *encoder)
{
  encoder->pulsos  = 0;
  encoder->vueltas = 0;
}

//lee el timer y actualiza pulsos, vueltas y rpm de UN encoder
static void Encoder_actualiza(ENCODERS *encoder, int8_t dir, int8_t vel)
{
  if (encoder->cpr == 0) return;   //aun no inicializado

  uint32_t cnt = __HAL_TIM_GET_COUNTER(encoder->htim);
  int32_t diff;

  //diferencia de cuentas; el cast resuelve el desborde del contador
  if (encoder->numbits == 16)
    diff = (int16_t)((uint16_t)cnt - (uint16_t)encoder->cnt_ant);
  else
    diff = (int32_t)(cnt - encoder->cnt_ant);

  encoder->cnt_ant = cnt;
  encoder->delta   = diff;

  //posicion acumulada (signo ENCODERx_DIR)
  encoder->pulsos += dir * diff;
  encoder->vueltas = (float)encoder->pulsos / encoder->cpr;

  //rpm del motor (signo ENCODERx_VEL) + filtro pasa bajos
  encoder->rpm_cruda = (float)(vel * diff) * encoder->k_rpm;
  encoder->rpm += ENCODER_FILTRO * (encoder->rpm_cruda - encoder->rpm);
}

//Lo llamo cada 1 ms en (SysTick_Handler)
void Encoder_velo(void)
{
  encoder_ms++;

  if (encoder_ms >= ENCODER_TS_MS)
  {
    encoder_ms = 0;

    //agregar una linea por cada encoder usado
    Encoder_actualiza(&ENCODER1, ENCODER1_DIR, ENCODER1_VEL);
    Encoder_actualiza(&ENCODER2, ENCODER2_DIR, ENCODER2_VEL);

    encoder_muestra = 1;   //avisa que hay rpm nuevas (Llanta_velo corre el PID con esto)
  }
}
