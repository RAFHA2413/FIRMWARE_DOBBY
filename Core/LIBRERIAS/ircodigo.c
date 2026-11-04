/*
 * ircodigo.c
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Ver explicacion y uso en ircodigo.h
 */

#include "ircodigo.h"
#include "uart.h"
#include <stdio.h>

//hash FNV-1 de 32 bits
#define FNV_BASE   2166136261u
#define FNV_PRIMO  16777619u

//nivel del pin durante una marca (receptor tipo TSOP: reposo en alto, marca en bajo)
#define IR_NIVEL_MARCA  GPIO_PIN_RESET

static uint16_t          ir_tiempos[IR_MAX_TIEMPOS]; //duraciones en us: marca, espacio, marca...
static volatile uint16_t ir_n = 0;        //duraciones guardadas
static volatile uint8_t  ir_activo = 0;   //1 = recibiendo un codigo
static volatile uint8_t  ir_listo = 0;    //1 = codigo completo, esperando al main
static volatile uint32_t ir_ultimo_ms = 0; //tick del ultimo flanco (para medir silencio)


//deja todo listo para recibir el siguiente codigo
static void IR_rearma(void)
{
  ir_n = 0;
  ir_activo = 0;
  __HAL_GPIO_EXTI_CLEAR_IT(IR_PIN);
  ir_listo = 0;            //de ultimo: desde aqui la interrupcion vuelve a capturar
}

//fin de codigo: para el timer y avisa al main
static void IR_fin(void)
{
  IR_HTIM.Instance->CR1 &= ~TIM_CR1_CEN;
  ir_activo = 0;
  ir_listo = 1;
}

void IR_init(void)
{
  TIM_TypeDef *tim = IR_HTIM.Instance;

  //timer: 1 us por cuenta, desborda si no hay flancos en IR_TIMEOUT_US
  tim->CR1 &= ~TIM_CR1_CEN;
  tim->DIER &= ~TIM_DIER_UIE;
  tim->ARR = IR_TIMEOUT_US;
  tim->EGR = TIM_EGR_UG;      //carga el ARR ya (tiene precarga)
  tim->SR = ~TIM_SR_UIF;      //el UG levanta la bandera: se limpia
  tim->CNT = 0;
  tim->DIER |= TIM_DIER_UIE;

  //interrupcion en los dos flancos (el .ioc lo deja solo en subida)
  EXTI->RTSR1 |= IR_PIN;
  EXTI->FTSR1 |= IR_PIN;

  IR_rearma();
}

//interrupcion por flanco (EXTI9_IRQHandler)
void IR_flanco(void)
{
  if (__HAL_GPIO_EXTI_GET_IT(IR_PIN) == 0) return;
  __HAL_GPIO_EXTI_CLEAR_IT(IR_PIN);

  //silencio antes de este flanco (se mide siempre, aunque no se capture)
  uint32_t ahora = HAL_GetTick();
  uint32_t silencio = ahora - ir_ultimo_ms;
  ir_ultimo_ms = ahora;

  if (ir_listo) return;    //el main aun no lee el anterior

  TIM_TypeDef *tim = IR_HTIM.Instance;

  if (ir_activo == 0)
  {
    //solo empieza si la linea estaba en reposo: si no, es la mitad de otro codigo
    if (silencio < IR_SILENCIO_MS) return;
    //y debe empezar con una marca
    if (HAL_GPIO_ReadPin(IR_PUERTO, IR_PIN) != IR_NIVEL_MARCA) return;

    tim->CNT = 0;
    tim->SR = ~TIM_SR_UIF;
    tim->CR1 |= TIM_CR1_CEN;
    ir_activo = 1;
    return;
  }

  uint16_t duracion = (uint16_t)tim->CNT;
  tim->CNT = 0;

  ir_tiempos[ir_n] = duracion;
  ir_n++;

  if (ir_n >= IR_MAX_TIEMPOS) IR_fin();   //buffer lleno: corta aqui
}

//interrupcion del timer (TIM7_IRQHandler): silencio largo = fin del codigo
void IR_timer(void)
{
  TIM_TypeDef *tim = IR_HTIM.Instance;

  if ((tim->SR & TIM_SR_UIF) == 0) return;
  tim->SR = ~TIM_SR_UIF;

  if (ir_activo) IR_fin();
  else tim->CR1 &= ~TIM_CR1_CEN;
}

//0 = el nuevo es mas corto, 1 = parecido (+-20%), 2 = mas largo
static uint8_t IR_compara(uint16_t viejo, uint16_t nuevo)
{
  if ((uint32_t)nuevo * 10u < (uint32_t)viejo * 8u) return 0;
  if ((uint32_t)viejo * 10u < (uint32_t)nuevo * 8u) return 2;
  return 1;
}

//compara cada duracion con la del mismo tipo (2 posiciones antes) y arma el hash
static uint32_t IR_hash(void)
{
  uint32_t hash = FNV_BASE;

  for (uint16_t i = 0; (uint16_t)(i + 2u) < ir_n; i++)
    hash = (hash * FNV_PRIMO) ^ IR_compara(ir_tiempos[i], ir_tiempos[i + 2u]);

  return hash;
}

uint8_t IR_leer(uint32_t *codigo)
{
  if (ir_listo == 0) return 0;

  uint8_t valido = 0;
  if (ir_n >= IR_MIN_TIEMPOS)
  {
    *codigo = IR_hash();
    valido = 1;
  }

  IR_rearma();
  return valido;
}

//imprime las duraciones: M = marca (pulso), E = espacio
static void IR_imprime_crudo(void)
{
  char texto[16];

  for (uint16_t i = 0; i < ir_n; i++)
  {
    sprintf(texto, "%c%u ", (i & 1u) ? 'E' : 'M', ir_tiempos[i]);
    uartx_write_text(&IR_UART, texto);
    if ((i % 16u) == 15u) uartx_write_text(&IR_UART, "\r\n");
  }
  uartx_write_text(&IR_UART, "\r\n");
}

//se queda aqui imprimiendo los codigos que llegan (0 = para siempre)
void IR_calibrar(uint16_t cantidad)
{
  char texto[80];
  uint16_t leidos = 0;

  uartx_write_text(&IR_UART, "IR calibrar: pulse cada boton 2 o 3 veces\r\n");

  while ((cantidad == 0) || (leidos < cantidad))
  {
    if (ir_listo == 0) continue;

    if (ir_n < IR_MIN_TIEMPOS)
    {
      sprintf(texto, "IR: descartado (%u tiempos: ruido o repetir)\r\n", ir_n);
      uartx_write_text(&IR_UART, texto);
      IR_rearma();
      continue;
    }

    leidos++;
    sprintf(texto, "IR %u: codigo=0x%08lX  tiempos=%u\r\n",
            leidos, (unsigned long)IR_hash(), ir_n);
    uartx_write_text(&IR_UART, texto);
    IR_imprime_crudo();

    IR_rearma();
  }

  uartx_write_text(&IR_UART, "IR calibrar: fin\r\n");
}
