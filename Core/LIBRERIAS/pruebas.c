/*
 * pruebas.c
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "pruebas.h"
#include "pinout1.h"
#include "llanta.h"
#include "IR_ANA_SEN.h"
#include "timer.h"
#include "ws2812b.h"
#include "telemetria.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PRUEBA_BANDERA_MS  500   //cada cuanto se avisa la bandera al panel

PRUEBAS PRUEBA = {0, PRUEBA_MOTORES};

//nombre de cada prueba (mismo orden del enum)
static const char *const PRUEBA_NOMBRE[PRUEBA_N] = {
    "MOTORES", "ENCODER", "SENSORES", "IMU"
};

//cada cuanto corre cada prueba en ms (0 = no manda nada)
static const uint16_t PRUEBA_PERIODO[PRUEBA_N] = {
    0, 250, 100, 50
};

static char tx[200];


//avisa al panel la bandera y la prueba:  *TST1**TSTE2*
static void Prueba_manda_bandera(void)
{
  sprintf(tx, "*TST%d**TSTE%d*", (int)PRUEBA.activo, (int)PRUEBA.estado);
  uartx_write_text(&TELE_UART, tx);
}

//aplica el PWM pedido a los motores, solo si TEST esta ON
static void Prueba_aplica_pwm(void)
{
  if (!PRUEBA.activo)
  {
    uartx_write_text(&TELE_UART, "TEST OFF: prender TEST para mover los motores\r\n");
    return;
  }
  Llanta_set_pwm(PRUEBA.pwm_izq, PRUEBA.pwm_der);
}

//accion de ENTRADA a un estado (se hace una sola vez al llegar)
static void Prueba_entra(uint8_t estado)
{
  sprintf(tx, "TEST: prueba %d %s\r\n", (int)estado, PRUEBA_NOMBRE[estado]);
  uartx_write_text(&TELE_UART, tx);

  switch (estado)
  {
    case PRUEBA_ENCODER:
      Llanta_reset();   //pulsos y distancia desde cero
      break;

    case PRUEBA_SENSORES:
      PIN_ON(LEDS_IR);  //enciende los emisores de los sensores
      break;

    default:
      break;
  }
}

//accion de SALIDA de un estado (se hace una sola vez al irse)
static void Prueba_sale(uint8_t estado)
{
  switch (estado)
  {
    default:          //por ahora ninguna prueba necesita limpiar algo
      break;
  }
}


//prende / apaga el modo test (la bandera)
void Prueba_activa(uint8_t on)
{
  if (on)
  {
    PRUEBA.activo = 1;
    PRUEBA.estado_ant = 0xFF;          //fuerza la accion de entrada de la prueba actual
    WS2812_Color(80, 60, 0);           //amarillo = robot en TEST
    WS2812_Manda_Trama();
    uartx_write_text(&TELE_UART, "TEST: ON\r\n");
  }
  else
  {
    if (PRUEBA.activo && (PRUEBA.estado_ant < PRUEBA_N)) Prueba_sale(PRUEBA.estado_ant);
    PRUEBA.activo  = 0;
    PRUEBA.pwm_izq = 0.0f;
    PRUEBA.pwm_der = 0.0f;
    if (LLANTA_CAL.modo == LLANTA_PWM) Llanta_modo(LLANTA_OFF);   //apaga los motores del test
    WS2812_Color(0, 0, 0);
    WS2812_Manda_Trama();
    uartx_write_text(&TELE_UART, "TEST: OFF\r\n");
  }
  Prueba_manda_bandera();
}

//elige la prueba (el cambio real lo hace Prueba_tarea, con salida y entrada)
void Prueba_estado(uint8_t estado)
{
  if (estado >= PRUEBA_N)
  {
    uartx_write_text(&TELE_UART, "TEST: prueba no existe\r\n");
    return;
  }
  PRUEBA.estado = estado;
  Prueba_manda_bandera();
}

//busca "clave" en la trama y lee el numero que sigue. 1 = la encontro
static uint8_t lee_valor(const char *trama, const char *clave, float *valor)
{
  const char *p = strstr(trama, clave);
  if (p == 0) return 0;
  *valor = strtof(p + strlen(clave), 0);
  return 1;
}

//comandos del test. Devuelve 1 si la trama era uno de ellos
uint8_t Prueba_comando(const char *trama)
{
  float v;

  if (strstr(trama, "TESTON"))  { Prueba_activa(1); return 1; }
  if (strstr(trama, "TESTOFF")) { Prueba_activa(0); return 1; }

  if (lee_valor(trama, "TESTSEL=", &v))
  {
    Prueba_estado((uint8_t)(v + 0.5f));
    return 1;
  }

  if (lee_valor(trama, "PWMIZQ=", &v)) { PRUEBA.pwm_izq = v; Prueba_aplica_pwm(); return 1; }
  if (lee_valor(trama, "PWMDER=", &v)) { PRUEBA.pwm_der = v; Prueba_aplica_pwm(); return 1; }

  //STOP: apaga tambien el test, pero devuelve 0 para que Llanta_comando lo procese
  if (strstr(trama, "STOP") && PRUEBA.activo) Prueba_activa(0);

  return 0;
}

//maquina de estados del test. Llamar en el while
void Prueba_tarea(void)
{
  uint32_t ahora = HAL_GetTick();

  //la bandera se avisa siempre (este o no en TEST)
  if (ahora - PRUEBA.t_bandera >= PRUEBA_BANDERA_MS)
  {
    PRUEBA.t_bandera = ahora;
    Prueba_manda_bandera();
  }

  if (!PRUEBA.activo) return;

  //cambio de estado: salida del anterior y entrada al nuevo
  if (PRUEBA.estado != PRUEBA.estado_ant)
  {
    if (PRUEBA.estado_ant < PRUEBA_N) Prueba_sale(PRUEBA.estado_ant);
    PRUEBA.estado_ant = PRUEBA.estado;
    Prueba_entra(PRUEBA.estado);
    PRUEBA.t_ant = ahora - PRUEBA_PERIODO[PRUEBA.estado];   //corre ya, sin esperar
  }

  //cada prueba corre a su periodo
  uint16_t periodo = PRUEBA_PERIODO[PRUEBA.estado];
  if ((periodo == 0) || (ahora - PRUEBA.t_ant < periodo)) return;
  PRUEBA.t_ant = ahora;

  switch (PRUEBA.estado)
  {
    case PRUEBA_MOTORES:
      //nada que mandar: el PWM lo aplica la interrupcion (llanta) y
      //las graficas de VELO salen de Llanta_tarea
      break;

    case PRUEBA_ENCODER:
      sprintf(tx, "ENC PUL1=%ld PUL2=%ld RPM1=%.0f RPM2=%.0f VIZQ=%.3f VDER=%.3f PWMI=%.0f PWMD=%.0f\r\n",
              (long)ENCODER1.pulsos, (long)ENCODER2.pulsos, ENCODER1.rpm, ENCODER2.rpm,
              LLANTA_IZQ.velocidad, LLANTA_DER.velocidad, LLANTA_IZQ.pwm, LLANTA_DER.pwm);
      uartx_write_text(&TELE_UART, tx);
      break;

    case PRUEBA_SENSORES:
      Lee_sensores();
      filtra_sensores();
      Normaliza_sensores();
      Manda_Normalizados();          //Nor=... -> LEDs del panel
      break;

    case PRUEBA_IMU:
      sprintf(tx, "*q%.1f*", giro_z_filtrado);   //grados/s -> reloj VEL_ANGULAR
      uartx_write_text(&TELE_UART, tx);
      break;
  }
}
