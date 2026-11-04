/*
 * carrera.c
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "carrera.h"
#include "llanta.h"
#include "PID.h"
#include "parametros.h"
#include "IR_ANA_SEN.h"
#include "DRV8871.h"
#include "pruebas.h"
#include "telemetria.h"
#include "uart.h"
#include "timer.h"          //giro_z_filtrado (IMU, se lee en el TIM6 antes de Carrera_control)
#include <stdio.h>

CARRERAS CARRERA = {CARRERA_REPOSO};

static char tx[40];


//velocidad base pedida (V_Bas del panel), limitada por seguridad
static float Carrera_v_base(void)
{
  float v = PARAM.velo_basica;                         //PWM 0..1000
  if (v < 0.0f) v = 0.0f;
  if (v > CARRERA_PWM_MAX) v = CARRERA_PWM_MAX;
  return v;
}

//(opcional, CARRERA_KV_CURVA) velocidad base segun el IMU: si gira rapido (curva) baja
//v = V_Bas - KV_CURVA * |giro_z|, sin bajar de CARRERA_V_MIN_CURVA
static float Carrera_v_curva(float v_base)
{
  if (CARRERA_KV_CURVA <= 0.0f) return v_base;         //apagado

  float g = giro_z_filtrado;                           //grados/s
  if (g < 0.0f) g = -g;

  float v = v_base - CARRERA_KV_CURVA * g;
  float v_min = (CARRERA_V_MIN_CURVA < v_base) ? CARRERA_V_MIN_CURVA : v_base;
  if (v < v_min) v = v_min;
  return v;
}

//sensores -> posicion de la linea (0 = centrada)
static void Carrera_lee_linea(void)
{
  Lee_sensores();
  Normaliza_sensores();
  CARRERA.posicion = CARRERA_SIGNO * Calcula_Pesos();
}


//se llama una vez al iniciar
void Carrera_init(void)
{
  CARRERA.estado   = CARRERA_REPOSO;
  CARRERA.cuenta   = 0;
  CARRERA.giro     = 0.0f;
  CARRERA.v_base   = 0.0f;
  CARRERA.frenando = 0;
}

//REPOSO -> ARRANQUE -> PID   (orden RUN por serial o START del control IR)
void Carrera_run(void)
{
  if (CARRERA.estado != CARRERA_REPOSO) return;        //ya esta corriendo

  if (PRUEBA.activo) Prueba_activa(0);                 //el test no puede mover motores en carrera

  CARRERA.frenando = 0;
  PID_Reset(&PID_DIR);
  PID_DIR.setpoint = 0.0f;                             //linea centrada
  CARRERA.giro = 0.0f;
  CARRERA.cuenta = 0;
  CARRERA.sin_linea = 0;
  CARRERA.aviso_perdido = 0;

  //rampa: arranca de una vez en V_INICIAL (rompe la inercia), sin pasarse de V_Bas
  float v_obj = Carrera_v_base();
  CARRERA.v_base = (CARRERA_V_INICIAL < v_obj) ? CARRERA_V_INICIAL : v_obj;

  Llanta_set_velocidad(0.0f, 0.0f);
  Llanta_modo(LLANTA_PID);                             //enciende los PID de velocidad

  CARRERA.estado = CARRERA_ARRANQUE;                   //al final: habilita Carrera_control
  uartx_write_text(&TELE_UART, "CARRERA: ARRANQUE\r\n");
}

//pasa a REPOSO frenando. Sin uart: se puede llamar tambien desde la interrupcion
static void Carrera_frena(void)
{
  CARRERA.estado = CARRERA_REPOSO;                     //primero: Carrera_control deja de mandar
  CARRERA.frenando = 0;                                //(que la interrupcion no suelte el freno)

  Llanta_modo(LLANTA_OFF);                             //apaga los PID de velocidad

  CARRERA.giro = 0.0f;
  CARRERA.v_base = 0.0f;
  CARRERA.sin_linea = 0;
  CARRERA.cuenta = 0;
  MD_Freno();                                          //freno activo (los dos pines en 1)
  MI_Freno();
  CARRERA.frenando = 1;                                //al final: cuenta CARRERA_FRENO_MS y suelta
}

//cualquier estado -> REPOSO frenando (orden STOP por serial o STOP del control IR)
void Carrera_stop(void)
{
  if (PRUEBA.activo) Prueba_activa(0);                 //STOP tambien apaga el test
  Carrera_frena();
  uartx_write_text(&TELE_UART, "CARRERA: REPOSO\r\n");
}

//un paso de seguir la linea con la velocidad base v_base:
//sensores -> seguridad (linea perdida) -> PID de linea -> velocidad de cada llanta
//con_giro = 0: calcula el PID pero no corrige (va recto).  Devuelve 0 si tuvo que parar
static uint8_t Carrera_sigue_linea(float v_base, uint8_t con_giro)
{
  Carrera_lee_linea();

  //SEGURIDAD: si no ve la linea CARRERA_PERDIDO_MS seguidos (se salio de la pista) -> para
  if (flag_linea) CARRERA.sin_linea = 0;
  else if (++CARRERA.sin_linea * CARRERA_TS_MS >= CARRERA_PERDIDO_MS)
  {
    Carrera_frena();
    CARRERA.aviso_perdido = 1;                         //el aviso por uart lo da Carrera_tarea
    return 0;
  }

  //PID de linea: entrada = posicion, salida = diferencia en PWM (limite V_Gir)
  CARRERA.giro = PID_Control(&PID_DIR, CARRERA.posicion);

  //IMU: amortiguador del giro. Si ya gira rapido, corrige menos (KG = cuadro KG del panel)
  CARRERA.giro -= PARAM.kg * CARRERA_SIGNO_GIRO * giro_z_filtrado;
  if (CARRERA.giro >  PARAM.vel_giro) CARRERA.giro =  PARAM.vel_giro;   //tope V_Gir
  if (CARRERA.giro < -PARAM.vel_giro) CARRERA.giro = -PARAM.vel_giro;

  float g = con_giro ? CARRERA.giro : 0.0f;

  //linea a la izquierda (posicion < 0) -> giro > 0 -> izquierda mas lenta: gira a la izquierda
  float izq = v_base - g;
  float der = v_base + g;
  if (izq >  CARRERA_PWM_MAX) izq =  CARRERA_PWM_MAX;
  if (izq < -CARRERA_PWM_MAX) izq = -CARRERA_PWM_MAX;
  if (der >  CARRERA_PWM_MAX) der =  CARRERA_PWM_MAX;
  if (der < -CARRERA_PWM_MAX) der = -CARRERA_PWM_MAX;

  //PWM equivalente -> m/s para el PID de velocidad de cada rueda
  Llanta_set_velocidad(izq * CARRERA_MPS_POR_PWM, der * CARRERA_MPS_POR_PWM);
  return 1;
}

//cada 5 ms en la interrupcion del TIM6 (TIMER_PROCEDIMIENTOS)
void Carrera_control(void)
{
  CARRERA.cuenta++;

  switch (CARRERA.estado)
  {
    case CARRERA_REPOSO:
      //despues de frenar un rato suelta los motores (para poder moverlo con la mano)
      if (CARRERA.frenando && (CARRERA.cuenta * CARRERA_TS_MS >= CARRERA_FRENO_MS))
      {
        CARRERA.frenando = 0;
        MD_Stop();
        MI_Stop();
      }
      break;

    case CARRERA_ARRANQUE:
    {
      //rampa: sube la velocidad base CARRERA_ACEL (PWM/s) hasta V_Bas
      float v_obj = Carrera_v_base();
      CARRERA.v_base += CARRERA_ACEL * (CARRERA_TS_MS / 1000.0f);
      uint8_t llego = (CARRERA.v_base >= v_obj);
      if (llego) CARRERA.v_base = v_obj;

      if (!Carrera_sigue_linea(CARRERA.v_base, CARRERA_GIRO_ARRANQUE)) break;   //se detuvo

      if (llego)                                       //termino la rampa -> PID normal
      {
        CARRERA.estado = CARRERA_PID;
        CARRERA.cuenta = 0;
      }
      break;
    }

    case CARRERA_PID:
      CARRERA.v_base = Carrera_v_curva(Carrera_v_base());   //V_Bas, mas lento si gira rapido (IMU)
      Carrera_sigue_linea(CARRERA.v_base, 1);

      //AQUI IRAN LAS TRANSICIONES: PID -> RECTA, PID -> PERDIDO ...
      break;

    default:   //estados pendientes: por seguridad vuelve a PID
      CARRERA.estado = CARRERA_PID;
      break;
  }
}

//en el while: telemetria de carrera para el panel
void Carrera_tarea(void)
{
  static uint32_t t_tele = 0, t_est = 0;
  uint32_t ahora = HAL_GetTick();

  if (CARRERA.aviso_perdido)
  {
    CARRERA.aviso_perdido = 0;
    uartx_write_text(&TELE_UART, "CARRERA: LINEA PERDIDA -> REPOSO (seguridad)\r\n");
  }

  //estado: siempre, cada 500 ms (el panel pinta el boton RUN/STOP y el indicador)
  if (ahora - t_est >= 500)
  {
    t_est = ahora;
    sprintf(tx, "*EST%d*", (int)CARRERA.estado);
    uartx_write_text(&TELE_UART, tx);
  }

  //en carrera: posicion, partes del PID de linea, giro y velocidades.
  //cada 4 envios (200 ms) tambien los 16 sensores (LEDs del panel)
  static uint8_t n_tele = 0;
  if ((CARRERA.estado != CARRERA_REPOSO) && (ahora - t_tele >= CARRERA_TELE_MS))
  {
    t_tele = ahora;
    n_tele++;
    manda_operando((n_tele & 3) == 0);
  }
}
