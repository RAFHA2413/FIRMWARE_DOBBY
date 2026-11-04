/*
 * llanta.c
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "llanta.h"
#include "DRV8871.h"
#include "telemetria.h"
#include "uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LLANTA_PI  3.14159265f
#define LLANTA_TS  (ENCODER_TS_MS / 1000.0f)   //periodo del control en segundos

LLANTA LLANTA_IZQ;
LLANTA LLANTA_DER;
ROBOTS ROBOT = {&LLANTA_IZQ, &LLANTA_DER};

//          modo        v_baja  esc_ms  pwm_ident
LLANTA_CALS LLANTA_CAL = {LLANTA_OFF, 0.30f, 1000, LLANTA_PWM_ID};   //igual que el panel

//las dos llantas, para recorrerlas con un for (0 = izquierda, 1 = derecha)
static LLANTA *const LL[2] = {&LLANTA_IZQ, &LLANTA_DER};
static const char *const LADO[2] = {"IZQ", "DER"};

//tau (en multiplos de Ts) y kaw (factor de Ki/Kp) como los ve el panel
static float vel_tau_ts = 5.0f;
static float vel_kaw_f  = 0.5f;

//identificacion: velocidades grabadas del escalon y pulsos al iniciar la rampa
static float   id_v[2][LLANTA_ID_N];
static int32_t id_pulsos0[2];

static char tx[200];   //texto que se manda al panel (solo desde el while)


//reductora: relacion de la reductora (10 = 10:1)
//diametro_mm: diametro de la llanta en milimetros
void Llanta_init(LLANTA *llanta, ENCODERS *enc, float reductora, float diametro_mm)
{
  llanta->enc = 0;   //mientras sea 0, Llanta_velo no toca esta llanta

  llanta->reductora = reductora;
  llanta->diametro  = diametro_mm / 1000.0f;

  //perimetro de la llanta / reductora = metros por vuelta del motor
  llanta->k_dist = LLANTA_PI * llanta->diametro / reductora;   //vueltas motor -> m
  llanta->k_vel  = llanta->k_dist / 60.0f;                     //rpm motor -> m/s

  llanta->rpm       = 0;
  llanta->velocidad = 0;
  llanta->distancia = 0;

  llanta->pwm         = 0;
  llanta->pwm_ref     = 0;
  llanta->vref        = 0;
  llanta->zona_muerta = -1;

  llanta->enc = enc;   //va al final: habilita la llanta en Llanta_velo
}

//separacion_mm: distancia entre las dos llantas (centro a centro) en milimetros
void Robot_init(float separacion_mm)
{
  ROBOT.separacion = separacion_mm / 1000.0f;
  ROBOT.velocidad  = 0;
  ROBOT.giro       = 0;
  ROBOT.distancia  = 0;
}

//pone en cero las distancias de las llantas y del robot
void Llanta_reset(void)
{
  if (LLANTA_IZQ.enc) Encoder_reset(LLANTA_IZQ.enc);
  if (LLANTA_DER.enc) Encoder_reset(LLANTA_DER.enc);
  LLANTA_IZQ.distancia = 0;
  LLANTA_DER.distancia = 0;
  ROBOT.distancia = 0;
}

//calcula velocidad y distancia de UNA llanta a partir de su encoder
static void Llanta_actualiza(LLANTA *llanta)
{
  if (llanta->enc == 0) return;   //aun no inicializada

  llanta->rpm       = llanta->enc->rpm / llanta->reductora;
  llanta->velocidad = llanta->enc->rpm * llanta->k_vel;
  llanta->distancia = llanta->enc->vueltas * llanta->k_dist;
}


//==============================================================================
//                       MOTOR Y CONTROL DE VELOCIDAD
//==============================================================================

//aplica un PWM con signo al motor de la llanta (+ = adelante)
static void Llanta_motor(LLANTA *ll, float pwm)
{
  if (pwm >  LLANTA_PWM_MAX) pwm =  LLANTA_PWM_MAX;
  if (pwm < -LLANTA_PWM_MAX) pwm = -LLANTA_PWM_MAX;
  ll->pwm = pwm;

  uint32_t p = (uint32_t)(pwm >= 0.0f ? pwm : -pwm);

  //PWM 0 = motor LIBRE (con DRV8871_FRENO, Adelante(0) lo dejaria frenado)
  if (p == 0)
  {
    if (ll == &LLANTA_IZQ) MI_Stop();
    else                   MD_Stop();
    return;
  }

  if (ll == &LLANTA_IZQ)
  {
    if (pwm >= 0.0f) MI_Adelante(p);
    else             MI_Atras(p);
  }
  else
  {
    if (pwm >= 0.0f) MD_Adelante(p);
    else             MD_Atras(p);
  }
}

//un paso del PID de velocidad de una llanta (el setpoint ya esta puesto)
static void Llanta_pid(LLANTA *ll)
{
  Llanta_motor(ll, PID_Control(ll->pid, ll->velocidad));
}

//identificacion: espera quietas -> rampa (zona muerta) -> pausa -> escalon de PWM grabando la velocidad
static void Llanta_ident_paso(void)
{
  LLANTA_CALS *c = &LLANTA_CAL;

  switch (c->fase)
  {
    case 3:   //espera a que las llantas esten QUIETAS (ej. si venian girando del escalon)
    {
      uint8_t quietas = 1;
      for (uint8_t i = 0; i < 2; i++)
      {
        Llanta_motor(LL[i], 0.0f);
        float v = LL[i]->velocidad;
        if ((v > LLANTA_V_QUIETA) || (v < -LLANTA_V_QUIETA)) quietas = 0;
      }
      c->rampa += (float)ENCODER_TS_MS;                 //(uso rampa como reloj total en ms)
      if (quietas) c->cuenta++;
      else         c->cuenta = 0;

      //quietas durante LLANTA_PAUSA_MS seguidos (o 3 s maximo) -> empieza la rampa
      if ((c->cuenta * ENCODER_TS_MS >= LLANTA_PAUSA_MS) || (c->rampa >= 3000.0f))
      {
        for (uint8_t i = 0; i < 2; i++) id_pulsos0[i] = LL[i]->enc->pulsos;
        c->rampa  = 0.0f;
        c->cuenta = 0;
        c->fase   = 0;
      }
      break;
    }

    case 0:   //rampa: sube el PWM hasta que cada llanta arranca
    {
      uint8_t faltan = 0;
      c->rampa += LLANTA_ZM_PASO;

      for (uint8_t i = 0; i < 2; i++)
      {
        LLANTA *ll = LL[i];
        if (ll->zona_muerta >= 0.0f) continue;          //ya arranco: queda apagada

        int32_t mov = ll->enc->pulsos - id_pulsos0[i];
        if (mov < 0) mov = -mov;

        if (mov >= LLANTA_ZM_PULSOS)
        {
          ll->zona_muerta = c->rampa;                   //arranco con este PWM
          Llanta_motor(ll, 0.0f);
        }
        else
        {
          Llanta_motor(ll, c->rampa);
          faltan = 1;
        }
      }

      if (!faltan || (c->rampa >= LLANTA_ZM_MAX))       //ya arrancaron las 2 (o no arrancan)
      {
        Llanta_motor(&LLANTA_IZQ, 0.0f);
        Llanta_motor(&LLANTA_DER, 0.0f);
        c->fase = 1;
        c->cuenta = 0;
      }
      break;
    }

    case 1:   //pausa con el motor apagado: que las llantas se detengan
      c->cuenta++;
      if (c->cuenta * ENCODER_TS_MS >= LLANTA_PAUSA_MS)
      {
        c->fase = 2;
        c->cuenta = 0;
      }
      break;

    case 2:   //escalon de PWM: graba la velocidad de cada muestra
      for (uint8_t i = 0; i < 2; i++)
      {
        Llanta_motor(LL[i], c->pwm_ident);
        id_v[i][c->cuenta] = LL[i]->velocidad;
      }
      c->cuenta++;
      if (c->cuenta >= LLANTA_ID_N)                     //termino: el while calcula y reporta
      {
        Llanta_motor(&LLANTA_IZQ, 0.0f);
        Llanta_motor(&LLANTA_DER, 0.0f);
        c->modo  = LLANTA_OFF;
        c->listo = 1;
      }
      break;
  }
}

//se llama en la interrupcion cada vez que hay una muestra nueva del encoder (ENCODER_TS_MS)
static void Llanta_control(void)
{
  uint8_t modo = LLANTA_CAL.modo;

  if (modo == LLANTA_OFF) return;
  if ((LLANTA_IZQ.pid == 0) || (LLANTA_DER.pid == 0)) return;   //falta Llanta_control_init
  if ((LLANTA_IZQ.enc == 0) || (LLANTA_DER.enc == 0)) return;   //falta Llanta_init

  switch (modo)
  {
    case LLANTA_PWM:
      Llanta_motor(&LLANTA_IZQ, LLANTA_IZQ.pwm_ref);
      Llanta_motor(&LLANTA_DER, LLANTA_DER.pwm_ref);
      break;

    case LLANTA_PID:
      LLANTA_IZQ.pid->setpoint = LLANTA_IZQ.vref;
      LLANTA_DER.pid->setpoint = LLANTA_DER.vref;
      Llanta_pid(&LLANTA_IZQ);
      Llanta_pid(&LLANTA_DER);
      break;

    case LLANTA_ESCALON:
      LLANTA_CAL.cuenta++;
      if (LLANTA_CAL.cuenta * ENCODER_TS_MS >= LLANTA_CAL.esc_ms)
      {
        LLANTA_CAL.cuenta = 0;
        LLANTA_CAL.alto = !LLANTA_CAL.alto;              //cambia de nivel
      }
      LLANTA_IZQ.pid->setpoint = LLANTA_CAL.alto ? LLANTA_IZQ.vref : LLANTA_CAL.v_baja;
      LLANTA_DER.pid->setpoint = LLANTA_CAL.alto ? LLANTA_DER.vref : LLANTA_CAL.v_baja;
      Llanta_pid(&LLANTA_IZQ);
      Llanta_pid(&LLANTA_DER);
      break;

    case LLANTA_IDENT:
      Llanta_ident_paso();
      break;
  }
}

//Lo llamo cada 1 ms en (SysTick_Handler), despues de Encoder_velo()
void Llanta_velo(void)
{
  Llanta_actualiza(&LLANTA_IZQ);
  Llanta_actualiza(&LLANTA_DER);

  //robot (modelo diferencial)
  ROBOT.velocidad = (LLANTA_IZQ.velocidad + LLANTA_DER.velocidad) * 0.5f;
  ROBOT.distancia = (LLANTA_IZQ.distancia + LLANTA_DER.distancia) * 0.5f;
  if (ROBOT.separacion > 0)
    ROBOT.giro = (LLANTA_DER.velocidad - LLANTA_IZQ.velocidad) / ROBOT.separacion;

  //control de velocidad: solo cuando el encoder saco una muestra nueva
  if (encoder_muestra)
  {
    encoder_muestra = 0;
    Llanta_control();
  }
}


//==============================================================================
//                      FUNCIONES PARA EL MAIN / EL PANEL
//==============================================================================

//inicia los PID de velocidad (valores de PID.h) y los asocia a cada llanta
//llamar DESPUES de Llanta_init
void Llanta_control_init(void)
{
  LLANTA_CAL.modo = LLANTA_OFF;

  PID_VEL_INIT();
  LLANTA_IZQ.pid = &PID_VEL_IZQ;
  LLANTA_DER.pid = &PID_VEL_DER;

  //factores que muestra el panel, sacados de los #define de PID.h
  vel_tau_ts = PID_VEL_TAU / PID_VEL_TS;
  if ((PID_VEL_KI > 0.0f) && (PID_VEL_KP > 0.0f))
    vel_kaw_f = PID_VEL_KAW * PID_VEL_KP / PID_VEL_KI;

  Llanta_modo(LLANTA_OFF);

  //el PID calcula con PID_VEL_TS pero se llama cada ENCODER_TS_MS: deben ser iguales
  float dif = PID_VEL_TS * 1000.0f - (float)ENCODER_TS_MS;
  if ((dif > 0.01f) || (dif < -0.01f))
    uartx_write_text(&TELE_UART, "LLANTA: OJO PID_VEL_TS (PID.h) distinto de ENCODER_TS_MS (encoder.h)\r\n");
}

//cambia el modo del control. Siempre pasa por apagado: motores en 0 y PID en cero
void Llanta_modo(uint8_t modo)
{
  LLANTA_CAL.modo = LLANTA_OFF;   //primero: la interrupcion deja de mover los motores

  for (uint8_t i = 0; i < 2; i++)
  {
    LLANTA *ll = LL[i];
    Llanta_motor(ll, 0.0f);
    ll->pwm_ref = 0.0f;
    if (ll->pid)
    {
      PID_Reset(ll->pid);
      ll->pid->setpoint = 0.0f;
    }
  }

  LLANTA_CAL.cuenta = 0;
  LLANTA_CAL.fase   = 0;
  LLANTA_CAL.alto   = 1;
  LLANTA_CAL.rampa  = 0.0f;

  if (modo == LLANTA_OFF) return;
  if ((LLANTA_IZQ.pid == 0) || (LLANTA_DER.pid == 0)) return;   //falta Llanta_control_init
  if ((LLANTA_IZQ.enc == 0) || (LLANTA_DER.enc == 0)) return;   //falta Llanta_init

  if (modo == LLANTA_IDENT)
  {
    for (uint8_t i = 0; i < 2; i++) LL[i]->zona_muerta = -1.0f;
    LLANTA_CAL.fase  = 3;    //primero espera a que las llantas esten quietas
    LLANTA_CAL.listo = 0;
  }

  LLANTA_CAL.modo = modo;   //al final: habilita el modo en la interrupcion
}

//velocidad deseada de cada llanta en m/s (modo LLANTA_PID y nivel alto del escalon)
void Llanta_set_velocidad(float v_izq, float v_der)
{
  LLANTA_IZQ.vref = v_izq;
  LLANTA_DER.vref = v_der;
}

//cambia las constantes de los DOS PID de velocidad sin borrar su estado
// tau_ts: filtro derivativo en multiplos de Ts (ej. 5)
// kaw_f : anti-windup como factor de Ki/Kp (ej. 0.5)
void Llanta_sintoniza(float kp, float ki, float kd, float tau_ts, float kaw_f)
{
  if (kp < 0.0f) kp = 0.0f;
  if (ki < 0.0f) ki = 0.0f;
  if (kd < 0.0f) kd = 0.0f;
  vel_tau_ts = tau_ts;
  vel_kaw_f  = kaw_f;

  float kaw = (kp > 0.0f) ? kaw_f * ki / kp : 0.0f;
  float tau = tau_ts * PID_VEL_IZQ.Ts;

  //sin interrupciones: que el PID no calcule con constantes a medio cambiar
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  PID_Sintoniza(&PID_VEL_IZQ, kp, ki, kd, tau, kaw);
  PID_Sintoniza(&PID_VEL_DER, kp, ki, kd, tau, kaw);
  __set_PRIMASK(primask);
}

//PWM directo (lazo abierto) a las dos llantas (-1000..1000, + = adelante)
//pasa a modo LLANTA_PWM si hace falta. La usa la libreria pruebas (TEST ON)
void Llanta_set_pwm(float pwm_izq, float pwm_der)
{
  if (LLANTA_CAL.modo != LLANTA_PWM) Llanta_modo(LLANTA_PWM);
  if (LLANTA_CAL.modo != LLANTA_PWM) return;   //no se pudo (falta init)

  if (pwm_izq >  LLANTA_PWM_MAX) pwm_izq =  LLANTA_PWM_MAX;
  if (pwm_izq < -LLANTA_PWM_MAX) pwm_izq = -LLANTA_PWM_MAX;
  if (pwm_der >  LLANTA_PWM_MAX) pwm_der =  LLANTA_PWM_MAX;
  if (pwm_der < -LLANTA_PWM_MAX) pwm_der = -LLANTA_PWM_MAX;
  LLANTA_IZQ.pwm_ref = pwm_izq;
  LLANTA_DER.pwm_ref = pwm_der;
}

//busca "clave" en la trama y lee el numero que sigue. 1 = la encontro
static uint8_t lee_valor(const char *trama, const char *clave, float *valor)
{
  const char *p = strstr(trama, clave);
  if (p == 0) return 0;
  *valor = strtof(p + strlen(clave), 0);
  return 1;
}

//procesa los comandos del PID de velocidad que manda el panel (ver llanta.h)
//devuelve 1 si la trama era uno de estos comandos
uint8_t Llanta_comando(const char *trama)
{
  float v;
  const PID_ *p = &PID_VEL_DER;   //los dos PID tienen las mismas constantes

  //apagar va primero (STOP del panel, VSTOP de la pestaña VELO)
  if (strstr(trama, "STOP"))
  {
    Llanta_modo(LLANTA_OFF);
    return 1;
  }

  //constantes: cambia una y deja las demas; responde con los valores que quedaron
  uint8_t constante = 1;
  if (lee_valor(trama, "VKP=", &v))
    Llanta_sintoniza(v, p->Ki, p->Kd, vel_tau_ts, vel_kaw_f);
  else if (lee_valor(trama, "VKI=", &v))
    Llanta_sintoniza(p->Kp, v, p->Kd, vel_tau_ts, vel_kaw_f);
  else if (lee_valor(trama, "VKD=", &v))
    Llanta_sintoniza(p->Kp, p->Ki, v, vel_tau_ts, vel_kaw_f);
  else if (lee_valor(trama, "VTAU=", &v))
    Llanta_sintoniza(p->Kp, p->Ki, p->Kd, v, vel_kaw_f);
  else if (lee_valor(trama, "VKAW=", &v))
    Llanta_sintoniza(p->Kp, p->Ki, p->Kd, vel_tau_ts, v);
  else
    constante = 0;

  if (constante)
  {
    manda_pid_parametros();   //el panel ve en rojo los valores que quedaron
    return 1;
  }

  //set point (m/s)
  if (lee_valor(trama, "VIZQ=", &v)) { LLANTA_IZQ.vref = v; return 1; }
  if (lee_valor(trama, "VDER=", &v)) { LLANTA_DER.vref = v; return 1; }

  //PWM directo (PWMIZQ= / PWMDER=): lo maneja la libreria pruebas, solo con TEST ON

  //modos
  if (lee_valor(trama, "VPID=", &v)) { Llanta_modo(v > 0.5f ? LLANTA_PID : LLANTA_OFF); return 1; }
  if (lee_valor(trama, "VESC=", &v)) { Llanta_modo(v > 0.5f ? LLANTA_ESCALON : LLANTA_OFF); return 1; }

  //ajustes del escalon
  if (lee_valor(trama, "VBAJA=", &v))  { LLANTA_CAL.v_baja = v; return 1; }
  if (lee_valor(trama, "VESCMS=", &v))
  {
    if (v >= (float)ENCODER_TS_MS) LLANTA_CAL.esc_ms = (uint32_t)v;
    return 1;
  }

  //identificacion automatica
  if (lee_valor(trama, "VIDENT=", &v))
  {
    if (v > LLANTA_PWM_MAX) v = LLANTA_PWM_MAX;
    LLANTA_CAL.pwm_ident = (v > 0.0f) ? v : LLANTA_PWM_ID;
    sprintf(tx, "IDENT: inicia (rampa hasta %.0f, escalon de PWM %.0f)\r\n",
            LLANTA_ZM_MAX, LLANTA_CAL.pwm_ident);
    uartx_write_text(&TELE_UART, tx);
    Llanta_modo(LLANTA_IDENT);
    return 1;
  }

  return 0;
}

//calcula zona muerta, K, tau y el PI sugerido de una llanta y lo manda al panel
static void Llanta_calcula(uint8_t i)
{
  LLANTA *ll = LL[i];
  const float ts = LLANTA_TS;

  ll->k = 0; ll->tau = 0; ll->kp_sug = 0; ll->ki_sug = 0;

  if (ll->zona_muerta < 0.0f)
  {
    sprintf(tx, "IDENT %s: ERROR no arranco con PWM %.0f (revisar motor, driver o encoder)\r\n",
            LADO[i], LLANTA_ZM_MAX);
    uartx_write_text(&TELE_UART, tx);
    return;
  }

  //velocidad final = promedio de las ultimas muestras
  float vf = 0.0f;
  for (uint16_t n = LLANTA_ID_N - LLANTA_ID_FINAL; n < LLANTA_ID_N; n++) vf += id_v[i][n];
  vf /= LLANTA_ID_FINAL;

  if (vf < 0.0f)
  {
    sprintf(tx, "IDENT %s: ERROR velocidad NEGATIVA con PWM adelante -> invertir ENCODERx_VEL y "
                "ENCODERx_DIR de esta llanta (encoder.c)\r\n", LADO[i]);
    uartx_write_text(&TELE_UART, tx);
    return;
  }
  if (vf < 0.05f)
  {
    sprintf(tx, "IDENT %s: ERROR casi no se movio (%.3f m/s): subir PWM_id\r\n", LADO[i], vf);
    uartx_write_text(&TELE_UART, tx);
    return;
  }

  float pwm_util = LLANTA_CAL.pwm_ident - ll->zona_muerta;
  if (pwm_util < 10.0f)
  {
    sprintf(tx, "IDENT %s: ERROR PWM_id (%.0f) debe ser mayor que la zona muerta (%.0f)\r\n",
            LADO[i], LLANTA_CAL.pwm_ident, ll->zona_muerta);
    uartx_write_text(&TELE_UART, tx);
    return;
  }

  //metodo del area: integral de (vf - v) = vf * tau   (aguanta bien el ruido del encoder)
  float area = 0.0f;
  for (uint16_t n = 0; n < LLANTA_ID_N; n++) area += vf - id_v[i][n];
  float tau = area * ts / vf;
  if (tau < ts) tau = ts;

  float k  = vf / pwm_util;                      //m/s por unidad de PWM
  float tc = LLANTA_TC_FACTOR * tau;             //constante de tiempo deseada en lazo cerrado
  if (tc < 4.0f * ts) tc = 4.0f * ts;

  ll->k      = k;
  ll->tau    = tau;
  ll->kp_sug = tau / (k * tc);
  ll->ki_sug = 1.0f / (k * tc);

  //trama para el panel:  IDENTIZQ=zona_muerta/K/tau/Kp/Ki/
  sprintf(tx, "IDENT%s=%.0f/%.6f/%.4f/%.2f/%.2f/\r\n",
          LADO[i], ll->zona_muerta, ll->k, ll->tau, ll->kp_sug, ll->ki_sug);
  uartx_write_text(&TELE_UART, tx);

  //si no alcanzo a estabilizarse, el tau sale corto
  if (tau * 5.0f > LLANTA_ID_N * ts)
  {
    sprintf(tx, "IDENT %s: OJO tau grande (%.3f s): aumentar LLANTA_ID_N\r\n", LADO[i], tau);
    uartx_write_text(&TELE_UART, tx);
  }
}

//llamar en el while: manda la telemetria para las graficas del panel y,
//cuando termina la identificacion, calcula y reporta los resultados
void Llanta_tarea(void)
{
  static uint32_t t_rapida = 0, t_lenta = 0;
  uint32_t ahora = HAL_GetTick();

  if (LLANTA_CAL.listo)
  {
    LLANTA_CAL.listo = 0;
    uartx_write_text(&TELE_UART, "IDENT fin: zona_muerta/K(m/s por PWM)/tau(s)/Kp/Ki\r\n");
    Llanta_calcula(0);
    Llanta_calcula(1);
  }

  //trama rapida: set point y velocidad de cada llanta (graficas)
  if (ahora - t_rapida >= LLANTA_TELE_MS)
  {
    t_rapida = ahora;
    float sp_izq = LLANTA_IZQ.pid ? LLANTA_IZQ.pid->setpoint : 0.0f;
    float sp_der = LLANTA_DER.pid ? LLANTA_DER.pid->setpoint : 0.0f;
    snprintf(tx, sizeof(tx), "*SETIZ%.2f**SETDE%.2f**w%.3f**z%.3f*",
             sp_izq, sp_der, LLANTA_IZQ.velocidad, LLANTA_DER.velocidad);
    uartx_write_text(&TELE_UART, tx);
  }

  //trama lenta: PWM, partes de cada PID y modo
  if ((ahora - t_lenta >= LLANTA_TELE_LENTA_MS) && LLANTA_IZQ.pid && LLANTA_DER.pid)
  {
    t_lenta = ahora;
    snprintf(tx, sizeof(tx),
             "*PIDIZQ%.0f**PIDDER%.0f*"
             "*UPVELIZQ%.1f**UIVELIZQ%.1f**UDVELIZQ%.1f*"
             "*UPVELDER%.1f**UIVELDER%.1f**UDVELDER%.1f*"
             "*VMODO%d*",
             LLANTA_IZQ.pwm, LLANTA_DER.pwm,
             LLANTA_IZQ.pid->up, LLANTA_IZQ.pid->ui, LLANTA_IZQ.pid->ud,
             LLANTA_DER.pid->up, LLANTA_DER.pid->ui, LLANTA_DER.pid->ud,
             (int)LLANTA_CAL.modo);
    uartx_write_text(&TELE_UART, tx);
  }
}
