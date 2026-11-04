/*
 * PID.c
 *  Created on: Nov 2, 2024
 * Autor: Alcides Ramos
 */

#include "PID.h"

//PIDs usados en el robot
PID_  PID_DIR;
PID_  PID_VEL_IZQ;
PID_  PID_VEL_DER;

//se debe colocar un pid_init pro cada pid usado
void PID_DIR_INIT(void)
{
  PID_Init(&PID_DIR, PID_DIR_KP, PID_DIR_KI, PID_DIR_KD, PID_DIR_TS,
           -PID_DIR_MAX, PID_DIR_MAX, PID_DIR_TAU, PID_DIR_KAW);
}

void PID_VEL_INIT(void)
{
  PID_Init(&PID_VEL_IZQ, PID_VEL_KP, PID_VEL_KI, PID_VEL_KD, PID_VEL_TS,
           -PID_VEL_MAX, PID_VEL_MAX, PID_VEL_TAU, PID_VEL_KAW);
  PID_Init(&PID_VEL_DER, PID_VEL_KP, PID_VEL_KI, PID_VEL_KD, PID_VEL_TS,
           -PID_VEL_MAX, PID_VEL_MAX, PID_VEL_TAU, PID_VEL_KAW);
}

//colocar mas INIT por cada PID usado

void PID_Sintoniza(PID_ *pid, float Kp, float Ki, float Kd,
                   float Tau, float Kaw)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;

  //filtro derivativo: Tau minimo = Ts (con Tau < Ts/2 la derivada oscila)
  if (Tau < pid->Ts) Tau = pid->Ts;
  pid->Tau = Tau;

  //anti-windup en 1/s (ej. 0.3*Ki/Kp, ver PID.h)
  if (Kaw < 0.0f) Kaw = 0.0f;
  pid->Kaw = Kaw;

  //derivada filtrada (Tustin) sobre la medicion:
  // D[k] = cd1*(input - prev_input) + cd2*D[k-1]
  float den = 2.0f * pid->Tau + pid->Ts;
  pid->cd1 = -2.0f * pid->Kd / den;
  pid->cd2 = (2.0f * pid->Tau - pid->Ts) / den;
}

void PID_Init(PID_ *pid, float Kp, float Ki, float Kd, float Ts,
              float out_min, float out_max, float Tau, float Kaw)
{
  pid->Ts = Ts;
  pid->out_min = out_min;
  pid->out_max = out_max;
  pid->setpoint = 0.0f;

  PID_Sintoniza(pid, Kp, Ki, Kd, Tau, Kaw);
  PID_Reset(pid);
}

void PID_Reset(PID_ *pid)
{
  pid->error = 0.0f;
  pid->prev_error = 0.0f;
  pid->up = 0.0f;
  pid->ui = 0.0f;
  pid->ud = 0.0f;
  pid->output = 0.0f;
  pid->prev_input = 0.0f;
  pid->aw_term = 0.0f;
  pid->primera = 1;      //el primer calculo toma la medicion actual como anterior
}

//limita v entre min y max
static float PID_limita(float v, float min, float max)
{
  if (v > max) return max;
  if (v < min) return min;
  return v;
}

float PID_Control(PID_ *pid, float input)
{
  pid->error = pid->setpoint - input;

  //primer calculo: sin historia, evita el golpe de la derivada al arrancar
  if (pid->primera)
  {
    pid->prev_input = input;
    pid->prev_error = pid->error;
    pid->primera = 0;
  }

  //  Parte proporcional
  pid->up = pid->Kp * pid->error;

  //  Parte integral (Tustin) + back-calculation
  pid->ui += (pid->Ki * pid->Ts * 0.5f) * (pid->error + pid->prev_error)
           + pid->Kaw * pid->aw_term * pid->Ts;
  //  seguro: la integral sola nunca pasa los limites
  pid->ui = PID_limita(pid->ui, pid->out_min, pid->out_max);

  //  Parte derivativa filtrada (Tustin) sobre la medicion
  pid->ud = pid->cd1 * (input - pid->prev_input) + pid->cd2 * pid->ud;
  pid->prev_input = input;

  //  salida PID
  float output = pid->up + pid->ui + pid->ud;
  float output_sat = PID_limita(output, pid->out_min, pid->out_max);

  //  cuanto se satura (para el anti-windup del siguiente ciclo)
  pid->aw_term = output_sat - output;

  pid->prev_error = pid->error;
  pid->output = output_sat;

  return pid->output;
}

// Version basica del PID
float PID_Control_Basic(PID_ *pid, float input)
{
  pid->error = pid->setpoint - input;

  if (pid->primera)
  {
    pid->prev_error = pid->error;
    pid->primera = 0;
  }

  //  Parte proporcional
  pid->up = pid->Kp * pid->error;

  //  Parte derivativa
  pid->ud = pid->Kd * (pid->error - pid->prev_error) / pid->Ts;

  //  salida temporal para saber si se satura y si se debe integrar o no
  float output_candidato = pid->up + pid->ui + pid->ud;

  //  se bloquea la integral si YA esta saturada Y el error sigue empujando hacia
  //  ese mismo lado (si el error ya cambio de signo, se deja integrar
  //  para salir de la saturacion mas rapido)
  uint8_t saturado_arriba = (output_candidato > pid->out_max) && (pid->error > 0.0f);
  uint8_t saturado_abajo  = (output_candidato < pid->out_min) && (pid->error < 0.0f);

  //  Parte integral calculada solo si no se satura la salida
  if (!(saturado_arriba || saturado_abajo))
  {
    pid->ui += pid->Ki * pid->Ts * pid->error;
  }

  //  salida PID (con ui ya actualizado o congelado segun el caso)
  float output = pid->up + pid->ui + pid->ud;

  pid->prev_error = pid->error;
  pid->output = PID_limita(output, pid->out_min, pid->out_max);

  return pid->output;
}
