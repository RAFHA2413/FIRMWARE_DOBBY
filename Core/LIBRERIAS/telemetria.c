/*
 * telemetria.c
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "telemetria.h"
#include "PID.h"
#include "parametros.h"
#include "uart.h"
#include "llanta.h"
#include "IR_ANA_SEN.h"
#include "MPU6050.h"
#include "carrera.h"
#include <stdio.h>

//buffer donde se arma la trama completa antes de enviarla
static char tele[600];
static int  tele_n;

//agrega texto al buffer sin pasarse de su tamaño
#define AGREGA(...)                                                          \
  do {                                                                       \
    if (tele_n < (int)sizeof(tele))                                          \
      tele_n += snprintf(tele + tele_n, sizeof(tele) - (size_t)tele_n, __VA_ARGS__); \
  } while (0)


//Tau en multiplos de Ts (lo que se ve en el panel, ej. 5)
static float factor_tau(const PID_ *pid)
{
  return (pid->Ts > 0.0f) ? pid->Tau / pid->Ts : 0.0f;
}

//Kaw como factor de Ki/Kp (lo que se ve en el panel, ej. 0.3); con Ki = 0 manda 0
static float factor_kaw(const PID_ *pid)
{
  return ((pid->Ki > 0.0f) && (pid->Kp > 0.0f)) ? pid->Kaw * pid->Kp / pid->Ki : 0.0f;
}


//parametros y estado de los PID (mismas etiquetas del panel de LabVIEW / Python)
void manda_pid_parametros(void)
{
  tele_n = 0;

  //PID de linea
  AGREGA("*j%.4f*", PID_DIR.Kp);               //constantes chicas (salida en m/s)
  AGREGA("*k%.4f*", PID_DIR.Ki);
  AGREGA("*l%.4f*", PID_DIR.Kd);
  AGREGA("*vbas%.2f*", PARAM.velo_basica);      //velocidad basica
  AGREGA("*g%.2f*", PID_DIR.out_max);           //velocidad de giro = salida maxima del PID
  AGREGA("*n%.2f*", factor_tau(&PID_DIR));      //tau
  AGREGA("*m%.2f*", factor_kaw(&PID_DIR));      //kaw
  AGREGA("*f%.2f*", PARAM.kg);                  //factor de correccion del giro

  //PID de velocidad: set point y salida
  AGREGA("*SETIZ%.2f*", PID_VEL_IZQ.setpoint);
  AGREGA("*SETDE%.2f*", PID_VEL_DER.setpoint);
  AGREGA("*PIDDER%.2f*", PID_VEL_DER.output);
  AGREGA("*PIDIZQ%.2f*", PID_VEL_IZQ.output);

  //PID de velocidad: parametros (los dos motores usan los mismos)
  AGREGA("*KPVEL%.2f*", PID_VEL_DER.Kp);
  AGREGA("*KIVEL%.2f*", PID_VEL_DER.Ki);
  AGREGA("*KDVEL%.2f*", PID_VEL_DER.Kd);

  //PID de velocidad: partes de la salida
  AGREGA("*UPVELDER%.2f*", PID_VEL_DER.up);
  AGREGA("*UIVELDER%.2f*", PID_VEL_DER.ui);
  AGREGA("*UDVELDER%.2f*", PID_VEL_DER.ud);
  AGREGA("*UPVELIZQ%.2f*", PID_VEL_IZQ.up);
  AGREGA("*UIVELIZQ%.2f*", PID_VEL_IZQ.ui);
  AGREGA("*UDVELIZQ%.2f*", PID_VEL_IZQ.ud);

  AGREGA("*KAWVEL%.2f*", factor_kaw(&PID_VEL_DER));
  AGREGA("*TAUVEL%.2f*", factor_tau(&PID_VEL_DER));

  //rectas y color de linea (cuadros de PRINCIPAL)
  AGREGA("*RECTERR%.2f*", PARAM.error_recta);
  AGREGA("*RECTVEL%.2f*", PARAM.velo_recta);
  AGREGA("*RECTLARG%.2f*", PARAM.largo_recta);
  AGREGA("*RECTGIR%.2f*", PARAM.giro_recta);
  AGREGA("*FRENOV%.2f*", PARAM.velo_freno);     //PWM
  AGREGA("*FRENOMS%.0f*", PARAM.freno_ms);      //ms
  AGREGA("*LINCO=%d*", (int)PARAM.linea_color);

  uartx_write_text(&TELE_UART, tele);   //una sola llamada para toda la trama
}


//robot QUIETO esperando el start (antes manda_espera_start / manda_quieto):
//posicion de la linea, giro y velocidad de cada llanta.
//Antes de llamarla actualizar la posicion: Lee_sensores, filtra_sensores,
//Normaliza_sensores y Calcula_Pesos. Los parametros van aparte (manda_pid_parametros)
void manda_espera_start(void)
{
  tele_n = 0;
  AGREGA("*L%d*", (int)flag_linea);          //1 = ve la linea, 0 = no (e = ultimo lado)
  AGREGA("*e%.2f*", peso);                    //posicion de la linea -> barra POSICION
  AGREGA("*q%.1f*", MPU6050.giro_z_f);        //grados/s -> reloj VEL_ANGULAR
  AGREGA("*w%.3f*", LLANTA_IZQ.velocidad);    //m/s
  AGREGA("*z%.3f*", LLANTA_DER.velocidad);
  uartx_write_text(&TELE_UART, tele);
}

//robot en CARRERA (antes manda_operando / manda_operando1):
//posicion, partes del PID de linea, giro y velocidades.
//sensores = 1 tambien manda los 16 sensores normalizados (Nor=..., mas lento)
void manda_operando(uint8_t sensores)
{
  if (sensores) Manda_Normalizados();

  tele_n = 0;
  AGREGA("*L%d*", (int)flag_linea);          //1 = ve la linea, 0 = no (e = ultimo lado)
  AGREGA("*e%.2f*", peso);                    //posicion de la linea
  AGREGA("*p%.2f*", PID_DIR.up);              //partes del PID de linea -> UP UI UD UT
  AGREGA("*i%.2f*", PID_DIR.ui);
  AGREGA("*d%.2f*", PID_DIR.ud);
  AGREGA("*t%.2f*", CARRERA.giro);            //correccion total: PID + IMU (UT del panel)
  AGREGA("*VB%.0f*", CARRERA.v_base);         //velocidad base que se esta usando (con el IMU)
  AGREGA("*RD%.0f*", CARRERA.recta_cm);       //cm que lleva seguidos en recta (para ajustar TRAMO)
  AGREGA("*q%.1f*", MPU6050.giro_z_f);
  AGREGA("*w%.3f*", LLANTA_IZQ.velocidad);
  AGREGA("*z%.3f*", LLANTA_DER.velocidad);
  uartx_write_text(&TELE_UART, tele);
}


//lectura CRUDA del ADC (0..4095) de los 16 sensores, de I8 a D8:  Cru=v1/v2/.../v16/
//(antes llamar Lee_sensores). El panel la muestra debajo de cada LED
void manda_crudos(void)
{
  tele_n = 0;
  AGREGA("Cru=");
  for (uint8_t i = 0; i < ir_n_sensores; i++) AGREGA("%u/", (unsigned)lecturas_ir_an[i]);
  AGREGA("\r\n");
  uartx_write_text(&TELE_UART, tele);
}
