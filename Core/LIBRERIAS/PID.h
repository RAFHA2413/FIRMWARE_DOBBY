/*
 * PID.h
 *
 *  Created on: Nov 2, 2024
 *      Author: ALCIDES_RAMOS
 *
 *  PID en forma de estructura: se crea un PID_ por cada lazo de control.
 *
 *  PID_Control       : integral Tustin + derivada filtrada sobre la medicion
 *                      + anti-windup por back-calculation (el recomendado)
 *  PID_Control_Basic : derivada simple del error + integracion condicional
 *
 *  UNIDADES: Ki y Kd van en segundos (forma continua):
 *     salida = Kp*e + Ki*integral(e dt) + Kd*de/dt
 *  Si cambias Ts las constantes siguen valiendo lo mismo.
 *  (Un Kd "por muestra" de antes equivale a Kd = Kd_muestra * Ts)
 *
 *  IMPORTANTE: PID_Control debe llamarse cada Ts exacto (interrupcion de timer
 *  o if con HAL_GetTick), no "cuando pase por el while".
 */

#ifndef SRC_LIBRERIAS_PID_H_
#define SRC_LIBRERIAS_PID_H_

#include "main.h"

//Tau=ts  respuesta rapida (señal sin ruido)
//Tau=5ts  respuesta media suviza la derivada filtra (señal algo ruido)
//Tau=10ts  respuesta lenta pero suaviza mas (Señal mucho ruido)

//anti-windup en relacion a Ki/Kp:
//kaw=0.1ki/kp  control suave sin tanta saturacion;
//kaw=0.3ki/kp  control rapido algunas saturaciones;
//kaw=0.5ki/kp  a 1.0ki/kp control agresivo
//kaw=0  sin anti-windup (la integral solo queda limitada a la salida max)

// PIDs  valores de inicio
//PID de direccion (entrada = posicion de la linea -7.5..7.5, salida = diferencia en PWM 0..1000)
//corre cada 5 ms en el TIM6 (Carrera_control). Valores de PARTIDA: sintonizar con el panel
#define PID_DIR_TS    5e-3f   // Tiempo de muestro
#define PID_DIR_KP    40.0f   // linea a 3 de distancia -> 120 PWM de diferencia
#define PID_DIR_KI    0.0f
#define PID_DIR_KD    1.0f
#define PID_DIR_TAU   (5.0f * PID_DIR_TS)  // filtro derivativo (Ts rapido, 5Ts medio, 10Ts lento)
#define PID_DIR_KAW   (0.3f * PID_DIR_KI / PID_DIR_KP)  // anti-windup
#define PID_DIR_MAX   250.0f   //salida +- en PWM (V_Gir)

//PID de velocidad de cada llanta (entrada m/s, salida PWM del TIM4 0..1000)
//calibrado 03/11/2026 con dobby_panel.py (IDENTIFICAR + escalones), ruedas al aire,
//DRV8871_FRENO = 1:  zona muerta ~90-100, K ~0.0026 m/s/pwm, tau ~50 ms
//(si se cambia el modo del DRV8871 o el encoder x2/x4 hay que volver a calibrar)
#define PID_VEL_TS    5e-3f   // ENCODER_TS_MS = 5
#define PID_VEL_KP    763.72f
#define PID_VEL_KI    15497.0f
#define PID_VEL_KD    0.0f
#define PID_VEL_TAU   (5.0f * PID_VEL_TS)  
#define PID_VEL_KAW   (0.5f * PID_VEL_KI / PID_VEL_KP)  
#define PID_VEL_MAX   1000.0f



typedef struct {
    // Parametros PID
    float Kp;
    float Ki;
    float Kd;
    float Ts;        // tiempo de muestreo (s)
    float Tau;       // constante del filtro derivativo (s)
    float Kaw;       // ganancia anti-windup (1/s)

    // Limites
    float out_min;
    float out_max;

    // Setpoint
    float setpoint;

    // Estado interno (lo maneja la libreria)
    float error;
    float prev_error;
    float up;        // parte proporcional
    float ui;        // parte integral
    float ud;        // parte derivativa
    float output;
    float prev_input;
    float aw_term;   // cuanto se esta saturando la salida (salida_sat - salida)
    uint8_t primera; // 1 = primer calculo despues de un reset

    // Coeficientes de la derivada filtrada (se calculan en PID_Sintoniza)
    float cd1;
    float cd2;
} PID_;


extern PID_  PID_DIR;
extern PID_  PID_VEL_IZQ;
extern PID_  PID_VEL_DER;

void PID_DIR_INIT(void);      //arranca con los #define de arriba
void PID_VEL_INIT(void);
//-------------------------------------------------------------------------


//inicia un PID (deja el estado en cero y setpoint = 0)
// Tau: filtro derivativo en segundos (minimo Ts)
//    Tau = Ts     respuesta rapida (senal sin ruido)
//    Tau = 5*Ts   media, suaviza la derivada (senal algo de ruido)
//    Tau = 10*Ts  lenta, suaviza mas (senal con mucho ruido)
// Kaw: anti-windup en 1/s, ej. Kaw = 0.3*Ki/Kp   (0 = sin back-calculation)
//    0.1 = suave   0.3 = rapido   0.5 a 1.0 = agresivo
void  PID_Init(PID_ *pid, float Kp, float Ki, float Kd, float Ts,
               float out_min, float out_max, float Tau, float Kaw);

//cambia las constantes SIN borrar el estado (para sintonizar en marcha, ej. por Bluetooth)
void  PID_Sintoniza(PID_ *pid, float Kp, float Ki, float Kd,
                    float Tau, float Kaw);

float PID_Control(PID_ *pid, float input);
float PID_Control_Basic(PID_ *pid, float input);
void  PID_Reset(PID_ *pid);

#endif
/*
//USO
 //EN EL MAIN
 PID_DIR_INIT();
 PID_VEL_INIT();

 //si despues quieres usar lo guardado en flash (o lo que llega por Bluetooth)
 PID_Sintoniza(&PID_DIR, PARAM.kp, PARAM.ki, PARAM.kd, PARAM.tau * PID_DIR.Ts,
               PARAM.kaw * PARAM.ki / PARAM.kp);
 PID_DIR.out_max =  PARAM.vel_giro;
 PID_DIR.out_min = -PARAM.vel_giro;

 //cada Ts (ej. en la interrupcion del timer de 5 ms)
 PID_DIR.setpoint = 0;                              //linea centrada
 float giro = PID_Control(&PID_DIR, posicion_linea);

 PID_VEL_IZQ.setpoint = 1.5f;                       //m/s
 float pwm_izq = PID_Control(&PID_VEL_IZQ, LLANTA_IZQ.velocidad);

 //crear otro PID cualquiera
 PID_ PID_X;
 PID_Init(&PID_X, kp, ki, kd, 0.01f, -1000, 1000, 5, 0.3f);
*/
