/*
 * llanta.h
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Convierte las RPM y vueltas del motor (libreria encoder) en velocidad (m/s)
 *  y distancia (m) de cada llanta, y calcula la velocidad y giro del robot.
 *
 *  Ademas tiene el CONTROL DE VELOCIDAD de cada llanta (PID_VEL_IZQ / PID_VEL_DER)
 *  y los procedimientos para CALIBRAR esos PID desde el panel (dobby_panel.py, pestaña VELO).
 *
 *  Necesita la libreria encoder funcionando (Encoder_init + Encoder_velo).
 */

#ifndef LIBRERIAS_LLANTA_H_
#define LIBRERIAS_LLANTA_H_

#include "encoder.h"
#include "PID.h"

//como usar
/*
 1) En "stm32h5xx_it.c", dentro de SysTick_Handler, DESPUES de Encoder_velo():
      Encoder_velo();
      Llanta_velo();          //tambien corre el PID de velocidad cada ENCODER_TS_MS
    e incluir #include "llanta.h"

 2) En main.c, despues de Encoder_init:
      //          llanta       encoder    reductora  diametro(mm)
      Llanta_init(&LLANTA_IZQ, &ENCODER1, 10.0f,     23.0f);
      Llanta_init(&LLANTA_DER, &ENCODER2, 10.0f,     23.0f);
      //        separacion entre llantas (mm, centro a centro)
      Robot_init(160.0f);
      //PID de velocidad (valores de PID.h) + calibracion
      Llanta_control_init();

 3) Usar las variables:
      LLANTA_IZQ.velocidad  -> m/s de la llanta izquierda
      LLANTA_IZQ.distancia  -> metros recorridos por la llanta izquierda
      LLANTA_IZQ.rpm        -> rpm de la llanta (despues de la reductora)
      LLANTA_IZQ.pwm        -> PWM que se esta aplicando al motor (-1000..1000)
      ROBOT.velocidad       -> m/s del centro del robot
      ROBOT.giro            -> rad/s (+ = gira a la izquierda)
      ROBOT.distancia       -> metros recorridos por el centro del robot
      Llanta_reset();       -> pone las distancias en 0

 4) Control de velocidad (en carrera):
      Llanta_modo(LLANTA_PID);            //enciende los PID de velocidad
      Llanta_set_velocidad(1.2f, 1.0f);   //m/s izquierda, derecha (cuando quieras)
      Llanta_modo(LLANTA_OFF);            //apaga motores

 5) CALIBRACION con el panel (ver PROCEDIMIENTO abajo). En el while del main:
      if (UARTRX1.flag_rx==1)
      {
        if (!Llanta_comando((char*)UARTRX1.trama_rx))   //1 = era un comando de velocidad
        {
          //otros comandos ...
        }
        uartRX_DMA_Re_init(&UARTRX1);
      }
      Llanta_tarea();   //telemetria para las graficas + resultados de la identificacion

 IMPORTANTE: ajustar ENCODERx_DIR y ENCODERx_VEL (encoder.c) para que las DOS
 llantas den velocidad y distancia POSITIVAS cuando el robot avanza.
 */

/*
 ======================= PROCEDIMIENTO DE CALIBRACION =======================
 (robot LEVANTADO, ruedas al aire; al final verificar en el piso)

 PASO 1 - Prueba de motores en lazo abierto (pestaña PRINCIPAL: TEST ON + PWM_IZQ / PWM_DER)
    (libreria pruebas.h: el PWM solo sale con TEST en ON)
    Poner PWM_IZQ = 300  -> debe girar SOLO la llanta izquierda hacia ADELANTE
                            y la grafica IZQUIERDA (pestaña VELO) debe dar POSITIVO.
    Igual con PWM_DER. Si la grafica da negativo: invertir ENCODERx_VEL y ENCODERx_DIR
    (encoder.c). Si gira hacia atras: invertir los cables del motor.
    Si se mueve la otra llanta: estan cruzados los encoders o los motores.
    TEST OFF o STOP MOTORES para terminar.

 PASO 2 - Identificacion automatica (pestaña VELO: PWM_id + IDENTIFICAR)
    a) Sube el PWM poco a poco hasta que cada llanta arranca -> ZONA MUERTA
    b) Pausa, y luego da un escalon de PWM_id (ej. 500) y graba la velocidad 1.5 s
    c) Calcula para cada llanta:
         K   = velocidad final / (PWM_id - zona muerta)      [m/s por PWM]
         tau = constante de tiempo (metodo del area)          [s]
       y sugiere un PI (Kp, Ki) que deja el lazo cerrado LLANTA_TC_FACTOR veces tau:
         Tc = LLANTA_TC_FACTOR * tau     Kp = tau/(K*Tc)     Ki = 1/(K*Tc)
    El panel muestra los resultados y el promedio en KP_SUG / KI_SUG.
    APLICAR los manda al robot (VKP / VKI).

 PASO 3 - Ajuste fino con escalones (pestaña VELO)
    SET_POINT_I / SET_POINT_D = velocidad alta (ej. 1.0 m/s), V_baja (ej. 0.3),
    T_esc (ej. 1000 ms) y ESCALON ON: el set point alterna entre alto y bajo.
    Mirar la grafica (amarillo = set point, verde = velocidad):
      - lento, no llega            -> subir Kp
      - oscila / sobrepaso grande  -> bajar Kp (y/o Ki)
      - queda error permanente     -> subir Ki
      - tarda en "despegarse" de la saturacion -> subir KAW (0.5 .. 1.0)
    PID ON = set point fijo (sin escalon), para ver el error en regimen.

 PASO 4 - Copiar los valores finales a los #define PID_VEL_xx de PID.h
          (LEE PID muestra los valores que tiene el robot).

 COMANDOS que entiende Llanta_comando (los manda el panel, terminan en /):
    VKP=  VKI=  VKD=  VTAU=(xTs)  VKAW=(x Ki/Kp)   constantes de los 2 PID de velocidad
    VIZQ= VDER=        set point (m/s)  de cada llanta
    (PWMIZQ= PWMDER= los maneja pruebas.c: solo con TEST ON)
    VPID=1/0           PID con set point fijo
    VESC=1/0           PID con escalones   VBAJA= (m/s)  VESCMS= (ms en cada nivel)
    VIDENT=pwm         identificacion automatica con escalon de pwm
    STOP / VSTOP       apaga motores
 =============================================================================
 */

//modos del control de las llantas
enum {
    LLANTA_OFF = 0,       //sin control (motores apagados al entrar)
    LLANTA_PWM,           //lazo abierto: PWM fijo en cada motor
    LLANTA_PID,           //lazo cerrado: los PID siguen el set point
    LLANTA_ESCALON,       //lazo cerrado: el set point alterna alto / bajo (sintonia)
    LLANTA_IDENT          //identificacion automatica (zona muerta, K, tau)
};

//------------------------------------------------ ajustes de la calibracion
#define LLANTA_PWM_MAX      1000.0f  //PWM maximo del motor (ARR del TIM4)
#define LLANTA_TELE_MS      20       //periodo de la trama rapida (set point y velocidad) ms
#define LLANTA_TELE_LENTA_MS 200     //periodo de la trama lenta (partes del PID, modo) ms

#define LLANTA_ZM_PASO      1.0f     //identificacion: subida del PWM por muestra (1 = 200 PWM/s)
#define LLANTA_ZM_MAX       950.0f   //identificacion: si con este PWM no arranca -> error
#define LLANTA_ZM_PULSOS    8        //cuentas de encoder para decir que la llanta arranco
#define LLANTA_PAUSA_MS     800      //identificacion: pausa (motor apagado) antes de la rampa y del escalon
#define LLANTA_V_QUIETA     0.02f    //identificacion: por debajo de esto (m/s) la llanta esta quieta
#define LLANTA_ID_N         300      //muestras grabadas del escalon (300 x 5 ms = 1.5 s)
#define LLANTA_ID_FINAL     60       //ultimas muestras promediadas = velocidad final
#define LLANTA_PWM_ID       500.0f   //PWM del escalon si el panel no manda otro

//lazo cerrado deseado = LLANTA_TC_FACTOR * tau del motor
//  0.3 = agresivo   0.5 = normal   1.0 = suave (si oscila, subirlo)
#define LLANTA_TC_FACTOR    0.5f
//------------------------------------------------------------------------------

typedef struct {
    ENCODERS *enc;        //encoder del motor de esta llanta
    float reductora;      //relacion de la reductora (10 = 10:1)
    float diametro;       //diametro de la llanta en metros
    float k_vel;          //factor: rpm motor -> m/s
    float k_dist;         //factor: vueltas motor -> m

    //resultados
    float rpm;            //rpm de la llanta
    float velocidad;      //m/s
    float distancia;      //m

    //control de velocidad
    PID_ *pid;            //PID de velocidad de esta llanta (lo pone Llanta_control_init)
    float pwm;            //PWM aplicado al motor (-1000..1000, + = adelante)
    float pwm_ref;        //PWM pedido en modo LLANTA_PWM
    float vref;           //velocidad pedida en m/s (modo LLANTA_PID / alto del escalon)

    //resultados de la identificacion
    float zona_muerta;    //PWM con el que arranca (-1 = no arranco)
    float k;              //ganancia: m/s por unidad de PWM
    float tau;            //constante de tiempo (s)
    float kp_sug;         //Kp sugerido
    float ki_sug;         //Ki sugerido
} LLANTA;

typedef struct {
    LLANTA *izq;
    LLANTA *der;
    float separacion;     //distancia entre llantas en metros

    //resultados
    float velocidad;      //m/s del centro del robot
    float giro;           //rad/s (+ = gira a la izquierda)
    float distancia;      //m recorridos por el centro del robot
} ROBOTS;

//estado de la calibracion (lo maneja la libreria)
typedef struct {
    volatile uint8_t modo;    //LLANTA_OFF, LLANTA_PWM ...
    float    v_baja;          //escalon: velocidad del nivel bajo (m/s)
    uint32_t esc_ms;          //escalon: tiempo en cada nivel (ms)
    float    pwm_ident;       //identificacion: PWM del escalon

    //interno
    uint32_t cuenta;          //muestras en la fase / nivel actual
    uint8_t  fase;            //fase de la identificacion
    uint8_t  alto;            //escalon: 1 = nivel alto
    float    rampa;           //identificacion: PWM de la rampa de zona muerta
    volatile uint8_t listo;   //1 = identificacion terminada, falta calcular y reportar
} LLANTA_CALS;

extern LLANTA LLANTA_IZQ;
extern LLANTA LLANTA_DER;
extern ROBOTS ROBOT;
extern LLANTA_CALS LLANTA_CAL;

void Llanta_init(LLANTA *llanta, ENCODERS *enc, float reductora, float diametro_mm);
void Robot_init(float separacion_mm);
void Llanta_reset(void);
void Llanta_velo(void);

//control de velocidad y calibracion
void    Llanta_control_init(void);
void    Llanta_modo(uint8_t modo);
void    Llanta_set_velocidad(float v_izq, float v_der);
void    Llanta_set_pwm(float pwm_izq, float pwm_der);   //lazo abierto (modo LLANTA_PWM)
void    Llanta_sintoniza(float kp, float ki, float kd, float tau_ts, float kaw_f);
uint8_t Llanta_comando(const char *trama);   //1 = era un comando de velocidad
void    Llanta_tarea(void);                  //en el while (NO en interrupcion)

#endif /* LIBRERIAS_LLANTA_H_ */
