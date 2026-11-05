/*
 * carrera.h
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Maquina de estados de la CARRERA del seguidor.
 *
 *     REPOSO --(RUN / START IR)--> ARRANQUE --(llega a V_Bas)--> PID ----> RECTA
 *        ^                                                       ^ ^          |
 *        |                                                       | +-- FRENO <+ (sale de la recta)
 *        |                                                       +------------+ (ya iba lento)
 *        +------------(STOP serial / STOP IR / pierde la linea)--- cualquiera   (frena)
 *
 *  RECTA (panel: Error_LIM, GIRO_LIM, TRAMO, V_LINEA; se guardan en flash con PARAM):
 *    una muestra "es recta" si  |posicion| <= Error_LIM  y  |giro_z| <= GIRO_LIM
 *    (GIRO_LIM = 0: no mira el IMU). Si lleva TRAMO cm seguidos asi (distancia del encoder)
 *    PID -> RECTA: la velocidad base sube con la rampa ACEL_R (PWM/s) hasta V_LINEA (PWM).
 *    Error_LIM, TRAMO o V_LINEA en 0 = rectas apagadas.
 *
 *  FRENO (panel: V_FRENO PWM, T_FRENO ms): la primera muestra que NO es recta (viene curva)
 *    -> si el robot va mas rapido que V_Bas: velocidad base = V_FRENO (el PID de velocidad
 *       de las llantas frena fuerte, hasta con el motor al reves) sin dejar de seguir la linea.
 *       Termina cuando ROBOT.velocidad ya bajo a V_Bas, o a los T_FRENO ms -> PID.
 *    -> si ya va a V_Bas o menos (o T_FRENO = 0): directo a PID.
 *
 *  CURVA y PERDIDO ya estan en el enum para agregarlos despues
 *  (ver "PARA AGREGAR UN ESTADO").
 *
 *  Lazos (los dos cada 5 ms):
 *    TIM6 (Carrera_control):  sensores -> posicion -> PID_DIR -> giro (PWM)
 *                             izq = v_base - giro     der = v_base + giro      (PWM 0..1000)
 *                             v_rueda (m/s) = PWM * CARRERA_MPS_POR_PWM
 *    SysTick (llanta):        PID de velocidad de cada llanta sigue v_izq / v_der (m/s)
 *
 *  UNIDADES: V_Bas (PARAM.velo_basica), V_Gir (PARAM.vel_giro) y la salida del PID de linea
 *  van en PWM EQUIVALENTE 0..1000 (como antes). Solo el PID de velocidad trabaja en m/s.
 *
 *  PRIMERA PRUEBA: V_Bas = 0  -> el robot queda en su sitio. Empujarlo hacia los lados
 *  sobre la linea y ver como gira para volver a centrarla. Luego subir V_Bas de a poco.
 */

#ifndef LIBRERIAS_CARRERA_H_
#define LIBRERIAS_CARRERA_H_

#include "main.h"

//como usar
/*
 1) En main.c despues de Llanta_control_init y Param_aplica:
      Carrera_init();

 2) En timer.c, dentro de TIMER_PROCEDIMIENTOS (interrupcion del TIM6, cada 5 ms):
      Carrera_control();

 3) Arrancar / parar (serial RUN/STOP, control IR):
      Carrera_run();     //REPOSO -> PID
      Carrera_stop();    //cualquier estado -> REPOSO, frenando

 4) En el while:
      Carrera_tarea();   //telemetria de carrera para el panel (*EST n*, posicion, PID...)

 PARA AGREGAR UN ESTADO (ej. RECTA):
    a) su case en Carrera_control (que hace cada 5 ms y cuando cambia a otro estado)
    b) si necesita algo al entrar, en Carrera_entra
    c) su nombre en ESTADOS_CARRERA del panel (dobby_panel.py) en el mismo orden
 */

//estados de la carrera (el orden debe ser el mismo del panel)
enum {
    CARRERA_REPOSO = 0,   //detenido (al entrar frena CARRERA_FRENO_MS y luego suelta)
    CARRERA_ARRANQUE,     //rampa de aceleracion: V_INICIAL -> V_Bas con CARRERA_ACEL
    CARRERA_PID,          //sigue la linea con PID_DIR y velocidad base
    CARRERA_RECTA,        //recta detectada: velocidad base V_LINEA
    CARRERA_CURVA,        //(pendiente) curva: baja la velocidad
    CARRERA_FRENO,        //sale de la recta: frena con V_FRENO hasta bajar a V_Bas
    CARRERA_PERDIDO,      //(pendiente) no ve la linea
    CARRERA_N
};

//------------------------------------------------ ajustes
#define CARRERA_SIGNO     -1.0f   //si el robot gira HACIA AFUERA de la linea, cambiar el signo
                                  //(-1: probado 03/11/2026, con +1 corregia al reves)
//conversion PWM equivalente -> m/s de cada rueda = K que mide IDENTIFICAR (pestaña VELO)
//  0.0026 -> PWM 1000 = 2.6 m/s,  V_Bas 400 = 1.04 m/s
//  (si se corrige el encoder x2/x4 o se cambian motores/ruedas: poner la K nueva)
#define CARRERA_MPS_POR_PWM 0.0026f
#define CARRERA_PWM_MAX    1000.0f //tope de cada rueda y de la velocidad base (PWM)
#define CARRERA_FRENO_MS   500    //al parar: tiempo frenando antes de soltar los motores
#define CARRERA_PERDIDO_MS 3000    //SEGURIDAD: sin ver la linea este tiempo -> se detiene (REPOSO)
#define CARRERA_TELE_MS    50     //periodo de la telemetria de carrera (ms)

//rampa de ARRANQUE (al recibir RUN / START)
#define CARRERA_V_INICIAL  120.0f //PWM: salto inmediato al arrancar (rompe la inercia y la zona muerta)
                                  //  medido 03/11/2026: en el piso arranca con PWM 100 -> 120 con margen
#define CARRERA_ACEL       1200.0f//PWM por segundo: rampa (1200 -> de 120 a 400 en 0.23 s)
                                  //  si patina o OUT llega a 1000 en la rampa: bajarla
#define CARRERA_GIRO_ARRANQUE 1   //1 = el PID de linea corrige tambien en la rampa (recomendado)
                                  //0 = rampa recta sin corregir (como el arranque viejo)
#define CARRERA_TS_MS      5      //periodo de Carrera_control (TIM6)

//IMU en el PID de DIRECCION (amortiguador del giro):
//    giro = PID(posicion) - KG * giro_z        (giro_z en grados/s del MPU6050)
//  KG = cuadro KG del panel (PARAM.kg), en PWM por grado/s. 0 = no usa el IMU.
//  Si el robot ya esta girando rapido, este termino lo frena antes de que se pase:
//  menos oscilacion y no se sale por girar de mas.  Rango util 0 .. 1 (empezar 0.1)
//  Ej. KG 0.2 girando a 300 grados/s -> 60 PWM menos de correccion
#define CARRERA_SIGNO_GIRO  1.0f   //si con KG > 0 OSCILA MAS (en vez de menos) poner -1.0f
                                   //(depende de como esta montado el MPU6050)

//(opcional, apagado) bajar la velocidad base en curva: v = V_Bas - KV_CURVA * |giro_z|
// DOBBY Competition V2: reduce velocidad SOLO si se aproxima a la curva
// y el error lateral tiene tendencia a crecer. Evita penalizar la recuperacion.
// 0 = algoritmo del profesor; 1 = limitador experimental habilitado.
// No altera PID, modo FRENO, sensores, control de llantas ni parametros en flash.
#define CARRERA_COMP_ADAPTATIVO 1
// Umbrales de la desviacion prevista (unidades del arreglo de 16 sensores).
#define CARRERA_COMP_ERR_INI 0.90f
#define CARRERA_COMP_ERR_FIN 1.45f
// Anticipacion basada en tendencia filtrada; NO observa curvas fuera del sensor.
#define CARRERA_COMP_PRED_S 0.025f
#define CARRERA_COMP_TREND_ALPHA 0.20f
#define CARRERA_COMP_TREND_MAX 30.0f

// DOBBY Competition V3: refuerzo solo en PID con linea centrada y giro pequeno.
// 0 = desactiva V3 completamente y conserva V2 (recomendado como comparacion).
// No modifica los PID, la recuperacion de linea ni las protecciones originales.
#define CARRERA_COMP_PID_BOOST 0  // V4 apaga temporalmente bonificacion V3
#define CARRERA_COMP_PID_BONUS_PWM 45.0f     // mejora experimental; probar primero con seguridad
#define CARRERA_COMP_PID_ERROR_MAX 0.90f     // centros de sensores en unidades de posicion
#define CARRERA_COMP_PID_GIRO_MAX 15.0f     // grados por segundo filtrados
#define CARRERA_COMP_PID_N_SENS_MAX 4       // evita acelerar sobre cruces anchos
#define CARRERA_COMP_PID_CONF_MS 20        // confirmacion de trayectoria centrada
#define CARRERA_COMP_PID_ACEL 600.0f       // PWM equivalentes por segundo
#define CARRERA_COMP_PID_DESACEL 2500.0f   // retira el bonus al aparecer una curva

// V4: mitigar saltos falsos a +/-7.5 durante perdidas de sensor breves.
#define CARRERA_COMP_V4_PUENTE 1
#define CARRERA_COMP_V4_MAX_MS 25U
#define CARRERA_COMP_V4_MAX_POS 2.0f

#define CARRERA_KV_CURVA    0.0f   //PWM por grado/s. 0 = apagado
#define CARRERA_V_MIN_CURVA 120.0f //PWM: la velocidad base nunca baja de esto por el IMU
                                   //(si V_Bas es menor, manda V_Bas)

//FRENO: solo frena si al salir de la recta va mas rapido que V_Bas + este margen (m/s)
#define CARRERA_FRENO_MARGEN 0.05f
//------------------------------------------------

typedef struct {
    volatile uint8_t estado;  //CARRERA_REPOSO, CARRERA_PID ...
    uint32_t cuenta;          //muestras (5 ms) en el estado actual
    float    posicion;        //posicion de la linea (-7.5 .. 7.5, 0 = centrada)
    float    giro;            //salida del PID de linea (PWM de diferencia)
    float    v_base;          //velocidad base que se esta usando (PWM)
    uint8_t  frenando;        //1 = REPOSO con los motores frenados
    uint32_t sin_linea;       //muestras seguidas sin ver la linea
    float    recta_ini;       //ROBOT.distancia (m) donde empezo el tramo recto actual
    float    recta_cm;        //cm que lleva seguidos en recta (telemetria *RD*)
    volatile uint8_t aviso_perdido; //1 = se detuvo por perder la linea (lo avisa Carrera_tarea)
} CARRERAS;

extern CARRERAS CARRERA;

void Carrera_init(void);
void Carrera_run(void);
void Carrera_stop(void);
void Carrera_control(void);   //cada 5 ms en la interrupcion del TIM6
void Carrera_tarea(void);     //en el while

#endif /* LIBRERIAS_CARRERA_H_ */
