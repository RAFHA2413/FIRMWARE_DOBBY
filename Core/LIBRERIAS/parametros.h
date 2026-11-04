/*
 * parametros.h
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Guarda los parametros del robot (PID, velocidades, etc.) en la FLASH interna
 *  (zona FLASH_PARAM_DIR de la libreria flash_interna). Al arrancar los lee;
 *  si no hay datos validos carga los valores por defecto.
 */

#ifndef LIBRERIAS_PARAMETROS_H_
#define LIBRERIAS_PARAMETROS_H_

#include "main.h"

//como usar
/*
 1) En main.c, al arrancar (despues de MX_ICACHE_Init):
      if (Param_init()) { //cargo de la flash }  else { //valores por defecto }

 2) Usar los valores en cualquier parte:  PARAM.kp, PARAM.velo_basica ...

 3) Comandos por Bluetooth (en el procesamiento de UARTRX1.trama_rx):

      //cambia los valores en RAM (NO graba). Formato:
      //   kp  ki  kd  Vb  Vg  tau kaw Kg errrect velrect largorect color girorect vfreno tfreno
      //   kp  ki  kd  Vb  Vg  tau kaw Kg errrect velrect largorect color girorect vfreno tfreno version acelr
      // DAT/100/0.1/10/370/150/5/0.3/0.1/1.5/500/30/0/40/115/200/8/300/
      //(girorect, vfreno, tfreno, version y acelr son opcionales: una trama vieja de 12 valores tambien sirve)
      //velrect y vfreno en PWM; tfreno en ms. Sin version: velrect y vfreno de 0 a 2.6 son m/s
      if (strstr((char*)UARTRX1.trama_rx, "DAT/"))
          Param_desde_trama((char*)UARTRX1.trama_rx);

      //graba en flash (SOLO con el robot detenido)
      else if (strstr((char*)UARTRX1.trama_rx, "GUARDAR"))
          Param_guardar();

      //devuelve los valores actuales
      else if (strstr((char*)UARTRX1.trama_rx, "LEER"))
      {
          Param_a_trama(texto);
          uartx_write_text(&huart1, texto);
      }

      //vuelve a los valores por defecto (en RAM; GUARDAR para grabarlos)
      else if (strstr((char*)UARTRX1.trama_rx, "DEFECTO"))
          Param_defecto();
 */

//cambiar si se agregan/quitan campos a la struct: invalida lo guardado y carga los de defecto
//2: velocidades en m/s   3: V_Bas, V_Gir y PID de linea otra vez en PWM (0..1000)
//4: + giro_recta
//5: V_LINEA en m/s, + freno (velo_freno, freno_ms)
//   Param_init pasa solo los datos de las versiones 3 y 4 (no se pierden); como V_LINEA
//   estaba en PWM, queda en PARAM_V_LINEA_DEF
//6: V_LINEA otra vez en PWM (0..1000, igual que V_Bas)
//7: V_FRENO tambien en PWM. Misma struct que la 5 y la 6: Param_init y Param_desde_trama
//   pasan los valores viejos de m/s a PWM (V_LINEA 1.30 -> 500, V_FRENO 0.30 -> 115)
//8: + acel_recta (ACEL_R). Param_leer_flash pasa los datos de las versiones anteriores
#define PARAM_VERSION   8

//valores por defecto de la recta y el freno, en PWM 0..1000 (el panel los cambia,
//GUARDAR los graba)
#define PARAM_V_LINEA_DEF   500.0f  //PWM
#define PARAM_V_FRENO_DEF   115.0f  //PWM
#define PARAM_T_FRENO_DEF   200.0f  //ms maximo frenando
#define PARAM_ACEL_R_DEF    300.0f  //PWM/s: rampa de V_Bas a V_LINEA en la recta
#define PARAM_ACEL_R_MAX    5000.0f //PWM/s: mas alto o <= 0 vuelve al valor por defecto

typedef struct {
    uint32_t marca;        //PARAM_MARCA si hay datos validos
    uint32_t version;      //PARAM_VERSION

    //PID de direccion (linea)
    float kp;
    float ki;
    float kd;
    float tau;             //filtro derivativo (en multiplos de Ts)
    float kaw;             //anti-windup: factor, Kaw = kaw*Ki/Kp (0 = sin back-calculation)
    float vel_giro;        //salida maxima del PID (Vg)

    //velocidades
    float velo_basica;     //Vb
    float kg;              //Kg

    //rectas (carrera.c, estado CARRERA_RECTA). 0 en error/velo/largo = rectas apagadas
    float error_recta;     //Error_LIM: |posicion de la linea| maxima para considerar recta (-7.5..7.5)
    float velo_recta;      //V_LINEA: velocidad base en recta (PWM 0..1000)
    float largo_recta;     //TRAMO: cm seguidos en recta (encoder) para pasar a V_LINEA

    uint32_t linea_color;  //0 = linea blanca, 1 = linea negra (IR_ANA_SEN.h: blanco_ 0, negro_ 1)

    //(version 4) va DESPUES de linea_color: asi la version 3 se puede leer igual
    float giro_recta;      //GIRO_LIM: |giro_z| maximo (grados/s) para considerar recta. 0 = no usa IMU

    //(version 5) freno al salir de la recta (carrera.c, estado CARRERA_FRENO)
    float velo_freno;      //V_FRENO: velocidad base mientras frena (PWM 0..1000; 0 = freno maximo)
    float freno_ms;        //T_FRENO: tiempo MAXIMO frenando (ms). 0 = sin freno (RECTA -> PID)

    //(version 8) rampa al acelerar en la recta (carrera.c, estado CARRERA_RECTA)
    float acel_recta;      //ACEL_R: PWM por segundo

    uint32_t crc;         //SIEMPRE el ultimo campo
} PARAMETROS;

extern PARAMETROS PARAM;

uint8_t Param_init(void);                      //1 = cargo de flash, 0 = cargo por defecto
uint8_t Param_leer_flash(void);                //1 = cargo de flash, 0 = sin datos validos (PARAM no cambia)
void    Param_defecto(void);                   //valores por defecto en RAM
void    Param_aplica(void);                    //PARAM -> PID_DIR y color de linea
uint8_t Param_guardar(void);                   //1 = grabo bien, 0 = error
uint8_t Param_desde_trama(const char *trama);  //1 = trama valida, 0 = error
void    Param_a_trama(char *texto);            //escribe "DAT/..../\r\n" en texto

#endif /* LIBRERIAS_PARAMETROS_H_ */
