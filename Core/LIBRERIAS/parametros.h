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
      //   kp  ki  kd  Vb  Vg  tau kaw Kg errrect velrect largorect color
      // DAT/100/0.1/10/370/150/5/0.3/0.1/0/0/0/0/
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
#define PARAM_VERSION   3

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

    //rectas
    float error_recta;     //error maximo para considerar recta
    float velo_recta;      //velocidad en recta
    float largo_recta;     //cuenta minima para considerar recta

    uint32_t linea_color;  //0 = linea blanca, 1 = linea negra (IR_ANA_SEN.h: blanco_ 0, negro_ 1)

    uint32_t crc;          //SIEMPRE el ultimo campo
} PARAMETROS;

extern PARAMETROS PARAM;

uint8_t Param_init(void);                      //1 = cargo de flash, 0 = cargo por defecto
void    Param_defecto(void);                   //valores por defecto en RAM
void    Param_aplica(void);                    //PARAM -> PID_DIR y color de linea
uint8_t Param_guardar(void);                   //1 = grabo bien, 0 = error
uint8_t Param_desde_trama(const char *trama);  //1 = trama valida, 0 = error
void    Param_a_trama(char *texto);            //escribe "DAT/..../\r\n" en texto

#endif /* LIBRERIAS_PARAMETROS_H_ */
