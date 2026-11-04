/*
 * pruebas.h
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Modo TEST del robot, manejado desde el panel (pestaña PRINCIPAL: TEST ON/OFF + selector).
 *
 *  - PRUEBA.activo es la BANDERA: 1 = TEST ON, 0 = TEST OFF.
 *  - Con TEST ON el PWM de los campos PWM_IZQ / PWM_DER sale a los motores.
 *    Con TEST OFF el PWM NO sale (los motores quedan apagados).
 *  - PRUEBA.estado elige QUE se prueba (maquina de estados). Cada estado manda
 *    al panel lo que hace falta para revisar esa parte del robot.
 *  - El robot avisa su bandera y su estado cada 500 ms:  *TST1**TSTE2*
 *    (el panel prende el LED de TEST y pone el selector en esa prueba).
 */

#ifndef LIBRERIAS_PRUEBAS_H_
#define LIBRERIAS_PRUEBAS_H_

#include "main.h"

//como usar
/*
 1) En main.c, en el procesamiento de lo que llega por UARTRX1 (ANTES de Llanta_comando):
      if (Prueba_comando(trama)) return;      //TESTON, TESTOFF, TESTSEL=n, PWMIZQ=, PWMDER=

 2) En el while:
      Prueba_tarea();     //corre la prueba seleccionada (solo si TEST esta ON)

 3) Usar la bandera donde se necesite:
      if (PRUEBA.activo) { ... }

 COMANDOS (los manda el panel, terminan en /):
    TESTON / TESTOFF     prende / apaga el modo test (la bandera)
    TESTSEL=n/           elige la prueba (0 = MOTORES, 1 = ENCODER ...)
    PWMIZQ=v/ PWMDER=v/  PWM de cada motor -1000..1000 (solo sale con TEST ON)
    STOP                 tambien apaga el test

 PARA AGREGAR UNA PRUEBA NUEVA:
    a) agregarla al enum (antes de PRUEBA_N)
    b) agregar su nombre en PRUEBA_NOMBRE[] y su periodo en PRUEBA_PERIODO[] (pruebas.c)
    c) agregar su case en Prueba_tarea (y si hace falta en Prueba_entra / Prueba_sale)
    d) agregar el nombre en PRUEBAS_ROBOT del panel (dobby_panel.py), en el MISMO orden
 */

//estados (pruebas) disponibles. El orden debe ser el mismo del panel
enum {
    PRUEBA_MOTORES = 0,   //solo PWM a los motores (mirar las graficas en VELO)
    PRUEBA_ENCODER,       //manda pulsos, rpm y velocidad de los encoders cada 250 ms
    PRUEBA_SENSORES,      //manda los sensores normalizados (LEDs del panel) cada 100 ms
    PRUEBA_IMU,           //manda el giro en Z (reloj VEL_ANGULAR) cada 50 ms
    PRUEBA_N              //cantidad de pruebas (dejar siempre al final)
};

typedef struct {
    volatile uint8_t activo;  //BANDERA: 1 = TEST ON
    uint8_t  estado;          //prueba seleccionada (PRUEBA_MOTORES ...)
    float    pwm_izq;         //ultimo PWM pedido para cada motor
    float    pwm_der;

    //interno
    uint8_t  estado_ant;      //para detectar el cambio de estado (accion de entrada)
    uint32_t t_ant;           //ultima vez que corrio la prueba (ms)
    uint32_t t_bandera;       //ultima vez que se aviso la bandera al panel (ms)
} PRUEBAS;

extern PRUEBAS PRUEBA;

void    Prueba_activa(uint8_t on);        //1 = TEST ON, 0 = TEST OFF (apaga motores)
void    Prueba_estado(uint8_t estado);    //elige la prueba
uint8_t Prueba_comando(const char *trama);//1 = era un comando de test
void    Prueba_tarea(void);               //en el while (NO en interrupcion)

#endif /* LIBRERIAS_PRUEBAS_H_ */
