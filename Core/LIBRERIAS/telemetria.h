/*
 * telemetria.h
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Tramas que el robot manda al panel (PC en Python / celular).
 *
 *  Formato de manda_pid_parametros:  *etiqueta valor*  pegados, sin fin de linea
 *     *j100.00**k0.10**l10.00*...
 *  (el panel separa cada dato por los asteriscos)
 *
 *  Llamar desde el while, NO desde una interrupcion (el envio espera cada byte).
 */

#ifndef LIBRERIAS_TELEMETRIA_H_
#define LIBRERIAS_TELEMETRIA_H_

#include "main.h"

extern UART_HandleTypeDef huart1;
#define TELE_UART  huart1      //puerto del Bluetooth

void manda_pid_parametros(void);   //constantes de los PID, rectas y color de linea
void manda_espera_start(void);     //robot quieto: posicion, giro, velocidades
void manda_operando(uint8_t sensores); //carrera: posicion, PID de linea, giro, velocidades
void manda_crudos(void);           //lectura cruda del ADC de cada sensor: Cru=v1/.../v16/

#endif /* LIBRERIAS_TELEMETRIA_H_ */
