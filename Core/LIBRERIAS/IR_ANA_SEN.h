/*
 * IR_ANA_SEN.h
 *
 *  Created on: Oct 10, 2025
 *      Author: ALCIDES_RAMOS
 */

#ifndef LIBRERIAS_IR_ANA_SEN_H_
#define LIBRERIAS_IR_ANA_SEN_H_

#include "IR_ANA_SEN.h"
#include "main.h"
#include "adc_x.h"
#include "uart.h"
#include "DRV8871.h"

#define blanco_  0
#define negro_   1



#define ir_n_sensores 16   //numero de sensores
#define linea_blanco  0.6f //umbral para decir que un sensor VE la linea (antes 0.4: el fondo
                           //alto de D8 llegaba a 0.4 y daba una linea "fantasma" a la derecha)
#define ir_confirma   3    //despues de perder la linea, verla esta cantidad de veces seguidas
                           //para creerla (3 x 5 ms): un pico aislado de un sensor no engaña

//sensores que se usan: bit 0 = I8 ... bit 15 = D8. Un sensor en 0 se ignora (normalizado = 0)
//ej. sin D8: 0x7FFF    todos: 0xFFFF
//#define IR_SENSORES_ACTIVOS  0x7FFF   //PRUEBA 03/11/2026: sin D8 (fondo alto, linea fantasma)
#define IR_SENSORES_ACTIVOS  0xFFFF   //PRUEBA 03/11/2026: sin D8 (fondo alto, linea fantasma)

//calculo de la posicion (Calcula_Pesos)
#define umbral_peso   0.25f //cada sensor aporta (valor - umbral_peso): el ruido bajo esto no cuenta
#define ventana_peso  2     //sensores a cada lado del maximo que entran al promedio
#define tolera_max    0.10f //sensores a menos de esto del maximo cuentan como "maximo" (empates)

//calibracion: un sensor con (max - min) menor a esto no distingue fondo y linea -> se ignora
//(normalizado siempre 0). Ej. D8 con fondo 2702 y linea 4095 tiene rango 1393: sirve
#define ir_rango_min  300

//#define  linea_color  negro_ //1= negro   0= blanco
extern uint8_t  linea_color; //1= negro   0= blanco



#define  alpha_sen  0.3f  //filtro media movil sensores


#define IR_AN_UART huart1
extern UART_HandleTypeDef IR_AN_UART;


extern uint16_t lecturas_ir_an[];
extern float peso;
extern uint8_t flag_linea;   //1 = algun sensor ve la linea (lo calcula Calcula_Pesos)
extern float Normalizados[ir_n_sensores]; //lecturas 0..1 para detectar cruces anchos (Competition V3)


void Sensores_init();
void Lee_sensores();
void Manda_Lecturas();
void  Manda_Lecturas_filtrada();
void Calibrar_sensores();
void Inicia_filtro();
void  filtra_sensores();
void Manda_Limites();
void Normaliza_sensores();
void Manda_Normalizados();
float Calcula_Pesos();
void Manda_pesos();

#endif /* LIBRERIAS_IR_ANA_SEN_H_ */
