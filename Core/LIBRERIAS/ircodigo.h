/*
 * ircodigo.h
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Lee el codigo de CUALQUIER control IR (NEC, Sony, RC5, Samsung, LG...) y lo
 *  convierte en un numero unico de 32 bits (hash) sin saber el protocolo.
 *
 *  METODO (sin calibrar grupos de tiempos):
 *   - Se miden TODOS los flancos (subida y bajada): pulso alto, pulso bajo, alto...
 *   - Cada duracion se compara con la del MISMO tipo dos posiciones antes
 *     (marca con marca, espacio con espacio) y se reduce a un simbolo:
 *        0 = mas corto   1 = parecido (+-20%)   2 = mas largo
 *   - Los simbolos entran a un hash FNV-1 de 32 bits.
 *   Como solo importa "mas corto / igual / mas largo", no depende del valor
 *   exacto de los tiempos ni de la tolerancia del control: no hay que agrupar.
 *
 *  HARDWARE:
 *   - Pin IR_in (PB9) con EXTI9. Se usan ambos flancos; IR_init activa el de bajada
 *     por codigo (si quieres, en el .ioc pon "Rising/Falling edge").
 *   - TIM7 a 1 us por cuenta (250 MHz / 250). IR_init le pone el timeout.
 *     Si no llegan flancos en IR_TIMEOUT_US el codigo termino.
 *
 *  PRIORIDADES (en el .ioc, NVIC):
 *   El tiempo se toma en la interrupcion del flanco, asi que NADA puede retrasarla.
 *   EXTI9 debe tener MAS prioridad (numero menor) que cualquier interrupcion larga
 *   (ej. TIM6 que lee la IMU por I2C):  EXTI9 = 0,  TIM6 = 1,  TIM7 = 2.
 *
 *  USO:
 *   en stm32h5xx_it.c:
 *     #include "ircodigo.h"
 *     EXTI9_IRQHandler  ->  IR_flanco();
 *     TIM7_IRQHandler   ->  IR_timer();
 *
 *   en el main antes del while:
 *     IR_init();
 *     IR_calibrar(10);   //solo para sacar los codigos: imprime 10 botones y sigue
 *
 *   en el while:
 *     uint32_t codigo;
 *     if (IR_leer(&codigo))
 *       {
 *       if (codigo == IR_START) ...
 *       if (codigo == IR_STOP)  ...
 *       }
 *
 *  NOTAS:
 *   - El "repetir" de NEC (boton sostenido) se descarta solo: tiene pocos flancos.
 *   - RC5/RC6 cambian un bit cada vez que se pulsa: ese boton da 2 codigos,
 *     guarda los dos (por eso calibrar pulsando cada boton 2 o 3 veces).
 */

#ifndef LIBRERIAS_IRCODIGO_H_
#define LIBRERIAS_IRCODIGO_H_

#include "main.h"

//------------------- configuracion --------------------------------
extern TIM_HandleTypeDef htim7;
#define IR_HTIM        htim7        //timer usado (1 us por cuenta)
extern UART_HandleTypeDef huart1;
#define IR_UART        huart1       //puerto para IR_calibrar

#define IR_PIN         IR_in_Pin            //pin del receptor (ver main.h)
#define IR_PUERTO      IR_in_GPIO_Port

#define IR_TIMEOUT_US  15000   //silencio que marca fin del codigo (NEC tiene marca de 9 ms)
#define IR_MAX_TIEMPOS 256     //maximo de duraciones guardadas por codigo
#define IR_MIN_TIEMPOS 12      //menos que esto es ruido o "repetir": se descarta
#define IR_SILENCIO_MS 8       //reposo minimo antes de aceptar el inicio de un codigo
                                //(entre codigos hay >= 18 ms; dentro, maximo 4.5 ms)
//------------------------------------------------------------------

//------------- codigos de los botones (sacados con IR_calibrar) -------------
#define IRSTART   0x87F5C475   //boton encendido
#define IRSTOP    0xCB38CFB7   //boton apagado

//segundo control / otros botones (valores viejos: falta calibrar)
#define IRSTART2  0xF4BA2988
#define IRSTOP2    0x47EA35FF
#define IRPROG    0XA4CAFB55
#define IRPROG2   0XEEC060D9
//------------------------------------------------------------------------------

void     IR_init(void);
void     IR_flanco(void);                 //llamar en EXTI9_IRQHandler
void     IR_timer(void);                  //llamar en TIM7_IRQHandler
uint8_t  IR_leer(uint32_t *codigo);       //1 = llego un codigo valido (y rearma)
void     IR_calibrar(uint16_t cantidad);  //imprime 'cantidad' codigos y sus tiempos (0 = sin fin)

#endif /* LIBRERIAS_IRCODIGO_H_ */
