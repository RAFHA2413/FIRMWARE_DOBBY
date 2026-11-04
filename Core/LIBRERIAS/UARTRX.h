/*
 * UART1RX1.h
 *
 *  Created on: Apr 9, 2025
 *      Author: ALCIDES_RAMOS
 */

#ifndef LIBRERIAS_UARTRX_H_
#define LIBRERIAS_UARTRX_H_

#include "main.h"


typedef struct {
    //los 4 primeros se cargan al definir:  {&huart1, USART1, buffer, sizeof(buffer)}
    UART_HandleTypeDef *huart;
    USART_TypeDef *usart_instance;
    uint8_t *trama_rx;           //buffer de recepcion 
	uint16_t sizeT;


    volatile uint32_t *DR;
    volatile uint32_t *SR;
    volatile uint32_t *RDR;
    volatile uint32_t *ICR;
    volatile char flag_rx;       //volatile: la escribe la interrupcion y la lee el main
	uint16_t dato;
	volatile uint16_t num_datos; //volatile: la escribe la interrupcion y la lee el main
} UARTRXS;


extern UARTRXS   UARTRX1;
extern UARTRXS   UARTRX2;
extern UARTRXS   UARTRX3;
extern UARTRXS   UARTRX4;
extern UARTRXS   UARTRX5;
extern UARTRXS   UARTRX6;
extern UARTRXS   UARTRX7;
extern UARTRXS   UARTRX8;
extern UARTRXS   UARTNEXTION;
extern UARTRXS   GPS_UARTRX;
extern UARTRXS   MQTT_UARTRX;

//definir en el  UART1RX1.c los uart a usar
/*
 //DEFINE LOS USART  A USAR
 extern UART_HandleTypeDef huart1;

//buffer de recepcion estatico e inicio del uart (tamaño = sizeof del buffer)
static uint8_t buffer_rx1[512];
UARTRXS UARTRX1 = {&huart1, USART1, buffer_rx1, sizeof(buffer_rx1)};

//si usa otro usart, otro buffer:
static uint8_t buffer_rx2[256];
UARTRXS UARTRX2 = {&huart2, USART2, buffer_rx2, sizeof(buffer_rx2)};

//en el procedimiento de interrupcion uartRX_INTERRUPT

 //colocar una por cada usart usado

	//colocar una por cada usart usado
	if ((UARTRX1.flag_rx==0)&& (huart->Instance == UARTRX1.usart_instance))//si es el uart de datos
		{
	  	HAL_UART_DMAStop(UARTRX1.huart);  //para la recepcion temporarmente
         UARTRX1.num_datos=sizex;
	     UARTRX1.flag_rx=1;
		 }

	//si usa otro usart
	if ((UARTRX2.flag_rx==0)&& (huart->Instance == UARTRX2.usart_instance))//si es el uart de datos
		{
		HAL_UART_DMAStop(UARTRX2.huart);  //para la recepcion temporarmente
       UARTRX2.num_datos=sizex;
	     UARTRX2.flag_rx=1;
		 }

//  En el procedimeinto uartRX_Errores  igual colocar una por cada usart usado


		 */
/*
// en el UART_INT_DMA_IDLE.c  (ya esta normalmente)
//llamar interrupciones de usart
	 uartRX_INTERRUPT(huart,Size);


 */

//en el void main antes del while(1) iniciar los uart a usar
/*
 uartRX_it_idle_dma_init( &UARTRX1);
 */

//en el while poner un if por cada usart usado
/*
if (UARTRX1.flag_rx==1)
	  {
	     // uartx_write_text(&huart1, UARTRX1.trama_rx);
	      procesa_rx(); //procedimeinto donde se procesan los datos
		  uartRX_DMA_Re_init(&UARTRX1);
	  }

	  //si usa otro usart
	  if (UARTRX2.flag_rx==1)
	 	    {
	     // uartx_write_text(&huart2, UARTRX2.trama_rx);
	      procesa_rx(); //procedimeinto donde se procesan los datos
		  uartRX_DMA_Re_init(&UARTRX2);
	  }



*******************
//no olvidar procedimiento de procesarx
 *
 * void procesa_rx()
{
//variable para procesar inf
	char procesa[100];
	if (strstr(UARTRX1.trama_rx,"V00=1")) PIN_OFF(LED);
	else if (strstr(UARTRX1.trama_rx,"V00=0")) PIN_ON(LED);

	else if (strstr(UARTRX1.trama_rx,"V01="))
	    {
	        strcpy(procesa, strtok(UARTRX1.trama_rx, "="));  //inicia captura de tokens desde el =
	        strcpy(procesa, strtok(0, "$"));  //captura hasta el /
	        //pasa la cadena a  numero
	       int16_t ancho=atoi(procesa);
	       PWM_valor(&PWM1, ancho);

	    }

}


*/

void uartRX_it_idle_dma_init(UARTRXS *SERIAL);
void uartRX_DMA_Re_init(UARTRXS *SERIAL);
void uartRX_INTERRUPT(UART_HandleTypeDef *huart,uint16_t sizex);
void uartRX_Errores(UART_HandleTypeDef *huart);
 
#endif /* LIBRERIAS_UARTRX_H_ */
