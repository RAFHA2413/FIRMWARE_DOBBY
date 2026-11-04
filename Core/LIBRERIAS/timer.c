
/*
 * timer.h
 *
 *  Created on: May 1, 2025
 *      Author: ALCIDES_RAMOS
 */
#include "timer.h"
#include "pinout1.h"
#include <stdint.h>
#include "mpu6050.h"
#include "carrera.h"
uint32_t pasos1;

uint32_t  ojos=0;

uint32_t tmicros;
float tmilis;

//pendiente padar a auxiliares
#define ALFA_GIRO   0.3f  //para el filto del giro
//float giro_z_filtrado = 0.0f;
//>>>>> CAMBIO: volatile, lo escribe la interrupcion del TIM6 y lo lee el main
volatile float giro_z_filtrado = 0.0f;


//LO QUE  SE REALIZA EN EL TIEMPO DE MUSTTREO
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//mira si el timer desead0
	 if (htim->Instance == TIM_INT)
	 {
  	 	   //IMU
		 TIMER_PROCEDIMIENTOS();
	 }
			 /* Prevent unused argument(s) compilation warning */
		  UNUSED(htim);

		  /* NOTE : This function should not be modified, when the callback is needed,
		            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
		   */
}

void TIMER_PROCEDIMIENTOS()
{
	//COLOQUE LOS PROCEDIMIENTOS  USAR ACORDE  SENSORES
	   
	 ojos++;
          MPU6050_lee_datos();
	    MPU6050_Giro_flotantes();
		  Actualiza_Giro_Filtrado();

	   //maquina de estados de la carrera: sensores -> PID de linea -> velocidad de llantas
	   Carrera_control();
		



	 ///titileo ojos
	 if (ojos>25) 
	 {
     PIN_BLINK(LEDR1);    
	  PIN_BLINK(LEDR2);    
	ojos=0;
	
	 }
	 
	 
}

void TIMER_INIT()
{
	HAL_TIM_Base_Start_IT(&htim_INT);
}


//MIDO   TIEMPO DE ESOS PROCEDIEMTOS
void TIMER_MIDE()
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	    pasos1=(HAL_RCC_GetSysClockFreq()/1000000);//le el cristal pasa a us

	    // INICIA CONTADOR
	    	    DWT->CYCCNT=0;
	    	    TIMER_PROCEDIMIENTOS();
	    	    tmicros= (DWT->CYCCNT / pasos1);//retrona los us
                 tmilis=tmicros/1000.0;
}



void Actualiza_Giro_Filtrado()
{
    // Lee nueva muestra del giroscopio
    float nueva_muestra = MPU6050.giro_z_f;

    // filtro media movil
    giro_z_filtrado = (1.0f - ALFA_GIRO) * giro_z_filtrado + ALFA_GIRO * nueva_muestra;
}



