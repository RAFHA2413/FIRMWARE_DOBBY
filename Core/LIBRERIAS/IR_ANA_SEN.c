/*
 * IR_ANA_SEN.c
 *
 *  Created on: Oct 10, 2025
 *      Author: ALCIDES_RAMOS
 */


#include "IR_ANA_SEN.h"
#include "stdio.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
uint8_t  linea_color= negro_;

uint16_t lecturas_ir_an[ir_n_sensores];

float lecturas_filtradas[ir_n_sensores]= {0};

float Normalizados[ir_n_sensores];
bool linea=false;
uint16_t ir_minimos[ir_n_sensores]={65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535,65535}; //Lectura minima   inicia con un valor alto
uint16_t ir_maximos[ir_n_sensores]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Lectura de maximo  inicia con un valor bajo

//lineal
float ir_pesos[ir_n_sensores]={-7.5,-6.5,-5.5,-4.5,-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5,4.5,5.5,6.5,7.5}; // Lectura normalizadas de 0 a 1
//float pesos[]={-16.0,-8.0,-4.0,-2.0,-0.5,0.5,2.0,4.0,8.0,16.0}; // Lectura normalizadas de 0 a 1
//cuadrarico
//float ir_pesos[]={-20.25, -12.25, -6.25, -2.25, -0.25, 0.25, 2.25, 6.25, 12.25, 20.25};

float ir_peso_anterior;// almacena el sentido de giro anterior
uint8_t flag_linea=0;
float peso,peso_anterior;

void Sensores_init()
{
	 //calibra adc
	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	    HAL_Delay(100);
     //inicia lectura dma
	   //inicia lectura dma
    ADC_Read_DMA(&hadc1, adc1_canales, adc1_codigo);
   HAL_Delay(10);
    

}

void Lee_sensores()
{
	lecturas_ir_an[0]=adc1_codigo[0];
	lecturas_ir_an[1]=adc1_codigo[1];
	lecturas_ir_an[2]=adc1_codigo[2];
	lecturas_ir_an[3]=adc1_codigo[3];
	lecturas_ir_an[4]=adc1_codigo[4];
	lecturas_ir_an[5]=adc1_codigo[5];
	lecturas_ir_an[6]=adc1_codigo[6];
	lecturas_ir_an[7]=adc1_codigo[7];
	lecturas_ir_an[8]=adc1_codigo[8];
	lecturas_ir_an[9]=adc1_codigo[9];
	lecturas_ir_an[10]=adc1_codigo[10];
	lecturas_ir_an[11]=adc1_codigo[11];
	lecturas_ir_an[12]=adc1_codigo[12];
	lecturas_ir_an[13]=adc1_codigo[13];
	lecturas_ir_an[14]=adc1_codigo[14];
	lecturas_ir_an[15]=adc1_codigo[15];
}

void Inicia_filtro1()
{
    for (int i = 0; i < ir_n_sensores; i++)
    {
        lecturas_filtradas[i] = (float)lecturas_ir_an[i];
    }
	lecturas_ir_an[0]=adc1_codigo[15];

}

void Inicia_filtro()
{
    for (int i = 0; i < ir_n_sensores; i++)
    {
        lecturas_filtradas[i] = (float)lecturas_ir_an[i];
    }
}

void  filtra_sensores()
{
	Lee_sensores();
	 for (int i = 0; i <ir_n_sensores; i++)
	    {
	        lecturas_filtradas[i] = alpha_sen * lecturas_ir_an[i] +
	                                (1.0f -alpha_sen ) * lecturas_filtradas[i];
	    }



}

void  Manda_Lecturas()
{
	char sen[200];
sprintf(sen,"%u   %u   %u   %u  %u  %u  %u  %u  %u  %u  %u  %u  %u  %u  %u  %u\r\n\r\n",
lecturas_ir_an[0],lecturas_ir_an[1],lecturas_ir_an[2],lecturas_ir_an[3],lecturas_ir_an[4],lecturas_ir_an[5],lecturas_ir_an[6],lecturas_ir_an[7],lecturas_ir_an[8],lecturas_ir_an[9],
lecturas_ir_an[10],lecturas_ir_an[11],lecturas_ir_an[12],lecturas_ir_an[13],lecturas_ir_an[14],lecturas_ir_an[15]);
uartx_write_text(&IR_AN_UART, sen);

}

void  Manda_Lecturas_filtrada()
{
	char sen[200];
sprintf(sen,"%.1f   %.1f   %.1f   %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f  %.1f\r\n\r\n",lecturas_filtradas[0],lecturas_filtradas[1],lecturas_filtradas[2],lecturas_filtradas[3],lecturas_filtradas[4],lecturas_filtradas[5],lecturas_filtradas[6],lecturas_filtradas[7],lecturas_filtradas[8],lecturas_filtradas[9],
lecturas_filtradas[10],lecturas_filtradas[11],lecturas_filtradas[12],lecturas_filtradas[13],lecturas_filtradas[14],lecturas_filtradas[15]);
uartx_write_text(&IR_AN_UART, sen);

}

void Calibrar_sensores()
{
	int16_t cuenta2;
	uint8_t cont;
	uint8_t sentido=0;
	uint32_t milis_QTR=uwTick;
	#define velo 4080

	//>>>>> CAMBIO 03/11/2026: cada calibracion empieza de CERO (antes solo ampliaba los limites:
	//      si una vez quedaban mal, recalibrar no lo arreglaba)
	for (cont=0; cont<ir_n_sensores; cont++)
	{
		ir_minimos[cont]=65535;
		ir_maximos[cont]=0;
	}

	//retardo inicial de 400 ms
	while(uwTick-milis_QTR<400);
	milis_QTR=uwTick;//inia un nuevo temporizador

	//si se queire que sea automatico

    	 milis_QTR=uwTick;
    	 //tiempo de calibracion
while (uwTick-milis_QTR<6000)
{
     //le los sensores
    for (cont=0; cont<ir_n_sensores;cont++)
        {
    	// Lee_sensores();
		filtra_sensores();
    	  HAL_Delay(1);//delay entre lectira ya que es automatico por dma
           if (lecturas_ir_an[cont]>=ir_maximos[cont]) ir_maximos[cont]=lecturas_ir_an[cont];
           if (lecturas_ir_an[cont]<=ir_minimos[cont]) ir_minimos[cont]=lecturas_ir_an[cont];
         }
}

//por si se hace automatico
//APAGA MOTORES
    MD_Stop();
    MI_Stop();



}
void Manda_Limites()
{
char textoir[300];
  sprintf(textoir,"min1=%u\r\nmin2=%u\r\nmin3=%u\r\nmin4=%u\r\nmin5=%u\r\nmin6=%u\r\nmin7=%u\r\nmin8=%u\r\nmin9=%u\r\nmin10=%u\r\nmin11=%u\r\nmin12=%u\r\nmin13=%u\r\nmin14=%u\r\nmin15=%u\r\nmin16=%u\r\n\r\n",ir_minimos[0],ir_minimos[1],ir_minimos[2],ir_minimos[3],ir_minimos[4],ir_minimos[5],ir_minimos[6],ir_minimos[7],ir_minimos[8],ir_minimos[9],ir_minimos[10],ir_minimos[11],ir_minimos[12],ir_minimos[13],ir_minimos[14],ir_minimos[15]);
  uartx_write_text(&IR_AN_UART, textoir);
  sprintf(textoir,"max1=%u\r\nmax2=%u\r\nmax3=%u\r\nmax4=%u\r\nmax5=%u\r\nmax6=%u\r\nmax7=%u\r\nmax8=%u\r\nmax9=%u\r\nmax10=%u\r\nmax11=%u\r\nmax12=%u\r\nmax13=%u\r\nmax14=%u\r\nmax15=%u\r\nmax16=%u\r\n\r\n",ir_maximos[0],ir_maximos[1],ir_maximos[2],ir_maximos[3],ir_maximos[4],ir_maximos[5],ir_maximos[6],ir_maximos[7],ir_maximos[8],ir_maximos[9],ir_maximos[10],ir_maximos[11],ir_maximos[12],ir_maximos[13],ir_maximos[14],ir_maximos[15]);
  uartx_write_text(&IR_AN_UART, textoir);

  //rango de cada sensor y aviso de los que se van a ignorar (rango < ir_rango_min)
  uint8_t malos = 0;
  for (uint8_t k = 0; k < ir_n_sensores; k++)
  {
    int32_t rango = (int32_t)ir_maximos[k] - (int32_t)ir_minimos[k];
    if (rango < ir_rango_min)
    {
      sprintf(textoir, "SENSOR %u SIN RANGO (%ld): se ignora. Recalibrar pasandolo por la linea\r\n",
              (unsigned)(k + 1), (long)rango);
      uartx_write_text(&IR_AN_UART, textoir);
      malos++;
    }
  }
  if (!malos) uartx_write_text(&IR_AN_UART, "SENSORES: todos con rango OK\r\n");
}

//normalizo datos  en rango de 0 a 1;
void Normaliza_sensores(void)
{
    for (uint8_t k = 0; k < ir_n_sensores; k++)
    {
        float rango = (float)ir_maximos[k] - (float)ir_minimos[k];
        float y = 0.0f;

        //sensor desactivado en IR_SENSORES_ACTIVOS (IR_ANA_SEN.h): se ignora
        if (!(IR_SENSORES_ACTIVOS & (1u << k))) { Normalizados[k] = 0.0f; continue; }

        //sensor sin rango util (no distingue fondo y linea): se ignora
        if (rango < (float)ir_rango_min) { Normalizados[k] = 0.0f; continue; }

        y = ((float)lecturas_ir_an[k] - (float)ir_minimos[k]) / rango;

        if (linea_color==blanco_)  y=1.0f-y;

        if (y < 0.0f) y = 0.0f;
        else if (y > 1.0f) y = 1.0f;

        Normalizados[k] = y;
    }
}

//normalizo datos  en rango de 0 a 1;
void Normaliza_sensores_filtrado(void)
{
    for (uint8_t k = 0; k < ir_n_sensores; k++)
    {
        float rango = (float)ir_maximos[k] - (float)ir_minimos[k];
        float y = 0.0f;

        //sensor desactivado en IR_SENSORES_ACTIVOS (IR_ANA_SEN.h): se ignora
        if (!(IR_SENSORES_ACTIVOS & (1u << k))) { Normalizados[k] = 0.0f; continue; }

        //sensor sin rango util (no distingue fondo y linea): se ignora
        if (rango < (float)ir_rango_min) { Normalizados[k] = 0.0f; continue; }

        y = (lecturas_filtradas[k] - (float)ir_minimos[k]) / rango;

        if (linea_color==blanco_)  y=1.0f-y;

        if (y < 0.0f) y = 0.0f;
        else if (y > 1.0f) y = 1.0f;

        Normalizados[k] = y;
    }
}

void Manda_Normalizados()
{
	char sen[200];
	sprintf(sen,"Nor=%.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/ %.2f/\r\n",
	Normalizados[0],Normalizados[1],Normalizados[2],Normalizados[3],Normalizados[4],Normalizados[5],Normalizados[6],Normalizados[7],Normalizados[8],Normalizados[9],
	Normalizados[10],Normalizados[11],Normalizados[12],Normalizados[13],Normalizados[14],Normalizados[15]);
	uartx_write_text(&IR_AN_UART, sen);
	uartx_write_text(&IR_AN_UART, "\r\n\r\n");
}


/* ANTERIOR
float Calcula_Pesos()
{
  char cuenta;
  float numerador=0,denominador=0;
  //float valor;
  float valor=0; //>>>>> CAMBIO: inicializada, si denominador==0 se devolvia basura
  float peso_min=ir_pesos[0];
  float peso_max=ir_pesos[ir_n_sensores-1];

  flag_linea=0;
  for (cuenta=0;cuenta<ir_n_sensores; cuenta++)
  {
    //linea blanca
	 peso=Normalizados[cuenta];


	 //este valor depende de sus sensores
	 	 if(peso>linea_blanco)  flag_linea=1;  //indica que al menos un sensor esta en la linea

			if(peso>0.1)//elimina reflejos irrelevantes  es el umbral u offset
			{
			 numerador+=Normalizados[cuenta]*ir_pesos[cuenta];
			 denominador+=Normalizados[cuenta];
			}

	    }//fin del for

	if(!flag_linea)//sino esta en la line  corrige
		//auna velocidad media
		{
			if (peso_anterior>0) return( peso_max); // si estaba muy a la derecha antes retorna el maximo valor a la derecha
			else  if (peso_anterior<0) return( peso_min);  // si estaba muy a la izquierda antes retorna el maximo valor a la izquierda

		}
	//si tod esta en la linea
	//if (numerador<0.001) return(peso_anterior);



 //si esta en la linea
	if (denominador!=0) valor=numerador/denominador;
 	   peso_anterior=valor;//  almacena el valor por si se llega a salir de la linea
       return(valor);
}
*/

//>>>>> CAMBIO: Calcula_Pesos corregida
// - usa variable local para el sensor, ya no pisa 'peso' (Manda_pesos ahora imprime la posicion)
// - si pierde la linea siempre retorna algo (antes con peso_anterior==0 se seguia de largo)
// - peso_anterior solo se guarda cuando hay una posicion valida
//>>>>> CAMBIO 03/11/2026: solo cuentan los sensores CERCA del maximo (ventana_peso a cada lado)
//      y cada uno aporta (valor - umbral_peso). Antes entraba al promedio todo sensor > 0.1:
//      un sensor lejano con ruido (0.2..0.4, LED apagado) movia la posicion aunque la linea
//      estuviera centrada (ej. I1 y D1 encendidos y posicion = 1.18)
float Calcula_Pesos()
{
  uint8_t cuenta;
  uint8_t k_max=0;
  float s_max=-1.0f;
  float numerador=0,denominador=0;
  float sensor;
  float peso_min=ir_pesos[0];
  float peso_max=ir_pesos[ir_n_sensores-1];

  //1) hay linea? y cual es el sensor mas alto
  flag_linea=0;
  for (cuenta=0;cuenta<ir_n_sensores; cuenta++)
  {
	 sensor=Normalizados[cuenta];

	 //este valor depende de sus sensores
	 if(sensor>linea_blanco)  flag_linea=1;  //indica que al menos un sensor esta en la linea

	 if(sensor>s_max) { s_max=sensor; k_max=cuenta; }
  }//fin del for

  //>>>>> CAMBIO 03/11/2026: confirmacion. Despues de perder la linea hay que verla
  //      ir_confirma veces seguidas para creerla: un pico aislado de un sensor (ej. el fondo
  //      alto de D8) ya no hace creer que la linea esta en un extremo
  static uint8_t perdida = 1, n_vista = 0;
  if (flag_linea) { if (n_vista < ir_confirma) n_vista++; }
  else            n_vista = 0;
  if (perdida && (n_vista < ir_confirma)) flag_linea = 0;   //todavia no confirmada

  //2) promedio ponderado SOLO en la ventana alrededor del maximo.
  //   La ventana va del PRIMER al ULTIMO sensor que esta casi en el maximo (empates):
  //   con todos en negro (cruce) entran los 16 y da 0; con la linea normal (1-2 sensores)
  //   queda ese par mas ventana_peso a cada lado.
  //   (antes se centraba solo en el primer maximo: todos en 4095 daba -6.5)
  if (flag_linea)
  {
	 uint8_t k_ini=k_max, k_fin=k_max;
	 for (cuenta=0; cuenta<ir_n_sensores; cuenta++)
	 {
		 if (Normalizados[cuenta] >= s_max-tolera_max)
		 {
			 if (cuenta<k_ini) k_ini=cuenta;
			 if (cuenta>k_fin) k_fin=cuenta;
		 }
	 }

	 int ini=(int)k_ini-ventana_peso;
	 int fin=(int)k_fin+ventana_peso;
	 if (ini<0) ini=0;
	 if (fin>ir_n_sensores-1) fin=ir_n_sensores-1;

	 for (int i=ini; i<=fin; i++)
	 {
		 sensor=Normalizados[i]-umbral_peso;   //lo que este bajo el umbral no aporta
		 if(sensor>0.0f)
		 {
			 numerador+=sensor*ir_pesos[i];
			 denominador+=sensor;
		 }
	 }
  }

  if(!flag_linea || denominador==0.0f)//sino esta en la linea corrige hacia el ultimo lado visto
  {
	  if (peso_anterior>0)      peso=peso_max; // estaba a la derecha, retorna el maximo a la derecha
	  else if (peso_anterior<0) peso=peso_min; // estaba a la izquierda, retorna el maximo a la izquierda
	  else                      peso=0.0f;     // nunca vio la linea, va al centro
	  perdida=1;
	  flag_linea=0;
	  return(peso);
  }

  //si esta en la linea
  perdida=0;
  peso=numerador/denominador;
  peso_anterior=peso;//  almacena el valor por si se llega a salir de la linea
  return(peso);
}

void Manda_pesos()
{
	char sen[200];
	sprintf(sen,"Peso=%.2f\r\n\r\n",peso);
	uartx_write_text(&IR_AN_UART, sen);
}
