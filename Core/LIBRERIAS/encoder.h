/*
 * encoder.h
 *
 *  Created on: Mar 18, 2025
 *      Author: ALCIDES_RAMOS
 *
 *  Libreria basica de encoder: lee RPM del motor a partir de los PPR del encoder.
 */

#ifndef LIBRERIAS_ENCODER_H_
#define LIBRERIAS_ENCODER_H_

#include "main.h"

//como usar
/*
 1) En encoder.c:
    - incluir los timer usados:     extern TIM_HandleTypeDef htim1;
    - declarar el encoder:          ENCODERS ENCODER1 = {&htim1,TIM1};
    - definir signos:               ENCODERx_DIR (pulsos/vueltas)  ENCODERx_VEL (rpm)
    - en Encoder_velo agregar:      Encoder_actualiza(&ENCODER1, ENCODER1_DIR, ENCODER1_VEL);
    - ajustar si se desea:          ENCODER_TS_MS (periodo de muestreo) y ENCODER_FILTRO

 2) En "stm32h5xx_it.c", dentro de SysTick_Handler (cada 1 ms):
      Encoder_velo();
    e incluir #include "encoder.h"

 3) En main.c, despues de iniciar los timers (cada encoder puede tener PPR distinto):
      //          encoder   PPR  modo  bits del timer
      Encoder_init(&ENCODER1, 7,   4,    16);
      Encoder_init(&ENCODER2, 7,   4,    32);

      //en el while
    y usar las variables:
      ENCODER1.rpm       -> RPM del motor (filtrada)
      ENCODER1.rpm_cruda -> RPM del motor sin filtro
      ENCODER1.pulsos    -> pulsos acumulados (cuentas x modo)
      ENCODER1.vueltas   -> vueltas acumuladas del motor

      sprintf(texto,"PUL1=%ld PUL2=%ld RPM1=%.0f RPM2=%.0f\r\n",
              (long)ENCODER1.pulsos,(long)ENCODER2.pulsos,ENCODER1.rpm,ENCODER2.rpm);

//en teleplo
  sprintf(texto,">PUL1:%ld\n",(long)ENCODER1.pulsos);
  uartx_write_text(&huart1, texto);

  sprintf(texto,">PUL2:%ld\n",(long)ENCODER2.pulsos);
  uartx_write_text(&huart1, texto);

  sprintf(texto,">RPM1:%.0f\n",ENCODER1.rpm);
  uartx_write_text(&huart1, texto);

  sprintf(texto,">RPM2:%.0f\n",ENCODER2.rpm);
  uartx_write_text(&huart1, texto);

  HAL_Delay(20);   //~50 muestras por segundo en la grafica
              */

//periodo de muestreo de la velocidad en milisegundos (Encoder_velo se llama cada 1 ms desde SysTick)
//OJO: el PID de velocidad usa el mismo periodo -> PID_VEL_TS (PID.h) = ENCODER_TS_MS / 1000
#define ENCODER_TS_MS  5

typedef struct {
    TIM_HandleTypeDef *htim;  //handle del timer en modo encoder
    TIM_TypeDef *TIMX;        //instancia del timer (TIM1, TIM2...)

    //configuracion (la pone Encoder_init)
    uint16_t ppr;             //pulsos por vuelta del encoder (un canal)
    uint8_t  modo;            //4 = x4 (TI1 y TI2), 2 = x2
    uint8_t  numbits;         //16 o 32 bits del timer
    uint16_t cpr;             //cuentas por vuelta del motor = ppr * modo
    float    k_rpm;           //factor: cuentas por ventana -> rpm

    //estado interno
    uint32_t cnt_ant;         //contador del timer en la muestra anterior

    //resultados
    int32_t  delta;           //cuentas en la ultima ventana de muestreo
    int32_t  pulsos;          //cuentas acumuladas (signo segun ENCODERx_DIR)
    float    vueltas;         //vueltas acumuladas del motor (signo segun ENCODERx_DIR)
    float    rpm_cruda;       //rpm del motor sin filtro (signo segun ENCODERx_VEL)
    float    rpm;             //rpm del motor filtrada (signo segun ENCODERx_VEL)
} ENCODERS;

extern ENCODERS ENCODER1;
extern ENCODERS ENCODER2;
extern volatile uint8_t encoder_muestra;   //1 = muestra nueva de rpm (cada ENCODER_TS_MS)

void Encoder_init(ENCODERS *encoder, uint16_t ppr, uint8_t modo, uint8_t numbits);
void Encoder_reset(ENCODERS *encoder);
void Encoder_velo(void);

#endif /* LIBRERIAS_ENCODER_H_ */
