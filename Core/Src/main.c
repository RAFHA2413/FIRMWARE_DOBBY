/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"

#include "pinout1.h"
#include "bits.h"
#include "uart.h"
#include "UARTRX.h"
#include "adc_x.h"
#include "ws2812b.h"
#include "delays.h"
#include "DRV8871.h"
#include "IR_ANA_SEN.h"
#include "PWMB.h"
#include "MPU6050.h"
#include "kalman.h"
#include "timer.h"
#include "I2C_SCAN_BUS.h"
#include "encoder.h"
#include "llanta.h"
#include "parametros.h"
#include "sd_card.h"
#include "ircodigo.h"
#include "PID.h"
#include "telemetria.h"
#include "pruebas.h"
#include "carrera.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_NodeTypeDef Node_GPDMA2_Channel0;
DMA_QListTypeDef List_GPDMA2_Channel0;
DMA_HandleTypeDef handle_GPDMA2_Channel0;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
DMA_HandleTypeDef handle_GPDMA1_Channel1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

/* USER CODE BEGIN PV */
char procesa[100];
uint32_t milis_pixel=0;
uint32_t milis_musica=0;
uint32_t milis_pid=0;

enum pixel_m_estado {apagado,rojo,verde, azul,rosado,morado,blanco,amarillo};
uint8_t  estado_pixel=apagado;
#define retardo_pixe 50
#define retardo_musica 55000
char texto[600];
float pesos_ARZ;


//variables para iniciar o pearr el robot
uint8_t R_START=0;
uint8_t R_STOP=0;
uint8_t  PID_VAR=0;
uint8_t cancion=1;
int8_t volumen=25;
extern uint8_t flag_fuera;

uint32_t recta=7500;
uint32_t  recta_inicio=0;
uint8_t flag_recta=0;
uint32_t recta_velo=12000;


//flag_text ya no se usa: la bandera de TEST es PRUEBA.activo (pruebas.h)
uint8_t  luces=0;     //LUZON / LUZOFF: 1 = WS2812 en blanco
uint8_t  play=0;      //musica sonando (falta la libreria mp3)
//codigos ir (IRSTART, IRSTOP...): ver ircodigo.h

//ADC
uint16_t codigo;
float voltaje;
float bateria;

float  sd_kp,sd_ki,sd_kd,sd_tau,sd_kaw,velo_giro;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_GPDMA2_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lee_ir()
{
	uint32_t codigo_ir;
	if (IR_leer(&codigo_ir))   //1 = llego un codigo (ya queda rearmado)
	{
     PIN_BLINK(LED);
	sprintf(texto,"codigo=0x%08lX/\r\n",(unsigned long)codigo_ir);
	uartx_write_text(&huart1, texto);
	if ((codigo_ir==IRSTART)||(codigo_ir==IRSTART2))
		{
		 uartx_write_text(&huart1, "START\r\n");
       R_START=1;
       R_STOP=0;
       Carrera_run();          //REPOSO -> PID
       //Analizar si debo colocar la rampa de arranque.
   //    if (rampamot==1)   Motores_Arranque();//aplico rampa de arranque


//		WS2812_Color(0, 255,0 );
		   WS2812_Color(0, 0,0 );

       WS2812_Manda_Trama();
		}
	  if ((codigo_ir==IRSTOP) ||(codigo_ir==IRSTOP2))
		  {
		 uartx_write_text(&huart1, "STOP\r\n");
		  R_STOP=1;
		  R_START=0;
		  Carrera_stop();       //-> REPOSO frenando
		  WS2812_Color(255, 0,0 );
		  	WS2812_Manda_Trama();

		  }

	  if ((codigo_ir==IRPROG))
	  		  {
	  		 uartx_write_text(&huart1, "PROG1\r\n");
	  	//	 mp3_modo_play();
	  	//	 mp3_next();

	  		  }


	  if ((codigo_ir==IRPROG2))
	  		  {
	  		 uartx_write_text(&huart1, "PROG2\r\n");
	  	//	  mp3_modo_pausa();

	  		  }


	}

}

//=============================================================================
//                   COMANDOS DEL PANEL (dobby_panel.py / celular)
//=============================================================================
#define USA_MP3   0     //1 cuando se agregue la libreria mp3 (mp3_next, mp3_play_x ...)

//aviso en el WS2812 sin HAL_Delay: el delay hacia perder los comandos que llegaban
static uint8_t  aviso = 0;
static uint32_t t_aviso = 0;

//color de fondo del WS2812 segun el estado del robot
static void Color_base(void)
{
  if (PRUEBA.activo) WS2812_Color(80, 60, 0);       //amarillo: TEST
  else if (luces)    WS2812_Color(255, 255, 255);   //luces encendidas
  else               WS2812_Color(0, 0, 0);
  WS2812_Manda_Trama();
}

//parpadeo magenta: llego un comando (se apaga solo en Aviso_tarea)
static void Aviso_on(void)
{
  WS2812_Color(255, 0, 255);
  WS2812_Manda_Trama();
  aviso = 1;
  t_aviso = HAL_GetTick();
}

//en el while: termina el parpadeo a los 80 ms
void Aviso_tarea(void)
{
  if (aviso && (HAL_GetTick() - t_aviso >= 80))
  {
    aviso = 0;
    Color_base();
  }
}

//busca "clave" en la trama y lee el numero que sigue (ej. "PROPOR=50.0/" -> 50.0)
static uint8_t lee_num(const char *trama, const char *clave, float *valor)
{
  const char *p = strstr(trama, clave);
  if (p == 0) return 0;
  *valor = strtof(p + strlen(clave), 0);
  return 1;
}

//robot detenido: se puede grabar flash, calibrar sensores...
static uint8_t Robot_quieto(void)
{
  return (CARRERA.estado == CARRERA_REPOSO) && (LLANTA_CAL.modo == LLANTA_OFF);
}

//procesa UN comando (copia de UARTRX1.trama_rx; la recepcion ya esta rearmada)
//ORDEN: primero las librerias, despues los comandos generales.
//OJO con strstr: "TAU=" tambien esta dentro de "VTAU=" -> los V... los toma antes Llanta_comando
void procesa_rx(char *trama)
{
  float v;
  uint8_t pid_dir = 0;    //1 = cambio el PID de linea: aplicar y avisar al panel

  Aviso_on();

  //---------------- 1) TEST: TESTON, TESTOFF, TESTSEL=, PWMIZQ=, PWMDER=
  if (Prueba_comando(trama)) return;

  //---------------- 2) STOP general (antes que Llanta_comando, que tambien lo toma)
  if (strstr(trama, "STOP"))
  {
    R_START = 0;
    R_STOP  = 0;
    Carrera_stop();          //-> REPOSO: apaga PID, test y frena
    return;
  }

  //---------------- 3) PID de velocidad: VKP= VKI= VKD= VTAU= VKAW= VIZQ= VDER= VPID= VESC= VIDENT=
  if (Llanta_comando(trama)) return;

  //---------------- 4) carrera
  if (strstr(trama, "RUN"))
  {
    R_START = 1;
    R_STOP  = 0;
    Carrera_run();           //REPOSO -> PID (sigue la linea, ver carrera.h)
    return;
  }

  //---------------- 5) parametros, SD y flash
  if (strstr(trama, "PIDVAL"))   { manda_pid_parametros(); return; }

  if (strstr(trama, "DAT/"))     //trama completa de parametros (ver parametros.h)
  {
    if (Param_desde_trama(trama)) { Param_aplica(); manda_pid_parametros(); }
    else uartx_write_text(&huart1, "DAT: trama incompleta\r\n");
    return;
  }

  if (strstr(trama, "GUARDASD"))  //guarda PARAM en la SD (datos.txt) y en la flash
  {
    Param_a_trama(texto);
    uartx_write_text(&huart1, SD_WRITE_TEXT(file_name, texto) ? "SD: guardado\r\n"
                                                               : "SD: error al guardar\r\n");
    if (Robot_quieto())
      uartx_write_text(&huart1, Param_guardar() ? "FLASH: guardado\r\n" : "FLASH: error\r\n");
    else
      uartx_write_text(&huart1, "FLASH: no se graba con el robot andando\r\n");
    return;
  }

  if (strstr(trama, "LEESD"))     //lee PARAM de la SD y lo aplica
  {
    if (SD_READ_LINE_N(file_name, 0) && Param_desde_trama(SD_BUFFER))
    {
      Param_aplica();
      uartx_write_text(&huart1, "SD: parametros leidos\r\n");
      manda_pid_parametros();
    }
    else uartx_write_text(&huart1, "SD: no se pudo leer los parametros\r\n");
    return;
  }

  if (strstr(trama, "CALI"))      //calibra los sensores de linea
  {
    if (!Robot_quieto()) { uartx_write_text(&huart1, "CALI: detener el robot\r\n"); return; }
    Calibrar_sensores();
    Manda_Limites();
    return;
  }

  //---------------- 6) luces
  if (strstr(trama, "LUZON"))  { luces = 1; return; }   //Aviso_tarea pone el color
  if (strstr(trama, "LUZOFF")) { luces = 0; return; }

  //---------------- 7) PID de linea (PARAM -> Param_aplica)
  if      (strstr(trama, "KP+")) { PARAM.kp += 1.0f;  pid_dir = 1; }
  else if (strstr(trama, "KP-")) { PARAM.kp -= 1.0f;  pid_dir = 1; }
  else if (strstr(trama, "KI+")) { PARAM.ki += 0.01f; pid_dir = 1; }
  else if (strstr(trama, "KI-")) { PARAM.ki -= 0.01f; pid_dir = 1; }
  else if (strstr(trama, "KD+")) { PARAM.kd += 0.1f;  pid_dir = 1; }
  else if (strstr(trama, "KD-")) { PARAM.kd -= 0.1f;  pid_dir = 1; }
  else if (lee_num(trama, "PROPOR=",  &v)) { PARAM.kp  = v; pid_dir = 1; }
  else if (lee_num(trama, "INTEGRA=", &v)) { PARAM.ki  = v; pid_dir = 1; }
  else if (lee_num(trama, "DERIVA=",  &v)) { PARAM.kd  = v; pid_dir = 1; }
  else if (lee_num(trama, "TAU=",     &v)) { PARAM.tau = v; pid_dir = 1; }   //en multiplos de Ts
  else if (lee_num(trama, "KAW=",     &v)) { PARAM.kaw = v; pid_dir = 1; }   //factor de Ki/Kp
  else if (lee_num(trama, "VGIRO=",   &v)) { PARAM.vel_giro = v; pid_dir = 1; }
  else if (lee_num(trama, "KGIRO=",   &v)) { PARAM.kg = v;       pid_dir = 1; }
  else if (lee_num(trama, "VBASICA=", &v)) { PARAM.velo_basica = v; pid_dir = 1; }
  else if (strstr(trama, "VC+"))           { PARAM.velo_basica += 1.0f; pid_dir = 1; }
  else if (strstr(trama, "VC-"))           { PARAM.velo_basica -= 1.0f; pid_dir = 1; }

  //rectas (carrera.c, estado RECTA)
  else if (lee_num(trama, "RECTERR=",  &v)) { PARAM.error_recta = v; pid_dir = 1; }   //Error_LIM (posicion)
  else if (lee_num(trama, "RECTGIR=",  &v)) { PARAM.giro_recta  = v; pid_dir = 1; }   //GIRO_LIM (grados/s)
  else if (lee_num(trama, "RECTVEL=",  &v)) { PARAM.velo_recta  = v; pid_dir = 1; }   //V_LINEA (PWM)
  else if (lee_num(trama, "RECTLARG=", &v)) { PARAM.largo_recta = v; pid_dir = 1; }   //TRAMO (cm)

  //freno al salir de la recta (carrera.c, estado FRENO)
  else if (lee_num(trama, "FRENOV=",  &v)) { PARAM.velo_freno = v; pid_dir = 1; }     //V_FRENO (PWM)
  else if (lee_num(trama, "FRENOMS=", &v)) { PARAM.freno_ms   = v; pid_dir = 1; }     //T_FRENO (ms)

  //color de linea: 0 = blanca, 1 = negra (IR_ANA_SEN.h)
  else if (lee_num(trama, "LINCO=", &v))    { PARAM.linea_color = (v > 0.5f) ? 1 : 0; pid_dir = 1; }

  if (pid_dir)
  {
    Param_aplica();          //PID_Sintoniza + limites + color: los cambios SI tienen efecto
    manda_pid_parametros();  //el panel ve en rojo los valores que quedaron
    return;
  }

  //---------------- 8) ajuste de la velocidad de cada llanta (m/s) desde el celular
  if (strstr(trama, "VD+") || strstr(trama, "VD-") || strstr(trama, "VI+") || strstr(trama, "VI-"))
  {
    if      (strstr(trama, "VD+")) LLANTA_DER.vref += 0.1f;
    else if (strstr(trama, "VD-")) LLANTA_DER.vref -= 0.1f;
    else if (strstr(trama, "VI+")) LLANTA_IZQ.vref += 0.1f;
    else                           LLANTA_IZQ.vref -= 0.1f;
    sprintf(texto, "*x%.2f**y%.2f*", LLANTA_DER.vref, LLANTA_IZQ.vref);
    uartx_write_text(&huart1, texto);
    return;
  }

  //---------------- 9) musica
  if (strstr(trama, "NEXT") || strstr(trama, "PREV") || strstr(trama, "PLAY") ||
      strstr(trama, "SILE") || strstr(trama, "V+")   || strstr(trama, "V-")   ||
      strstr(trama, "SUENA="))
  {
#if USA_MP3
    if      (strstr(trama, "NEXT")) { if (play) mp3_next(); }
    else if (strstr(trama, "PREV")) { if (play) mp3_previo(); }
    else if (strstr(trama, "PLAY")) { play = 1; mp3_modo_play(); mp3_next(); }
    else if (strstr(trama, "SILE")) { play = 0; mp3_modo_reset(); }
    else if (strstr(trama, "V+"))   { if (++volumen > 30) volumen = 30; if (play) mp3_volumen_x(volumen); }
    else if (strstr(trama, "V-"))   { if (--volumen < 0)  volumen = 0;  if (play) mp3_volumen_x(volumen); }
    else if (lee_num(trama, "SUENA=", &v)) mp3_play_x((uint8_t)v);
#else
    uartx_write_text(&huart1, "MP3: falta la libreria (USA_MP3 = 0)\r\n");
#endif
    return;
  }

  //comando desconocido: eco para verlo en TRAMA
  uartx_write_text(&huart1, trama);
  uartx_write_text(&huart1, "\r\n");
}

//robot QUIETO: cada 200 ms manda posicion de la linea, giro y velocidades
//(en carrera, cuando exista el lazo, usar manda_operando)
void Tarea_espera(void)
{
  static uint32_t t = 0;
  if (CARRERA.estado != CARRERA_REPOSO) return;   //en carrera manda Carrera_tarea
  if (HAL_GetTick() - t < 200) return;
  t = HAL_GetTick();

  Lee_sensores();
  filtra_sensores();
  Normaliza_sensores();
  Calcula_Pesos();          //deja la posicion en 'peso'
  manda_espera_start();
  Manda_Normalizados();     //Nor=... -> LEDs I8..D8 del panel (ver que sensores sirven)
  manda_crudos();           //Cru=... -> numero crudo del ADC debajo de cada LED
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_GPDMA2_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
uartx_write_text(&huart1,"DOBBY 2026\r\n");

//cargo los parametros guardados en flash (o los de defecto si no hay)
 if (Param_init()) uartx_write_text(&huart1,"PARAMETROS: flash\r\n");
 else              uartx_write_text(&huart1,"PARAMETROS: defecto\r\n");
 Param_a_trama(texto);
 uartx_write_text(&huart1,texto);
//PID de linea con los parametros cargados (antes PID_DIR nunca se iniciaba)
 PID_DIR_INIT();
 Param_aplica();

//prueba de la SD: monta, muestra espacio, lee test.txt y escribe dobby.txt
 if (SD_INIT())
 {
   uint32_t total_kb, libre_kb;
   uartx_write_text(&huart1,"SD: montada\r\n");

   if (SD_ESPACIO(&total_kb, &libre_kb))
   {
     sprintf(texto,"SD: total=%lu KB  libre=%lu KB\r\n",(unsigned long)total_kb,(unsigned long)libre_kb);
     uartx_write_text(&huart1,texto);
   }

   if (SD_READ_LINE_N("test.txt", 0))
   {
     uartx_write_text(&huart1,"SD: test.txt -> ");
     uartx_write_text(&huart1,SD_BUFFER);
     uartx_write_text(&huart1,"\r\n");
   }
   else uartx_write_text(&huart1,"SD: no se pudo leer test.txt\r\n");

   if (SD_WRITE_TEXT("dobby.txt", "Hola desde DOBBY 2026\r\n"))
     uartx_write_text(&huart1,"SD: dobby.txt escrito\r\n");
   else
     uartx_write_text(&huart1,"SD: error al escribir\r\n");
 }
 else
 {
   sprintf(texto,"SD: no montada (error FatFS %d)\r\n",(int)fres);
   uartx_write_text(&huart1,texto);
 }
//SI NECEDITA VER BUS  I2C
 //I2C_SCAN(hi2c1);
//enciendo leds ir
//PIN_ON(IR_ON);
  PIN_ON(LEDR1);
  PIN_OFF(LEDR2);
  //inicio motores
 DRV8871_Init();

 

  //TIMER_INIT();
  //>>>>> CAMBIO: TIMER_INIT se movio despues de MPU6050_Calibra (ver abajo)

delay_Init();

      WS2812_Color(255,255,255);
    WS2812_Manda_Trama();
    HAL_Delay(500);


  Sensores_init();

  //imu
  if (MPU6050_init()==1)  uartx_write_text(&huart1, "IMU OKS\r\n");
    else   uartx_write_text(&huart1, "IMU BAD\r\n");
     MPU6050_Calibra();

  TIMER_INIT();

//calibro sensores ir

   WS2812_Color(0,0,50);
    WS2812_Manda_Trama();

  PIN_ON(LEDS_IR);
  Calibrar_sensores();
  //Manda_Lecturas_filtrada()
  Manda_Limites(); //MANDA MAXIMOS Y MINIMOS

   WS2812_Color(255,0,0);
    WS2812_Manda_Trama();
//inicio los encoders
//           encoder   PPR modo bits
 Encoder_init(&ENCODER1, 7,  2,  16);
 Encoder_init(&ENCODER2, 7,  2,  32);

//inicio las llantas (motor JGA12-N30-10: reductora 10:1, llanta 23 mm)
//          llanta       encoder    reductora  diametro(mm)
 Llanta_init(&LLANTA_IZQ, &ENCODER1, 10.0f,     23.0f);
 Llanta_init(&LLANTA_DER, &ENCODER2, 10.0f,     23.0f);
//separacion entre llantas (mm)
 Robot_init(160.0f);
//PID de velocidad de las llantas (valores de PID.h) + calibracion desde el panel
 Llanta_control_init();
//maquina de estados de la carrera (arranca en REPOSO; corre en el TIM6)
 Carrera_init();

 //MD_Adelante(800);
 //MI_Adelante(800);
//control IR
 IR_init();
 //IR_calibrar(10);   //SOLO para sacar los codigos: imprime 10 y sigue. Luego comentar
 uartRX_it_idle_dma_init( &UARTRX1);
 PIN_BLINK(LED1);   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//CALIBRACION DEL PID DE VELOCIDAD con el panel (dobby_panel.py, pestaña VELO)
//procedimiento completo en llanta.h
 manda_pid_parametros();   //el panel arranca viendo las constantes actuales
  while(1)
  {
    lee_ir();
    //START / STOP del control IR: los procesa lee_ir con Carrera_run / Carrera_stop

    if (UARTRX1.flag_rx==1)
	  {
	      //copia el comando y rearma la recepcion YA: asi no se pierde lo que llega
	      //mientras se procesa (SD, flash y CALI tardan)
	      char cmd[256];
	      strncpy(cmd, (char*)UARTRX1.trama_rx, sizeof(cmd) - 1);
	      cmd[sizeof(cmd) - 1] = 0;
		  uartRX_DMA_Re_init(&UARTRX1);
	      procesa_rx(cmd); //procedimeinto donde se procesan los datos
	  }

    Aviso_tarea();    //apaga el parpadeo magenta de "llego un comando"
    Tarea_espera();   //robot quieto: posicion, giro y velocidades para el panel
    Carrera_tarea();  //estado de la carrera y telemetria de carrera
    Prueba_tarea();   //maquina de estados del TEST (solo corre con TEST ON)
    Llanta_tarea();   //graficas del panel + resultados de la identificacion
    //sin HAL_Delay: Llanta_tarea lleva sus propios tiempos

  }


while(1)
  {
  //datos para Teleplot: formato ">nombre:valor\n"
  sprintf(texto,">RPM1:%.0f\n",ENCODER1.rpm);           //rpm motor
  uartx_write_text(&huart1, texto);

  sprintf(texto,">RPM2:%.0f\n",ENCODER2.rpm);
  uartx_write_text(&huart1, texto);

  sprintf(texto,">VIZQ:%.3f\n",LLANTA_IZQ.velocidad);   //m/s
  uartx_write_text(&huart1, texto);

  sprintf(texto,">VDER:%.3f\n",LLANTA_DER.velocidad);
  uartx_write_text(&huart1, texto);

  sprintf(texto,">VROBOT:%.3f\n",ROBOT.velocidad);      //m/s centro del robot
  uartx_write_text(&huart1, texto);

  sprintf(texto,">GIRO:%.2f\n",ROBOT.giro);             //rad/s
  uartx_write_text(&huart1, texto);

  sprintf(texto,">DIST:%.3f\n",ROBOT.distancia);        //m
  uartx_write_text(&huart1, texto);

  HAL_Delay(20);   //~30 muestras por segundo en la grafica


  }
/*while (1)
  {
  	 sprintf(texto,"giro=%.2f\r\n\r\n",giro_z_filtrado);
      uartx_write_text(&huart1, texto); 
      HAL_Delay(1000);
	
  
  }
*/
while (1)
    {



 //analiza si solo lee o filtra
    Lee_sensores();
   filtra_sensores();
   Normaliza_sensores();
	 	Manda_Normalizados();
 //   filtra_sensores();
  //Manda_Lecturas();
	      HAL_Delay(1000);

    }

  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //uartx_write_text(&huart1,"OPERANDO\r\n");
    WS2812_Color(0,255,0);
    WS2812_Manda_Trama();
    HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 16;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_16;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief GPDMA2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA2_Init(void)
{

  /* USER CODE BEGIN GPDMA2_Init 0 */

  /* USER CODE END GPDMA2_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA2_CLK_ENABLE();

  /* GPDMA2 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA2_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA2_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA2_Init 1 */

  /* USER CODE END GPDMA2_Init 1 */
  /* USER CODE BEGIN GPDMA2_Init 2 */

  /* USER CODE END GPDMA2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C043E5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 312;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2500-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 250-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SW_IN_GPIO_Port, SW_IN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|LEDS_IR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED1_Pin|LEDR1_Pin|LEDR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SW_IN_Pin */
  GPIO_InitStruct.Pin = SW_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SW_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin LEDS_IR_Pin */
  GPIO_InitStruct.Pin = LED_Pin|LEDS_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LEDR1_Pin LEDR2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LEDR1_Pin|LEDR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DET_Pin */
  GPIO_InitStruct.Pin = SD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STOP_Pin */
  GPIO_InitStruct.Pin = STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_in_Pin */
  GPIO_InitStruct.Pin = IR_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_in_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  MPU_Attributes_InitTypeDef MPU_AttributesInit = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region 0 and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08FFF000;
  MPU_InitStruct.LimitAddress = 0x08FFFFFF;
  MPU_InitStruct.AttributesIndex = MPU_ATTRIBUTES_NUMBER0;
  MPU_InitStruct.AccessPermission = MPU_REGION_ALL_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Attribute 0 and the memory to be protected
  */
  MPU_AttributesInit.Number = MPU_ATTRIBUTES_NUMBER0;
  MPU_AttributesInit.Attributes = INNER_OUTER(MPU_NOT_CACHEABLE);

  HAL_MPU_ConfigMemoryAttributes(&MPU_AttributesInit);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
