/*
 * carrera.c
 *
 *  Created on: Nov 3, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "carrera.h"
#include "llanta.h"
#include "PID.h"
#include "parametros.h"
#include "IR_ANA_SEN.h"
#include "DRV8871.h"
#include "pruebas.h"
#include "telemetria.h"
#include "uart.h"
#include "timer.h"          //giro_z_filtrado (IMU, se lee en el TIM6 antes de Carrera_control)
#include <stdio.h>

CARRERAS CARRERA = {CARRERA_REPOSO};

static char tx[40];

#if CARRERA_COMP_ADAPTATIVO
// Persisten entre ciclos de control de 5 ms; se reinician con RUN.
static float comp_e_anterior = 0.0f;
static float comp_tendencia = 0.0f;
static uint8_t comp_tiene_muestra = 0;

// Instrumentacion diagnostica, sin modificar el control V2:
// contadores de muestras a 5 ms; serial solo al detener (fuera de la ISR).
static volatile uint32_t comp_total_muestras = 0;
static volatile uint32_t comp_recta_muestras = 0;
static volatile uint32_t comp_limit_muestras = 0;
static volatile uint32_t comp_recorte_suma_pwm = 0;
static volatile uint32_t comp_recorte_max_pwm = 0;
static volatile uint32_t comp_max_error_x100 = 0;
// Datos de estado y calidad de seguimiento (5 ms por muestra).
static volatile uint32_t comp_pid_muestras = 0;
static volatile uint32_t comp_freno_muestras = 0;
static volatile uint32_t comp_arranque_muestras = 0;
static volatile uint32_t comp_sin_linea_muestras = 0;
static volatile uint32_t comp_sin_linea_racha = 0;
static volatile uint32_t comp_sin_linea_racha_max = 0;
static volatile uint8_t comp_reporte_pendiente = 0;
#endif


//velocidad base pedida (V_Bas del panel), limitada por seguridad
static float Carrera_v_base(void)
{
  float v = PARAM.velo_basica;                         //PWM 0..1000
  if (v < 0.0f) v = 0.0f;
  if (v > CARRERA_PWM_MAX) v = CARRERA_PWM_MAX;
  return v;
}

//(opcional, CARRERA_KV_CURVA) velocidad base segun el IMU: si gira rapido (curva) baja
//v = V_Bas - KV_CURVA * |giro_z|, sin bajar de CARRERA_V_MIN_CURVA
static float Carrera_v_curva(float v_base)
{
  if (CARRERA_KV_CURVA <= 0.0f) return v_base;         //apagado

  float g = giro_z_filtrado;                           //grados/s
  if (g < 0.0f) g = -g;

  float v = v_base - CARRERA_KV_CURVA * g;
  float v_min = (CARRERA_V_MIN_CURVA < v_base) ? CARRERA_V_MIN_CURVA : v_base;
  if (v < v_min) v = v_min;
  return v;
}

//sensores -> posicion de la linea (0 = centrada)
static void Carrera_lee_linea(void)
{
  Lee_sensores();
  Normaliza_sensores();
  CARRERA.posicion = CARRERA_SIGNO * Calcula_Pesos();
}


//------------------------------------------------ RECTAS
//rectas encendidas: Error_LIM, TRAMO y V_LINEA > 0
static uint8_t Carrera_rectas_on(void)
{
  return (PARAM.error_recta > 0.0f) && (PARAM.largo_recta > 0.0f) && (PARAM.velo_recta > 0.0f);
}

//1 = la muestra actual parece recta: ve la linea, |posicion| <= Error_LIM
//y, si GIRO_LIM > 0, el robot casi no gira (|giro_z| <= GIRO_LIM)
//(llamar despues de Carrera_sigue_linea, que actualiza la posicion)
static uint8_t Carrera_es_recta(void)
{
  if (!flag_linea) return 0;

  float e = CARRERA.posicion;
  if (e < 0.0f) e = -e;
  if (e > PARAM.error_recta) return 0;

  if (PARAM.giro_recta > 0.0f)
  {
    float g = giro_z_filtrado;                         //grados/s
    if (g < 0.0f) g = -g;
    if (g > PARAM.giro_recta) return 0;
  }
  return 1;
}

//mide el tramo recto con la distancia del encoder (ROBOT.distancia).
//1 = esta muestra es recta (recta_cm = cm seguidos);  0 = no lo es: el tramo vuelve a 0
static uint8_t Carrera_mide_recta(void)
{
  if (!Carrera_rectas_on() || !Carrera_es_recta())
  {
    CARRERA.recta_ini = ROBOT.distancia;
    CARRERA.recta_cm  = 0.0f;
    return 0;
  }

  float d = ROBOT.distancia - CARRERA.recta_ini;       //m
  if (d < 0.0f) d = -d;                                //por si el encoder cuenta al reves
  CARRERA.recta_cm = d * 100.0f;
  return 1;
}

//V_LINEA: velocidad base en recta (PWM 0..1000, Param_aplica ya revisa el rango)
static float Carrera_v_recta(void)
{
  return PARAM.velo_recta;
}

//sale de la recta: FRENO si va mas rapido que V_Bas (y T_FRENO > 0); si no, PID
static void Carrera_sale_recta(void)
{
  float v_bas = Carrera_v_base() * CARRERA_MPS_POR_PWM;          //m/s
  CARRERA.estado = ((PARAM.freno_ms > 0.0f) &&
                    (ROBOT.velocidad > v_bas + CARRERA_FRENO_MARGEN)) ? CARRERA_FRENO
                                                                      : CARRERA_PID;
  CARRERA.cuenta = 0;
}

//empieza a contar un tramo nuevo desde donde esta el robot
static void Carrera_recta_reset(void)
{
  CARRERA.recta_ini = ROBOT.distancia;
  CARRERA.recta_cm  = 0.0f;
}


//se llama una vez al iniciar
void Carrera_init(void)
{
  CARRERA.estado   = CARRERA_REPOSO;
  CARRERA.cuenta   = 0;
  CARRERA.giro     = 0.0f;
  CARRERA.v_base   = 0.0f;
  CARRERA.frenando = 0;
}

//REPOSO -> ARRANQUE -> PID   (orden RUN por serial o START del control IR)
void Carrera_run(void)
{
  if (CARRERA.estado != CARRERA_REPOSO) return;        //ya esta corriendo

  if (PRUEBA.activo) Prueba_activa(0);                 //el test no puede mover motores en carrera

  CARRERA.frenando = 0;
  PID_Reset(&PID_DIR);
  PID_DIR.setpoint = 0.0f;                             //linea centrada
  CARRERA.giro = 0.0f;
  CARRERA.cuenta = 0;
  CARRERA.sin_linea = 0;
  CARRERA.aviso_perdido = 0;
#if CARRERA_COMP_ADAPTATIVO
  comp_e_anterior = 0.0f;
  comp_tendencia = 0.0f;
  comp_tiene_muestra = 0;
  comp_total_muestras = 0;
  comp_recta_muestras = 0;
  comp_limit_muestras = 0;
  comp_recorte_suma_pwm = 0;
  comp_recorte_max_pwm = 0;
  comp_max_error_x100 = 0;
  comp_pid_muestras = 0;
  comp_freno_muestras = 0;
  comp_arranque_muestras = 0;
  comp_sin_linea_muestras = 0;
  comp_sin_linea_racha = 0;
  comp_sin_linea_racha_max = 0;
  comp_reporte_pendiente = 0;
#endif
  Carrera_recta_reset();

  //rampa: arranca de una vez en V_INICIAL (rompe la inercia), sin pasarse de V_Bas
  float v_obj = Carrera_v_base();
  CARRERA.v_base = (CARRERA_V_INICIAL < v_obj) ? CARRERA_V_INICIAL : v_obj;

  Llanta_set_velocidad(0.0f, 0.0f);
  Llanta_modo(LLANTA_PID);                             //enciende los PID de velocidad

  CARRERA.estado = CARRERA_ARRANQUE;                   //al final: habilita Carrera_control
  uartx_write_text(&TELE_UART, "CARRERA: ARRANQUE\r\n");
}

//pasa a REPOSO frenando. Sin uart: se puede llamar tambien desde la interrupcion
static void Carrera_frena(void)
{
  CARRERA.estado = CARRERA_REPOSO;                     //primero: Carrera_control deja de mandar
  CARRERA.frenando = 0;                                //(que la interrupcion no suelte el freno)

  Llanta_modo(LLANTA_OFF);                             //apaga los PID de velocidad

  CARRERA.giro = 0.0f;
  CARRERA.v_base = 0.0f;
  CARRERA.sin_linea = 0;
  CARRERA.cuenta = 0;
  MD_Freno();                                          //freno activo (los dos pines en 1)
  MI_Freno();
  CARRERA.frenando = 1;                                //al final: cuenta CARRERA_FRENO_MS y suelta
#if CARRERA_COMP_ADAPTATIVO
  comp_reporte_pendiente = 1;                           // Carrera_tarea reporta fuera de la ISR
#endif
}

//cualquier estado -> REPOSO frenando (orden STOP por serial o STOP del control IR)
void Carrera_stop(void)
{
  if (PRUEBA.activo) Prueba_activa(0);                 //STOP tambien apaga el test
  Carrera_frena();
  uartx_write_text(&TELE_UART, "CARRERA: REPOSO\r\n");
}

// DOBBY Competition V2. En RECTA, la velocidad objetivo se limita solo
// cuando el error lateral (actual + su tendencia positiva filtrada) crece.
// Si el error decrece, se evita limitar por una desviacion residual grande.
// El valor inferior de la consigna es V_Bas; mantiene el modo FRENO original.
#if CARRERA_COMP_ADAPTATIVO
static float Carrera_comp_limita_recta(float v_pedida, float e_abs)
{
  if (CARRERA_COMP_ERR_FIN <= CARRERA_COMP_ERR_INI) return v_pedida;
  float v_bas = Carrera_v_base();
  if (!flag_linea) return (v_pedida > v_bas) ? v_bas : v_pedida;

  // No frenar rectas cuando el robot esta volviendo al centro.
  if (comp_tendencia <= 0.0f) return v_pedida;

  float e_previsto = e_abs + comp_tendencia * CARRERA_COMP_PRED_S;
  if (e_previsto <= CARRERA_COMP_ERR_INI) return v_pedida;

  float f = (e_previsto - CARRERA_COMP_ERR_INI) /
            (CARRERA_COMP_ERR_FIN - CARRERA_COMP_ERR_INI);
  if (f > 1.0f) f = 1.0f;

  float v_tope = v_bas + (Carrera_v_recta() - v_bas) * (1.0f - f);
  if (v_tope < v_bas) v_tope = v_bas;
  return (v_pedida > v_tope) ? v_tope : v_pedida;
}
#endif


//un paso de seguir la linea con la velocidad base v_base:
//sensores -> seguridad (linea perdida) -> PID de linea -> velocidad de cada llanta
//con_giro = 0: calcula el PID pero no corrige (va recto).  Devuelve 0 si tuvo que parar
static uint8_t Carrera_sigue_linea(float v_base, uint8_t con_giro)
{
  Carrera_lee_linea();

#if CARRERA_COMP_ADAPTATIVO
  // La posicion se actualiza cada 5 ms en ARRANQUE, PID, RECTA y FRENO.
  // Se filtra la variacion absoluta de error para rechazar picos de ruido.
  float e_abs = CARRERA.posicion;
  if (e_abs < 0.0f) e_abs = -e_abs;
  if (comp_tiene_muestra)
  {
    float tendencia = (e_abs - comp_e_anterior) * (1000.0f / CARRERA_TS_MS);
    if (tendencia > CARRERA_COMP_TREND_MAX) tendencia = CARRERA_COMP_TREND_MAX;
    if (tendencia < -CARRERA_COMP_TREND_MAX) tendencia = -CARRERA_COMP_TREND_MAX;
    comp_tendencia = CARRERA_COMP_TREND_ALPHA * tendencia +
                     (1.0f - CARRERA_COMP_TREND_ALPHA) * comp_tendencia;
  }
  else
  {
    comp_tiene_muestra = 1;
    comp_tendencia = 0.0f;
  }
  comp_e_anterior = e_abs;

  // El extremo +/-7.5 puede ser linea en borde O posicion de respaldo
  // durante perdida. Registrar flag_linea por separado para distinguirlos.
  if (!flag_linea)
  {
    comp_sin_linea_muestras++;
    comp_sin_linea_racha++;
    if (comp_sin_linea_racha > comp_sin_linea_racha_max)
      comp_sin_linea_racha_max = comp_sin_linea_racha;
  }
  else comp_sin_linea_racha = 0;

  uint32_t e_x100 = (uint32_t)(e_abs * 100.0f + 0.5f);
  if (e_x100 > comp_max_error_x100) comp_max_error_x100 = e_x100;

  if (CARRERA.estado == CARRERA_RECTA)
  {
    comp_recta_muestras++;
    float v_antes = v_base;
    v_base = Carrera_comp_limita_recta(v_base, e_abs);
    if (v_base < v_antes)
    {
      uint32_t recorte = (uint32_t)(v_antes - v_base + 0.5f);
      if (recorte > 0)
      {
        comp_limit_muestras++;
        comp_recorte_suma_pwm += recorte;
        if (recorte > comp_recorte_max_pwm) comp_recorte_max_pwm = recorte;
      }
    }
    CARRERA.v_base = v_base;
  }
#endif

  //SEGURIDAD: si no ve la linea CARRERA_PERDIDO_MS seguidos (se salio de la pista) -> para
  if (flag_linea) CARRERA.sin_linea = 0;
  else if (++CARRERA.sin_linea * CARRERA_TS_MS >= CARRERA_PERDIDO_MS)
  {
    Carrera_frena();
    CARRERA.aviso_perdido = 1;                         //el aviso por uart lo da Carrera_tarea
    return 0;
  }

  //PID de linea: entrada = posicion, salida = diferencia en PWM (limite V_Gir)
  CARRERA.giro = PID_Control(&PID_DIR, CARRERA.posicion);

  //IMU: amortiguador del giro. Si ya gira rapido, corrige menos (KG = cuadro KG del panel)
  CARRERA.giro -= PARAM.kg * CARRERA_SIGNO_GIRO * giro_z_filtrado;
  if (CARRERA.giro >  PARAM.vel_giro) CARRERA.giro =  PARAM.vel_giro;   //tope V_Gir
  if (CARRERA.giro < -PARAM.vel_giro) CARRERA.giro = -PARAM.vel_giro;

  float g = con_giro ? CARRERA.giro : 0.0f;

  //linea a la izquierda (posicion < 0) -> giro > 0 -> izquierda mas lenta: gira a la izquierda
  float izq = v_base - g;
  float der = v_base + g;
  if (izq >  CARRERA_PWM_MAX) izq =  CARRERA_PWM_MAX;
  if (izq < -CARRERA_PWM_MAX) izq = -CARRERA_PWM_MAX;
  if (der >  CARRERA_PWM_MAX) der =  CARRERA_PWM_MAX;
  if (der < -CARRERA_PWM_MAX) der = -CARRERA_PWM_MAX;

  //PWM equivalente -> m/s para el PID de velocidad de cada rueda
  Llanta_set_velocidad(izq * CARRERA_MPS_POR_PWM, der * CARRERA_MPS_POR_PWM);
  return 1;
}

//cada 5 ms en la interrupcion del TIM6 (TIMER_PROCEDIMIENTOS)
void Carrera_control(void)
{
  CARRERA.cuenta++;
#if CARRERA_COMP_ADAPTATIVO
  if (CARRERA.estado != CARRERA_REPOSO) comp_total_muestras++;
  if (CARRERA.estado == CARRERA_PID) comp_pid_muestras++;
  if (CARRERA.estado == CARRERA_FRENO) comp_freno_muestras++;
  if (CARRERA.estado == CARRERA_ARRANQUE) comp_arranque_muestras++;
#endif

  switch (CARRERA.estado)
  {
    case CARRERA_REPOSO:
      //despues de frenar un rato suelta los motores (para poder moverlo con la mano)
      if (CARRERA.frenando && (CARRERA.cuenta * CARRERA_TS_MS >= CARRERA_FRENO_MS))
      {
        CARRERA.frenando = 0;
        MD_Stop();
        MI_Stop();
      }
      break;

    case CARRERA_ARRANQUE:
    {
      //rampa: sube la velocidad base CARRERA_ACEL (PWM/s) hasta V_Bas
      float v_obj = Carrera_v_base();
      CARRERA.v_base += CARRERA_ACEL * (CARRERA_TS_MS / 1000.0f);
      uint8_t llego = (CARRERA.v_base >= v_obj);
      if (llego) CARRERA.v_base = v_obj;

      if (!Carrera_sigue_linea(CARRERA.v_base, CARRERA_GIRO_ARRANQUE)) break;   //se detuvo

      if (llego)                                       //termino la rampa -> PID normal
      {
        CARRERA.estado = CARRERA_PID;
        CARRERA.cuenta = 0;
        Carrera_recta_reset();                         //el tramo recto se cuenta desde aqui
      }
      break;
    }

    case CARRERA_PID:
      CARRERA.v_base = Carrera_v_curva(Carrera_v_base());   //V_Bas, mas lento si gira rapido (IMU)
      if (!Carrera_sigue_linea(CARRERA.v_base, 1)) break;   //se detuvo

      //PID -> RECTA: lleva TRAMO cm seguidos con la linea centrada (y sin girar)
      if (Carrera_mide_recta() && (CARRERA.recta_cm >= PARAM.largo_recta))
      {
        CARRERA.estado = CARRERA_RECTA;
        CARRERA.cuenta = 0;
      }
      break;

    case CARRERA_RECTA:
    {
      //apagaron las rectas desde el panel (Error_LIM, TRAMO o V_LINEA en 0) -> FRENO / PID
      if (!Carrera_rectas_on())
      {
        Carrera_sale_recta();
        Carrera_recta_reset();
        break;
      }

      //sube la velocidad base con la rampa ACEL_R hasta V_LINEA: si la recta se detecto
      //tarde, alcanza a ganar poca velocidad antes de la curva
      //(si V_LINEA es menor que la velocidad actual, baja de una)
      float v_obj = Carrera_v_recta();
      CARRERA.v_base += PARAM.acel_recta * (CARRERA_TS_MS / 1000.0f);
      if (CARRERA.v_base > v_obj) CARRERA.v_base = v_obj;

      if (!Carrera_sigue_linea(CARRERA.v_base, 1)) break;   //se detuvo

      //se acabo la recta: la linea salio de Error_LIM o gira mas que GIRO_LIM (viene curva)
      //-> FRENO (o PID si ya va lento). El tramo vuelve a 0
      if (!Carrera_mide_recta()) Carrera_sale_recta();
      break;
    }

    case CARRERA_FRENO:
    {
      //velocidad base V_FRENO: el PID de velocidad de las llantas frena fuerte,
      //pero el PID de linea sigue corrigiendo (la curva ya empezo)
      CARRERA.v_base = PARAM.velo_freno;               //PWM (Param_aplica ya revisa el rango)
      if (!Carrera_sigue_linea(CARRERA.v_base, 1)) break;   //se detuvo

      //FRENO -> PID: ya bajo a V_Bas, o paso el tiempo maximo T_FRENO
      float v_bas = Carrera_v_base() * CARRERA_MPS_POR_PWM;  //m/s
      if ((ROBOT.velocidad <= v_bas) ||
          ((float)(CARRERA.cuenta * CARRERA_TS_MS) >= PARAM.freno_ms))
      {
        CARRERA.estado = CARRERA_PID;
        CARRERA.cuenta = 0;
        Carrera_recta_reset();
      }
      break;
    }

    default:   //estados pendientes: por seguridad vuelve a PID
      CARRERA.estado = CARRERA_PID;
      break;
  }
}

//en el while: telemetria de carrera para el panel
void Carrera_tarea(void)
{
  static uint32_t t_tele = 0, t_est = 0;
  uint32_t ahora = HAL_GetTick();

#if CARRERA_COMP_ADAPTATIVO
  // Informe solo despues de STOP/perdida; nunca transmite desde interrupciones.
  // En la trama del panel aparece como texto: COMP_DIAG ...
  if (comp_reporte_pendiente && CARRERA.estado == CARRERA_REPOSO)
  {
    comp_reporte_pendiente = 0;
    char resumen[175];
    uint32_t promedio = comp_limit_muestras ?
                        (comp_recorte_suma_pwm / comp_limit_muestras) : 0;
    snprintf(resumen, sizeof(resumen),
             "COMP_DIAG total_ms=%lu recta_ms=%lu limite_ms=%lu "
             "recorte_med_pwm=%lu recorte_max_pwm=%lu error_max_x100=%lu\r\n",
             (unsigned long)(comp_total_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_recta_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_limit_muestras * CARRERA_TS_MS),
             (unsigned long)promedio,
             (unsigned long)comp_recorte_max_pwm,
             (unsigned long)comp_max_error_x100);
    uartx_write_text(&TELE_UART, resumen);

    // Segunda trama de diagnostico, sin modificar los comandos del panel.
    snprintf(resumen, sizeof(resumen),
             "COMP_EST arranque_ms=%lu pid_ms=%lu freno_ms=%lu "
             "sin_linea_ms=%lu perdida_max_ms=%lu\r\n",
             (unsigned long)(comp_arranque_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_pid_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_freno_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_sin_linea_muestras * CARRERA_TS_MS),
             (unsigned long)(comp_sin_linea_racha_max * CARRERA_TS_MS));
    uartx_write_text(&TELE_UART, resumen);
  }
#endif

  if (CARRERA.aviso_perdido)
  {
    CARRERA.aviso_perdido = 0;
    uartx_write_text(&TELE_UART, "CARRERA: LINEA PERDIDA -> REPOSO (seguridad)\r\n");
  }

  //estado: siempre, cada 500 ms (el panel pinta el boton RUN/STOP y el indicador)
  if (ahora - t_est >= 500)
  {
    t_est = ahora;
    sprintf(tx, "*EST%d*", (int)CARRERA.estado);
    uartx_write_text(&TELE_UART, tx);
  }

  //en carrera: posicion, partes del PID de linea, giro y velocidades.
  //cada 4 envios (200 ms) tambien los 16 sensores (LEDs del panel)
  static uint8_t n_tele = 0;
  if ((CARRERA.estado != CARRERA_REPOSO) && (ahora - t_tele >= CARRERA_TELE_MS))
  {
    t_tele = ahora;
    n_tele++;
    manda_operando((n_tele & 3) == 0);
  }
}
