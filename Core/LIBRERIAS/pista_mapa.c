#include "pista_mapa.h"
#include "telemetria.h"
#include "uart.h"
#include <stdio.h>

// Se compacta cada muestra: 2+2+2+2+1+1 = 10 bytes, sin malloc.
// Solo RAM: el mapa desaparece al cortar la alimentacion.
typedef struct {
  uint16_t distancia_cm;
  int16_t giro_dps_x10;
  int16_t velocidad_mmps;
  int16_t posicion_x100;
  uint8_t linea_valida;
  uint8_t reservado;
} PISTA_PUNTO;

static PISTA_PUNTO pista[PISTA_MAX_PUNTOS];
static volatile uint8_t estado_pista = PISTA_VACIA;
static volatile uint16_t n_puntos = 0U;
static float origen_m = 0.0f;
static float proxima_m = 0.0f;
static volatile uint32_t ciclos_sin_linea = 0U;
static volatile uint32_t giro_max_x10 = 0U;
static volatile uint32_t distancia_final_cm = 0U;
static uint8_t aviso_interrupcion = 0U;

static int16_t compacta_firmado(float x)
{
  if (x > 32767.0f) x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)x;
}

void Pista_mapa_init(void)
{
  estado_pista = PISTA_VACIA;
  n_puntos = 0U;
  origen_m = 0.0f;
  proxima_m = 0.0f;
  ciclos_sin_linea = 0U;
  giro_max_x10 = 0U;
  distancia_final_cm = 0U;
  aviso_interrupcion = 0U;
}

uint8_t Pista_mapa_iniciar(float distancia_m)
{
  if (estado_pista == PISTA_APRENDIENDO) return 0U;
  // La tarea llama esta funcion solo con el robot detenido.
  estado_pista = PISTA_VACIA;
  n_puntos = 0U;
  origen_m = distancia_m;
  proxima_m = 0.0f;
  ciclos_sin_linea = 0U;
  giro_max_x10 = 0U;
  distancia_final_cm = 0U;
  aviso_interrupcion = 0U;
  estado_pista = PISTA_APRENDIENDO;
  return 1U;
}

// Llamada cada 5ms desde Carrera_sigue_linea(), despues de leer sensores.
// Nunca hace IO ni reserva memoria dentro de la interrupcion.
void Pista_mapa_registrar(float distancia_m, float velocidad_mps,
                         float giro_dps, float posicion, uint8_t hay_linea)
{
  if (estado_pista != PISTA_APRENDIENDO) return;

  if (!hay_linea) ciclos_sin_linea++;
  float giro_abs = (giro_dps < 0.0f) ? -giro_dps : giro_dps;
  uint32_t w10 = (uint32_t)(giro_abs * 10.0f + 0.5f);
  if (w10 > giro_max_x10) giro_max_x10 = w10;

  float avance_m = distancia_m - origen_m;
  if (avance_m < 0.0f) return; // no aprende movimientos atras de la salida
  if (avance_m < proxima_m) return;

  if (n_puntos >= PISTA_MAX_PUNTOS)
  {
    estado_pista = PISTA_LLENA;
    return; // falla explicita; nunca sobrescribe los puntos previos
  }

  PISTA_PUNTO *p = &pista[n_puntos];
  float s_cm = avance_m * 100.0f;
  if (s_cm > 65535.0f) { estado_pista = PISTA_LLENA; return; }
  p->distancia_cm = (uint16_t)(s_cm + 0.5f);
  p->giro_dps_x10 = compacta_firmado(giro_dps * 10.0f);
  p->velocidad_mmps = compacta_firmado(velocidad_mps * 1000.0f);
  p->posicion_x100 = compacta_firmado(posicion * 100.0f);
  p->linea_valida = hay_linea ? 1U : 0U;
  p->reservado = 0U;

  n_puntos++;
  proxima_m = avance_m + PISTA_PASO_M;
}

uint8_t Pista_mapa_finalizar(float distancia_m)
{
  if (estado_pista != PISTA_APRENDIENDO) return 0U;
  // Asigna estado primero para impedir escritura concurrente desde TIM6.
  estado_pista = PISTA_LISTA;
  float recorrido_m = distancia_m - origen_m;
  if (recorrido_m < 0.0f) recorrido_m = 0.0f;
  distancia_final_cm = (uint32_t)(recorrido_m * 100.0f + 0.5f);
  if (n_puntos < 50U || distancia_final_cm < 100U)
  {
    estado_pista = PISTA_INTERRUPTA;
    return 0U;
  }
  return 1U;
}

void Pista_mapa_cancelar(void)
{
  if (estado_pista == PISTA_APRENDIENDO)
  {
    estado_pista = PISTA_INTERRUPTA;
    aviso_interrupcion = 1U;
  }
}

uint8_t Pista_mapa_estado(void) { return estado_pista; }

void Pista_mapa_estado_uart(void)
{
  char linea[160];
  snprintf(linea, sizeof(linea),
           "MAPA estado=%u puntos=%u longitud_cm=%lu sin_linea_ms=%lu giro_max_x10=%lu\r\n",
           (unsigned)estado_pista,
           (unsigned)n_puntos,
           (unsigned long)distancia_final_cm,
           (unsigned long)(ciclos_sin_linea * 5U),
           (unsigned long)giro_max_x10);
  uartx_write_text(&TELE_UART, linea);
}

void Pista_mapa_vigilar(uint8_t robot_en_reposo)
{
  if (robot_en_reposo && estado_pista == PISTA_APRENDIENDO)
    Pista_mapa_cancelar(); // IR STOP o parada por linea perdida

  if (aviso_interrupcion)
  {
    aviso_interrupcion = 0U;
    uartx_write_text(&TELE_UART,
                     "MAPA: aprendizaje interrumpido, no usar como mapa valido\r\n");
  }
}
