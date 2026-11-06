#include "pista_mapa.h"
#include "telemetria.h"
#include "uart.h"
#include "flash_interna.h"
#include <stdio.h>
#include <stddef.h>
#include <string.h>

// Se compacta cada muestra: 2+2+2+2+1+1 = 10 bytes, sin malloc.
// V5.5: copia persistente con CRC en FLASH_MAPA_DIR; PARAMETROS en otro sector.
typedef struct {
  uint16_t distancia_cm;
  int16_t giro_dps_x10;
  int16_t velocidad_mmps;
  int16_t posicion_x100;
  uint8_t linea_valida;
  uint8_t reservado;
} PISTA_PUNTO;

// Cabecera de version fija, con CRC de la cabecera y CRC independiente de los puntos.
// Los dos CRC impiden usar un mapa parcial o corrupto tras un corte de energia.
typedef struct {
  uint32_t marca;
  uint32_t version;
  uint32_t cantidad;
  uint32_t longitud_cm;
  uint32_t sin_linea_ciclos;
  uint32_t giro_max_x10;
  uint32_t cantidad_validos;
  uint32_t crc_puntos;
  uint32_t crc_cabecera;
} PISTA_CABECERA_FLASH;

typedef struct {
  PISTA_CABECERA_FLASH cabecera;
  PISTA_PUNTO puntos[PISTA_MAX_PUNTOS];
} PISTA_IMAGEN;

#define PISTA_FLASH_MARCA   0x504D4244u  // DBMP
#define PISTA_FLASH_VERSION 1u

_Static_assert(sizeof(PISTA_PUNTO) == 10u, "El punto debe tener 10 bytes");
_Static_assert(sizeof(PISTA_CABECERA_FLASH) == 36u, "Cabecera Flash de 36 bytes");
_Static_assert(sizeof(PISTA_IMAGEN) <= FLASH_MAPA_TAM, "Mapa excede la zona reservada");

// Una sola imagen en RAM: contiene los puntos y una cabecera para grabar.
// Se evita duplicar una matriz de 40 KB en otra copia temporal.
static PISTA_IMAGEN imagen;
#define pista imagen.puntos
static volatile uint8_t estado_pista = PISTA_VACIA;
static volatile uint16_t n_puntos = 0U;
static float origen_m = 0.0f;
static float proxima_m = 0.0f;
static volatile uint32_t ciclos_sin_linea = 0U;
static volatile uint16_t puntos_linea_valida = 0U;
static volatile uint32_t giro_max_x10 = 0U;
static volatile uint32_t distancia_final_cm = 0U;
static uint8_t aviso_interrupcion = 0U;

// Reproduccion del mapa: nunca toca los motores, solo la consigna base en PID.
// El mapa es reutilizable hasta reiniciar, aprender otra pista o apagar el robot.
static volatile uint8_t replay_activo = 0U;
static volatile uint8_t replay_aviso = 0U;
static volatile uint8_t replay_perfil = 0U;
static float replay_origen_m = 0.0f;
static float replay_bonus = 0.0f;
static uint16_t replay_indice = 0U;
static volatile uint32_t replay_boost_ciclos = 0U;
static volatile uint32_t replay_bonus_max = 0U;
static volatile uint32_t replay_prev_ciclos = 0U;
static volatile uint32_t replay_veto_ciclos = 0U;

static int16_t compacta_firmado(float x)
{
  if (x > 32767.0f) x = 32767.0f;
  if (x < -32768.0f) x = -32768.0f;
  return (int16_t)x;
}

// La memoria Flash esta mapeada: leerla no borra ni programa sectores.
// Validacion obligatoria ANTES de copiar cualquier contenido a la RAM.
uint8_t Pista_mapa_flash_valido(void)
{
  if (!Flash_zona_libre()) return 0U;

  const PISTA_IMAGEN *f = (const PISTA_IMAGEN *)(uintptr_t)FLASH_MAPA_DIR;
  const PISTA_CABECERA_FLASH *h = &f->cabecera;

  if (h->marca != PISTA_FLASH_MARCA || h->version != PISTA_FLASH_VERSION)
    return 0U;
  if (h->cantidad < 50U || h->cantidad > PISTA_MAX_PUNTOS ||
      h->longitud_cm < 100U || h->cantidad_validos > h->cantidad)
    return 0U;

  const uint32_t bytes = h->cantidad * (uint32_t)sizeof(PISTA_PUNTO);
  if ((sizeof(PISTA_CABECERA_FLASH) + bytes) > FLASH_MAPA_TAM)
    return 0U;
  if (Flash_crc32(h, offsetof(PISTA_CABECERA_FLASH, crc_cabecera)) != h->crc_cabecera)
    return 0U;
  if (Flash_crc32(f->puntos, bytes) != h->crc_puntos)
    return 0U;

  // Evita reproduccion de registros desordenados por corrupcion.
  uint16_t anterior = 0U;
  for (uint32_t i = 0U; i < h->cantidad; i++)
  {
    if (i && f->puntos[i].distancia_cm <= anterior) return 0U;
    if (f->puntos[i].linea_valida > 1U || f->puntos[i].reservado > 2U)
      return 0U;
    anterior = f->puntos[i].distancia_cm;
  }
  return 1U;
}

// Solo llamar con el robot detenido despues de MAPEND.
// Escribe exclusivamente dentro de los 128 KiB del mapa, nunca en PARAM_DIR.
uint8_t Pista_mapa_guardar_flash(void)
{
  if (estado_pista != PISTA_LISTA || replay_activo || n_puntos < 50U ||
      n_puntos > PISTA_MAX_PUNTOS || distancia_final_cm < 100U)
    return 0U;

  PISTA_CABECERA_FLASH *h = &imagen.cabecera;
  h->marca = PISTA_FLASH_MARCA;
  h->version = PISTA_FLASH_VERSION;
  h->cantidad = n_puntos;
  h->longitud_cm = distancia_final_cm;
  h->sin_linea_ciclos = ciclos_sin_linea;
  h->giro_max_x10 = giro_max_x10;
  h->cantidad_validos = puntos_linea_valida;
  const uint32_t bytes_puntos = (uint32_t)n_puntos * sizeof(PISTA_PUNTO);
  h->crc_puntos = Flash_crc32(pista, bytes_puntos);
  h->crc_cabecera = Flash_crc32(h, offsetof(PISTA_CABECERA_FLASH, crc_cabecera));

  const uint32_t bytes_total = sizeof(PISTA_CABECERA_FLASH) + bytes_puntos;
  if (!Flash_guardar(FLASH_MAPA_DIR, FLASH_MAPA_TAM, &imagen, bytes_total))
    return 0U;

  return Pista_mapa_flash_valido();
}

// Al arrancar el micro: copia la ultima vuelta valida Flash -> RAM.
// No utiliza ningun sector de parametros. No mueve los motores.
uint8_t Pista_mapa_leer_flash(void)
{
  if (estado_pista == PISTA_APRENDIENDO || replay_activo)
    return 0U;
  if (!Pista_mapa_flash_valido())
    return 0U;

  const PISTA_IMAGEN *f = (const PISTA_IMAGEN *)(uintptr_t)FLASH_MAPA_DIR;
  const uint32_t bytes = sizeof(PISTA_CABECERA_FLASH) +
                         f->cabecera.cantidad * sizeof(PISTA_PUNTO);
  estado_pista = PISTA_VACIA;
  memcpy(&imagen, f, bytes);

  n_puntos = (uint16_t)imagen.cabecera.cantidad;
  distancia_final_cm = imagen.cabecera.longitud_cm;
  ciclos_sin_linea = imagen.cabecera.sin_linea_ciclos;
  giro_max_x10 = imagen.cabecera.giro_max_x10;
  puntos_linea_valida = (uint16_t)imagen.cabecera.cantidad_validos;
  replay_activo = 0U;
  replay_perfil = 0U;
  replay_bonus = 0.0f;
  replay_indice = 0U;
  origen_m = 0.0f;
  proxima_m = 0.0f;
  estado_pista = PISTA_LISTA;
  return 1U;
}

void Pista_mapa_init(void)
{
  estado_pista = PISTA_VACIA;
  n_puntos = 0U;
  origen_m = 0.0f;
  proxima_m = 0.0f;
  ciclos_sin_linea = 0U;
  puntos_linea_valida = 0U;
  giro_max_x10 = 0U;
  distancia_final_cm = 0U;
  aviso_interrupcion = 0U;
  replay_activo = 0U;
  replay_aviso = 0U;
  replay_perfil = 0U;
  replay_origen_m = 0.0f;
  replay_bonus = 0.0f;
  replay_indice = 0U;
  replay_boost_ciclos = 0U;
  replay_bonus_max = 0U;
  replay_prev_ciclos = 0U;
  replay_veto_ciclos = 0U;
}

uint8_t Pista_mapa_iniciar(float distancia_m)
{
  if (estado_pista == PISTA_APRENDIENDO) return 0U;
  // La tarea llama esta funcion solo con el robot detenido.
  estado_pista = PISTA_VACIA;
  replay_activo = 0U;
  replay_bonus = 0.0f;
  n_puntos = 0U;
  origen_m = distancia_m;
  proxima_m = 0.0f;
  ciclos_sin_linea = 0U;
  puntos_linea_valida = 0U;
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
  // Clase del punto para mirar hacia adelante durante la vuelta rapida:
  // 0 recta/suave; 1 curva real; 2 lectura sin linea (no inventar curva).
  float e_abs = (posicion < 0.0f) ? -posicion : posicion;
  p->reservado = !hay_linea ? 2U :
                ((w10 >= PISTA_GIRO_CURVA_X10 ||
                  e_abs * 100.0f >= PISTA_ERR_CURVA_X100) ? 1U : 0U);

  n_puntos++;
  if (hay_linea) puntos_linea_valida++;
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
  if (replay_activo)
  {
    replay_activo = 0U;
    replay_aviso = 1U;
    replay_bonus = 0.0f;
  }
  if (estado_pista == PISTA_APRENDIENDO)
  {
    estado_pista = PISTA_INTERRUPTA;
    aviso_interrupcion = 1U;
  }
}

uint8_t Pista_mapa_replay_iniciar(float distancia_m, uint8_t perfil)
{
  if (estado_pista != PISTA_LISTA || n_puntos < 50U || distancia_final_cm < 100U)
    return 0U;
  if (perfil != PISTA_PERFIL_SEGURO && perfil != PISTA_PERFIL_RAPIDO)
    return 0U;
  if (((uint32_t)puntos_linea_valida * 100U / n_puntos) < PISTA_COBERTURA_MIN_PCT)
    return 0U;

  replay_activo = 0U; // configuracion atomica antes de reactivar
  replay_perfil = perfil;
  replay_origen_m = distancia_m;
  replay_bonus = 0.0f;
  replay_indice = 0U;
  replay_boost_ciclos = 0U;
  replay_bonus_max = 0U;
  replay_prev_ciclos = 0U;
  replay_veto_ciclos = 0U;
  replay_aviso = 0U;
  replay_activo = 1U;
  return 1U;
}

// Se ejecuta en TIM6 a 5 ms. Nunca transmite, ni bloquea, ni reserva RAM.
// En RECTA/FRENO/ARRANQUE o sin linea se conserva exactamente la consigna V3.
float Pista_mapa_replay_velocidad(float distancia_m, float v_base,
                                 float posicion, float giro_dps,
                                 uint8_t hay_linea, uint8_t en_pid)
{
  if (!replay_activo) return v_base;

  float recorrido_m = distancia_m - replay_origen_m;
  if (recorrido_m < -0.03f ||
      (recorrido_m * 100.0f > (float)distancia_final_cm + 10.0f))
  {
    replay_activo = 0U; // encoder retrocedio o termina el mapa: vuelve a V3
    replay_bonus = 0.0f;
    replay_aviso = 1U;
    return v_base;
  }

  if (!en_pid)
  {
    replay_bonus = 0.0f;
    return v_base;
  }

  float e_abs = (posicion < 0.0f) ? -posicion : posicion;
  float w_abs = (giro_dps < 0.0f) ? -giro_dps : giro_dps;
  if (!hay_linea || e_abs > PISTA_ERR_REAL_MAX ||
      w_abs > PISTA_GIRO_REAL_MAX)
  {
    replay_bonus = 0.0f;
    replay_veto_ciclos++;
    return v_base; // no acelerar mientras corrige / no detecta la linea
  }

  // Un pequeno retroceso/ruido del encoder no debe provocar unsigned wrap.
  if (recorrido_m < 0.0f) recorrido_m = 0.0f;
  uint32_t cm = (uint32_t)(recorrido_m * 100.0f + 0.5f);
  while ((uint32_t)replay_indice + 1U < n_puntos &&
         pista[replay_indice+1U].distancia_cm <= cm)
    replay_indice++;

  uint32_t vista_cm = (replay_perfil == PISTA_PERFIL_RAPIDO)
                    ? PISTA_LOOK_RAPIDO_CM : PISTA_LOOK_SEGURO_CM;
  float bonus_max = (replay_perfil == PISTA_PERFIL_RAPIDO)
                  ? PISTA_BONUS_RAPIDO_PWM : PISTA_BONUS_SEGURO_PWM;

  // Busca la siguiente curva; 3 muestras consecutivas sin linea se toman
  // como region incierta y obligan a retirar la bonificacion por precaucion.
  uint32_t distancia_curva_cm = vista_cm + 1U;
  uint8_t sin_linea_seguidos = 0U;
  for (uint16_t i = replay_indice; i < n_puntos; i++)
  {
    if ((uint32_t)pista[i].distancia_cm > cm + vista_cm) break;
    if (pista[i].reservado == 2U) sin_linea_seguidos++;
    else sin_linea_seguidos = 0U;
    if (pista[i].reservado == 1U || sin_linea_seguidos >= 3U)
    {
      uint32_t d = ((uint32_t)pista[i].distancia_cm > cm)
                 ? (uint32_t)pista[i].distancia_cm - cm : 0U;
      distancia_curva_cm = d;
      break;
    }
  }

  float objetivo = bonus_max;
  if (distancia_curva_cm <= vista_cm)
  {
    objetivo = bonus_max * ((float)distancia_curva_cm / (float)vista_cm);
    replay_prev_ciclos++;
  }

  const float dt = 0.005f;
  if (replay_bonus < objetivo)
  {
    replay_bonus += PISTA_ACEL_PWM_S * dt;
    if (replay_bonus > objetivo) replay_bonus = objetivo;
  }
  else if (replay_bonus > objetivo)
  {
    replay_bonus -= PISTA_DESACEL_PWM_S * dt;
    if (replay_bonus < objetivo) replay_bonus = objetivo;
  }

  if (replay_bonus > 0.0f)
  {
    replay_boost_ciclos++;
    uint32_t b = (uint32_t)(replay_bonus + 0.5f);
    if (b > replay_bonus_max) replay_bonus_max = b;
  }

  float v = v_base + replay_bonus;
  return (v > 1000.0f) ? 1000.0f : v;
}

uint8_t Pista_mapa_estado(void) { return estado_pista; }

void Pista_mapa_estado_uart(void)
{
  char linea[220];
  snprintf(linea, sizeof(linea),
           "MAPA estado=%u puntos=%u longitud_cm=%lu sin_linea_ms=%lu giro_max_x10=%lu validos_pct=%lu replay=%u perfil=%u flash=%u\r\n",
           (unsigned)estado_pista,
           (unsigned)n_puntos,
           (unsigned long)distancia_final_cm,
           (unsigned long)(ciclos_sin_linea * 5U),
           (unsigned long)giro_max_x10,
           (unsigned long)(n_puntos ? ((uint32_t)puntos_linea_valida * 100U / n_puntos) : 0U),
           (unsigned)replay_activo, (unsigned)replay_perfil,
           (unsigned)Pista_mapa_flash_valido());
  uartx_write_text(&TELE_UART, linea);
}

void Pista_mapa_vigilar(uint8_t robot_en_reposo)
{
  if (robot_en_reposo &&
      (estado_pista == PISTA_APRENDIENDO || replay_activo))
    Pista_mapa_cancelar(); // IR STOP o parada por linea perdida

  if (replay_aviso)
  {
    replay_aviso = 0U;
    char linea_r[160];
    snprintf(linea_r, sizeof(linea_r),
             "MAP_RACE perfil=%u boost_ms=%lu bonus_max_pwm=%lu previa_ms=%lu veto_ms=%lu\r\n",
             (unsigned)replay_perfil,
             (unsigned long)(replay_boost_ciclos * 5U),
             (unsigned long)replay_bonus_max,
             (unsigned long)(replay_prev_ciclos * 5U),
             (unsigned long)(replay_veto_ciclos * 5U));
    uartx_write_text(&TELE_UART, linea_r);
  }

  if (aviso_interrupcion)
  {
    aviso_interrupcion = 0U;
    uartx_write_text(&TELE_UART,
                     "MAPA: aprendizaje interrumpido, no usar como mapa valido\r\n");
  }
}
