#ifndef LIBRERIAS_PISTA_MAPA_H_
#define LIBRERIAS_PISTA_MAPA_H_

#include "main.h"
#include <stdint.h>

// V5: aprendizaje de cualquier circuito cerrado con linea visible.
// Solo registra el recorrido; NO modifica el controlador de motores.
// 2048 x 10 bytes = 20 KiB de RAM, una muestra por aprox. 2 cm.
#define PISTA_MAX_PUNTOS 2048U
#define PISTA_PASO_M 0.02f

enum {
  PISTA_VACIA = 0,
  PISTA_APRENDIENDO = 1,
  PISTA_LISTA = 2,
  PISTA_LLENA = 3,
  PISTA_INTERRUPTA = 4
};

void Pista_mapa_init(void);
uint8_t Pista_mapa_iniciar(float distancia_m);
uint8_t Pista_mapa_finalizar(float distancia_m);
void Pista_mapa_cancelar(void);
void Pista_mapa_registrar(float distancia_m, float velocidad_mps,
                         float giro_dps, float posicion, uint8_t hay_linea);
void Pista_mapa_estado_uart(void);
void Pista_mapa_vigilar(uint8_t robot_en_reposo);
uint8_t Pista_mapa_estado(void);

#endif /* LIBRERIAS_PISTA_MAPA_H_ */
