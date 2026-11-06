#ifndef LIBRERIAS_PISTA_MAPA_H_
#define LIBRERIAS_PISTA_MAPA_H_

#include "main.h"
#include <stdint.h>

// V5: aprendizaje de cualquier circuito cerrado con linea visible.
// Registra y reproduce la pista en RAM y conserva una copia en Flash interna.
// V5.5: se guarda solo al finalizar el aprendizaje y con los motores detenidos.
// 4096 x 10 bytes = 40 KiB, una muestra por aprox. 2 cm.
#define PISTA_MAX_PUNTOS 4096U
#define PISTA_PASO_M 0.02f

// Perfiles seleccionables SIN volver a programar la placa.
// La velocidad adicional solamente se aplica en PID y con linea visible.
#define PISTA_PERFIL_SEGURO 1U
#define PISTA_PERFIL_RAPIDO 2U
#define PISTA_BONUS_SEGURO_PWM 30.0f
#define PISTA_BONUS_RAPIDO_PWM 55.0f
#define PISTA_LOOK_SEGURO_CM 35U
#define PISTA_LOOK_RAPIDO_CM 45U
#define PISTA_GIRO_CURVA_X10 550U  // |omega| >= 55 grados/s: curva
#define PISTA_ERR_CURVA_X100 300U  // |posicion| >= 3.0: curva
#define PISTA_GIRO_REAL_MAX 75.0f
#define PISTA_ERR_REAL_MAX 2.5f
#define PISTA_ACEL_PWM_S 700.0f
#define PISTA_DESACEL_PWM_S 2400.0f
#define PISTA_COBERTURA_MIN_PCT 60U

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
// No ejecutarlas durante carrera ni desde interrupciones.
uint8_t Pista_mapa_guardar_flash(void);  // 1 = guardado y CRC verificado
uint8_t Pista_mapa_leer_flash(void);    // 1 = mapa recuperado tras reiniciar
uint8_t Pista_mapa_flash_valido(void);  // 1 = cabecera y datos validos
uint8_t Pista_mapa_replay_iniciar(float distancia_m, uint8_t perfil);
float Pista_mapa_replay_velocidad(float distancia_m, float v_base,
                                 float posicion, float giro_dps,
                                 uint8_t hay_linea, uint8_t en_pid);
void Pista_mapa_registrar(float distancia_m, float velocidad_mps,
                         float giro_dps, float posicion, uint8_t hay_linea);
void Pista_mapa_estado_uart(void);
void Pista_mapa_vigilar(uint8_t robot_en_reposo);
uint8_t Pista_mapa_estado(void);

#endif /* LIBRERIAS_PISTA_MAPA_H_ */
