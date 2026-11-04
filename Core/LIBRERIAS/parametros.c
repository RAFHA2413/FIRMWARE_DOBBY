/*
 * parametros.c
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "parametros.h"
#include "flash_interna.h"
#include "PID.h"
#include "IR_ANA_SEN.h"
#include "carrera.h"        //CARRERA_MPS_POR_PWM y CARRERA_PWM_MAX (rango de V_LINEA / V_FRENO)
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>

#define PARAM_MARCA   0xD0BB2026u

PARAMETROS PARAM;


//valores por defecto (cuando la flash esta vacia o los datos no son validos)
void Param_defecto(void)
{
  memset(&PARAM, 0, sizeof(PARAM));

  //PID de direccion: los mismos valores de inicio de PID.h
  PARAM.kp          = PID_DIR_KP;
  PARAM.ki          = PID_DIR_KI;
  PARAM.kd          = PID_DIR_KD;
  PARAM.tau         = PID_DIR_TAU / PID_DIR_TS;   //se guarda en multiplos de Ts (ej. 5)
  PARAM.kaw         = (PID_DIR_KI > 0.0f) ? (PID_DIR_KAW * PID_DIR_KP / PID_DIR_KI) : 0.3f; //factor de Ki/Kp (ej. 0.3)
  PARAM.vel_giro    = PID_DIR_MAX;

  PARAM.velo_basica = 0.0f;     //PWM 0..1000. 0 = queda en su sitio y solo corrige (primera prueba)
  PARAM.kg          = 0.0f;

  PARAM.error_recta = 0.0f;
  PARAM.velo_recta  = PARAM_V_LINEA_DEF;   //m/s (las rectas siguen apagadas: error y largo en 0)
  PARAM.largo_recta = 0.0f;
  PARAM.giro_recta  = 0.0f;     //0 = la recta se decide solo con el error de la linea

  PARAM.velo_freno  = PARAM_V_FRENO_DEF;   //m/s
  PARAM.freno_ms    = PARAM_T_FRENO_DEF;   //ms

  PARAM.linea_color = negro_;   //IR_ANA_SEN.h: blanco_ 0, negro_ 1
}

//pasa PARAM al PID de linea (PID_DIR) y al color de linea de los sensores.
//llamar despues de Param_init y cada vez que se cambie un valor de PARAM
void Param_aplica(void)
{
  if (PID_DIR.Ts <= 0.0f) PID_DIR_INIT();   //el PID necesita su Ts antes de sintonizar

  if (PARAM.kp < 0.0f) PARAM.kp = 0.0f;
  if (PARAM.ki < 0.0f) PARAM.ki = 0.0f;
  if (PARAM.kd < 0.0f) PARAM.kd = 0.0f;
  if (PARAM.tau < 1.0f) PARAM.tau = 1.0f;          //minimo Ts
  if (PARAM.vel_giro < 0.0f) PARAM.vel_giro = -PARAM.vel_giro;

  //V_LINEA y V_FRENO en m/s (0 .. PWM 1000). Un valor mas alto es uno viejo en PWM
  //(ej. 600 de la SD): en vez de salir a fondo se pone el valor por defecto
  const float v_max = CARRERA_PWM_MAX * CARRERA_MPS_POR_PWM;   //2.6 m/s
  if ((PARAM.velo_recta < 0.0f) || (PARAM.velo_recta > v_max)) PARAM.velo_recta = PARAM_V_LINEA_DEF;
  if ((PARAM.velo_freno < 0.0f) || (PARAM.velo_freno > v_max)) PARAM.velo_freno = PARAM_V_FRENO_DEF;
  if (PARAM.freno_ms < 0.0f) PARAM.freno_ms = 0.0f;

  //tau en multiplos de Ts, kaw como factor de Ki/Kp (igual que el panel)
  float kaw = (PARAM.kp > 0.0f) ? PARAM.kaw * PARAM.ki / PARAM.kp : 0.0f;
  PID_Sintoniza(&PID_DIR, PARAM.kp, PARAM.ki, PARAM.kd, PARAM.tau * PID_DIR.Ts, kaw);
  PID_DIR.out_max =  PARAM.vel_giro;
  PID_DIR.out_min = -PARAM.vel_giro;

  linea_color = (PARAM.linea_color != 0) ? negro_ : blanco_;
}

//crc de toda la struct menos el campo crc
static uint32_t Param_calcula_crc(const PARAMETROS *p)
{
  return Flash_crc32(p, offsetof(PARAMETROS, crc));
}

//1 = los datos son validos
static uint8_t Param_valido(const PARAMETROS *p)
{
  return (p->marca == PARAM_MARCA) &&
         (p->version == PARAM_VERSION) &&
         (p->crc == Param_calcula_crc(p));
}


//1 = datos validos de una version vieja: su crc estaba en el byte off_crc
static uint8_t Param_vieja_valida(const PARAMETROS *p, uint32_t version, uint32_t off_crc)
{
  uint32_t crc;
  if ((p->marca != PARAM_MARCA) || (p->version != version)) return 0;
  memcpy(&crc, (const uint8_t *)p + off_crc, sizeof(crc));
  return crc == Flash_crc32(p, off_crc);
}


uint8_t Param_init(void)
{
  PARAMETROS leido;
  memcpy(&leido, (const void *)FLASH_PARAM_DIR, sizeof(leido));

  if (Param_valido(&leido))
  {
    PARAM = leido;
    return 1;
  }

  //versiones viejas: la misma struct pero mas corta (los campos nuevos van al final).
  //Su crc esta donde hoy empieza el primer campo que no tenian
  if (Param_vieja_valida(&leido, 3u, offsetof(PARAMETROS, giro_recta)))
    leido.giro_recta = 0.0f;                           //no tenia GIRO_LIM: no usa el IMU
  else if (!Param_vieja_valida(&leido, 4u, offsetof(PARAMETROS, velo_freno)))
  {
    Param_defecto();                                   //nada valido
    return 0;
  }

  //3 y 4: V_LINEA estaba en PWM y no habia freno -> valores por defecto
  PARAM = leido;
  PARAM.velo_recta = PARAM_V_LINEA_DEF;
  PARAM.velo_freno = PARAM_V_FRENO_DEF;
  PARAM.freno_ms   = PARAM_T_FRENO_DEF;
  PARAM.version    = PARAM_VERSION;                    //al GUARDAR queda en la version nueva
  return 1;
}


//graba PARAM en la flash. Llamar SOLO con el robot detenido.
uint8_t Param_guardar(void)
{
  PARAM.marca   = PARAM_MARCA;
  PARAM.version = PARAM_VERSION;
  PARAM.crc     = Param_calcula_crc(&PARAM);

  //borra el sector de parametros, graba y verifica (1 = ok, 0 = error)
  return Flash_guardar(FLASH_PARAM_DIR, FLASH_PARAM_TAM, &PARAM, sizeof(PARAM));
}


//lee una trama "DAT/kp/ki/kd/Vb/Vg/tau/kaw/Kg/errrect/velrect/largorect/color/girorect/vfreno/tfreno/"
//y cambia los valores en RAM (no graba en flash).
//girorect, vfreno y tfreno son opcionales (tramas viejas de la SD)
uint8_t Param_desde_trama(const char *trama)
{
  char copia[200];
  float v[15];
  uint8_t n = 0;

  const char *ini = strstr(trama, "DAT/");
  if (ini == NULL) return 0;

  strncpy(copia, ini + 4, sizeof(copia) - 1);
  copia[sizeof(copia) - 1] = 0;

  char *tok = strtok(copia, "/\r\n");
  while ((tok != NULL) && (n < 15))
  {
    v[n++] = strtof(tok, NULL);
    tok = strtok(NULL, "/\r\n");
  }

  if (n < 12) return 0;   //trama incompleta: no cambia nada

  PARAM.kp          = v[0];
  PARAM.ki          = v[1];
  PARAM.kd          = v[2];
  PARAM.velo_basica = v[3];
  PARAM.vel_giro    = v[4];
  PARAM.tau         = v[5];
  PARAM.kaw         = v[6];
  PARAM.kg          = v[7];
  PARAM.error_recta = v[8];
  PARAM.velo_recta  = v[9];
  PARAM.largo_recta = v[10];
  PARAM.linea_color = (uint32_t)v[11];
  if (n >= 13) PARAM.giro_recta = v[12];
  if (n >= 15) { PARAM.velo_freno = v[13]; PARAM.freno_ms = v[14]; }
  return 1;
}


//escribe los valores actuales en el mismo formato de la trama
void Param_a_trama(char *texto)
{
  sprintf(texto, "DAT/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%.2f/%lu/%.2f/%.2f/%.0f/\r\n",
          PARAM.kp, PARAM.ki, PARAM.kd, PARAM.velo_basica, PARAM.vel_giro,
          PARAM.tau, PARAM.kaw, PARAM.kg, PARAM.error_recta, PARAM.velo_recta,
          PARAM.largo_recta, (unsigned long)PARAM.linea_color, PARAM.giro_recta,
          PARAM.velo_freno, PARAM.freno_ms);
}
