/*
 * flash_interna.h
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Guardado de datos en la FLASH interna del STM32H5 (sectores de 8 KB,
 *  grabacion de a 16 bytes). La usan las librerias parametros y mapa.
 *
 *  Zonas reservadas al final de la flash (STM32H562VG, 1 MB):
 *
 *   0x08000000 +------------------------+
 *              | programa               |
 *              | ...libre...            |
 *   0x080DE000 +------------------------+ FLASH_ZONA_INICIO
 *              | MAPA       (128 KB)    | FLASH_MAPA_DIR
 *   0x080FE000 +------------------------+
 *              | PARAMETROS (8 KB)      | FLASH_PARAM_DIR
 *   0x08100000 +------------------------+ fin
 *
 *  Proteccion: Flash_guardar() revisa con los simbolos del linker que el programa
 *  no llegue a FLASH_ZONA_INICIO; si llega, NO borra nada y devuelve 0.
 *  Opcional: en STM32H562xx_FLASH.ld poner FLASH LENGTH = 888K
 *  (1024 - 128 - 8) para que el linker avise si el codigo crece hasta ahi.
 */

#ifndef LIBRERIAS_FLASH_INTERNA_H_
#define LIBRERIAS_FLASH_INTERNA_H_

#include "main.h"

//como usar
/*
   //guardar una struct o arreglo (borra los sectores que ocupan los datos y graba) -> 1 = ok, 0 = error
   Flash_guardar(FLASH_PARAM_DIR, FLASH_PARAM_TAM, &mis_datos, sizeof(mis_datos));

   //leer: la flash se lee como memoria normal
   memcpy(&mis_datos, (const void *)FLASH_PARAM_DIR, sizeof(mis_datos));

   //verificar integridad de los datos con un CRC
   uint32_t crc = Flash_crc32(&mis_datos, bytes);

   Grabar SOLO con el robot detenido (borrar 8 KB tarda algunos ms).
 */

//tamano de cada zona (multiplos de 8 KB)
#define FLASH_PARAM_TAM    (8u * 1024u)
#define FLASH_MAPA_TAM     (128u * 1024u)

//direcciones (se calculan desde el final de la flash del chip)
#define FLASH_FIN          (FLASH_BASE + FLASH_SIZE)
#define FLASH_PARAM_DIR    (FLASH_FIN - FLASH_PARAM_TAM)
#define FLASH_MAPA_DIR     (FLASH_PARAM_DIR - FLASH_MAPA_TAM)
#define FLASH_ZONA_INICIO  (FLASH_MAPA_DIR)   //inicio de todo lo reservado

uint8_t  Flash_zona_libre(void);                 //1 = el programa no toca la zona reservada
uint8_t  Flash_guardar(uint32_t dir, uint32_t tam_zona, const void *datos, uint32_t bytes);
uint32_t Flash_crc32(const void *datos, uint32_t bytes);

#endif /* LIBRERIAS_FLASH_INTERNA_H_ */
