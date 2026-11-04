/*
 * sd_card.h
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Funciones sencillas para usar la tarjeta SD (FatFS) por SDMMC1.
 *  El driver de bajo nivel esta en LIBRERIAS/FATFS/diskio.c
 */

#ifndef LIBRERIAS_SD_CARD_H_
#define LIBRERIAS_SD_CARD_H_

#include "main.h"
#include "ff.h"

//como usar
/*
 1) En main.c:
      if (SD_INIT()) { //tarjeta montada }  else { //no hay tarjeta }

 2) Escribir:
      SD_WRITE_TEXT("datos.txt", texto);       //crea/borra el archivo y escribe
      SD_WRITE_TEXT_ADD("datos.txt", texto);   //agrega al final del archivo

 3) Leer linea por linea:
      if (SD_OPEN_READ("datos.txt"))
          while (SD_READ_LINE())               //cada linea queda en SD_BUFFER
              uartx_write_text(&huart1, SD_BUFFER);
      //SD_READ_LINE cierra el archivo al llegar al final

 4) Leer una linea especifica (0 = primera):
      SD_READ_LINE_N("datos.txt", 3);          //queda en SD_BUFFER

 5) Otros:
      SD_FILE_EXIST("datos.txt");  SD_ESPACIO(&total_kb, &libre_kb);  SD_EXIT();

 IMPORTANTE: no escribir en la SD mientras el robot corre (una escritura puede
 tardar de 10 a mas de 100 ms). Guardar en RAM y escribir con el robot detenido.
 */

#define SD_SIZE_BUFF  200   //largo maximo de una linea leida

extern char    file_name[];
extern char    SD_BUFFER[SD_SIZE_BUFF];
extern FATFS   fs;
extern FIL     fil;
extern FRESULT fres;   //resultado de la ultima operacion (para depurar)

uint8_t SD_INIT(void);                                         //1 = tarjeta montada
uint8_t SD_EXIT(void);                                         //desmonta la tarjeta
uint8_t SD_CREA_FILE(const TCHAR *nombre);                     //crea vacio (borra si existe)
uint8_t SD_WRITE_TEXT(const TCHAR *nombre, const TCHAR *datos);     //crea/borra y escribe
uint8_t SD_WRITE_TEXT_ADD(const TCHAR *nombre, const TCHAR *datos); //agrega al final
uint8_t SD_OPEN_READ(const TCHAR *nombre);                     //abre para leer
uint8_t SD_READ_LINE(void);                                    //1 = leyo linea en SD_BUFFER
uint8_t SD_READ_LINE_N(const TCHAR *nombre, uint32_t linea);   //1 = linea en SD_BUFFER
uint8_t SD_CLOSE_FILE(void);
uint8_t SD_FILE_EXIST(const TCHAR *nombre);                    //1 = existe
uint8_t SD_ESPACIO(uint32_t *total_kb, uint32_t *libre_kb);    //1 = ok

#endif /* LIBRERIAS_SD_CARD_H_ */
