/*
 * sd_card.c
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "sd_card.h"
#include <string.h>

char    file_name[] = "datos.txt";   //nombre de archivo por defecto
char    SD_BUFFER[SD_SIZE_BUFF];
FATFS   fs;
FIL     fil;
FRESULT fres;


uint8_t SD_INIT(void)
{
  fres = f_mount(&fs, "", 1);   //1 = monta ya (inicializa la tarjeta)
  return (fres == FR_OK) ? 1 : 0;
}

uint8_t SD_EXIT(void)
{
  fres = f_mount(NULL, "", 0);
  return (fres == FR_OK) ? 1 : 0;
}

uint8_t SD_CREA_FILE(const TCHAR *nombre)
{
  fres = f_open(&fil, nombre, FA_CREATE_ALWAYS | FA_WRITE);
  if (fres != FR_OK) return 0;
  fres = f_close(&fil);
  return (fres == FR_OK) ? 1 : 0;
}

//escribe 'datos' con el modo indicado y cierra el archivo
static uint8_t SD_escribe(const TCHAR *nombre, const TCHAR *datos, BYTE modo)
{
  UINT escritos = 0;
  UINT largo = (UINT)strlen(datos);

  fres = f_open(&fil, nombre, modo | FA_WRITE);
  if (fres != FR_OK) return 0;

  fres = f_write(&fil, datos, largo, &escritos);
  FRESULT fc = f_close(&fil);   //el cierre es el que asegura que queda grabado
  if (fres == FR_OK) fres = fc;

  return ((fres == FR_OK) && (escritos == largo)) ? 1 : 0;
}

uint8_t SD_WRITE_TEXT(const TCHAR *nombre, const TCHAR *datos)
{
  return SD_escribe(nombre, datos, FA_CREATE_ALWAYS);
}

uint8_t SD_WRITE_TEXT_ADD(const TCHAR *nombre, const TCHAR *datos)
{
  return SD_escribe(nombre, datos, FA_OPEN_APPEND);
}

uint8_t SD_OPEN_READ(const TCHAR *nombre)
{
  fres = f_open(&fil, nombre, FA_READ);
  return (fres == FR_OK) ? 1 : 0;
}

//lee la siguiente linea del archivo abierto con SD_OPEN_READ
//al llegar al final cierra el archivo y devuelve 0
uint8_t SD_READ_LINE(void)
{
  memset(SD_BUFFER, 0, sizeof(SD_BUFFER));

  if (f_gets(SD_BUFFER, sizeof(SD_BUFFER), &fil) != NULL) return 1;

  f_close(&fil);
  return 0;
}

//lee la linea numero 'linea' (0 = primera) y la deja en SD_BUFFER
uint8_t SD_READ_LINE_N(const TCHAR *nombre, uint32_t linea)
{
  uint32_t n = 0;

  if (!SD_OPEN_READ(nombre)) return 0;

  while (SD_READ_LINE())   //cierra el archivo si llega al final
  {
    if (n == linea)
    {
      f_close(&fil);
      return 1;
    }
    n++;
  }
  return 0;
}

uint8_t SD_CLOSE_FILE(void)
{
  fres = f_close(&fil);
  return (fres == FR_OK) ? 1 : 0;
}

uint8_t SD_FILE_EXIST(const TCHAR *nombre)
{
  FILINFO fno;
  fres = f_stat(nombre, &fno);
  return (fres == FR_OK) ? 1 : 0;
}

uint8_t SD_ESPACIO(uint32_t *total_kb, uint32_t *libre_kb)
{
  FATFS *pfs;
  DWORD libres;

  fres = f_getfree("", &libres, &pfs);
  if (fres != FR_OK) return 0;

  //clusters * sectores por cluster * 512 bytes / 1024
  *total_kb = (uint32_t)((pfs->n_fatent - 2u) * pfs->csize / 2u);
  *libre_kb = (uint32_t)(libres * pfs->csize / 2u);
  return 1;
}
