/*
 * flash_interna.c
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 */

#include "flash_interna.h"
#include <string.h>

//simbolos que crea el linker: el programa ocupa la flash hasta _sidata + (_edata - _sdata)
extern uint32_t _sidata, _sdata, _edata;


//1 = el programa NO llega a la zona reservada (funciona aunque CubeMX regenere el .ld)
uint8_t Flash_zona_libre(void)
{
  uint32_t fin_programa = (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata);
  return (fin_programa <= FLASH_ZONA_INICIO) ? 1 : 0;
}


//CRC32 por software (sin usar el periferico CRC)
uint32_t Flash_crc32(const void *datos, uint32_t bytes)
{
  const uint8_t *p = (const uint8_t *)datos;
  uint32_t crc = 0xFFFFFFFFu;

  for (uint32_t i = 0; i < bytes; i++)
  {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
  }
  return ~crc;
}


//borra UN sector de 8 KB a partir de su direccion
static HAL_StatusTypeDef Flash_borra_sector(uint32_t dir)
{
  FLASH_EraseInitTypeDef borrar;
  uint32_t error_sector = 0;
  uint32_t offset = dir - FLASH_BASE;
  uint32_t banco  = (offset < FLASH_BANK_SIZE) ? FLASH_BANK_1 : FLASH_BANK_2;

  //si los bancos estan intercambiados (opcion SWAP_BANK), se invierte el banco fisico
  if (READ_BIT(FLASH->OPTSR_CUR, FLASH_OPTSR_SWAP_BANK) != 0u)
    banco = (banco == FLASH_BANK_1) ? FLASH_BANK_2 : FLASH_BANK_1;

  borrar.TypeErase = FLASH_TYPEERASE_SECTORS;
  borrar.Banks     = banco;
  borrar.Sector    = (offset % FLASH_BANK_SIZE) / FLASH_SECTOR_SIZE;
  borrar.NbSectors = 1;
  return HAL_FLASHEx_Erase(&borrar, &error_sector);
}


//borra la zona (dir, tam_zona) y graba 'bytes' de 'datos' al inicio de la zona
//dir y tam_zona deben ser multiplos de 8 KB. Llamar SOLO con el robot detenido.
//1 = grabo y verifico bien, 0 = error
uint8_t Flash_guardar(uint32_t dir, uint32_t tam_zona, const void *datos, uint32_t bytes)
{
  uint32_t qw[4] __attribute__((aligned(16)));   //un bloque de 16 bytes
  const uint8_t *p = (const uint8_t *)datos;
  HAL_StatusTypeDef st = HAL_OK;

  //validaciones
  if (!Flash_zona_libre()) return 0;                                  //el programa llego a la zona
  if ((dir < FLASH_ZONA_INICIO) || (dir + tam_zona > FLASH_FIN)) return 0;   //fuera de lo reservado
  if (((dir - FLASH_BASE) % FLASH_SECTOR_SIZE) != 0u) return 0;       //no alineado a sector
  if ((tam_zona % FLASH_SECTOR_SIZE) != 0u) return 0;
  if (bytes > tam_zona) return 0;                                     //no cabe

  //el ICACHE guarda copias de la flash: se apaga mientras se graba
  HAL_ICACHE_Disable();
  HAL_FLASH_Unlock();

  //borra solo los sectores necesarios para los datos
  for (uint32_t s = 0; (st == HAL_OK) && (s < bytes); s += FLASH_SECTOR_SIZE)
    st = Flash_borra_sector(dir + s);

  //graba de a 16 bytes (el ultimo bloque se completa con 0)
  for (uint32_t i = 0; (st == HAL_OK) && (i < bytes); i += 16u)
  {
    uint32_t n = ((bytes - i) < 16u) ? (bytes - i) : 16u;
    memset(qw, 0, sizeof(qw));
    memcpy(qw, p + i, n);
    st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, dir + i, (uint32_t)qw);
  }

  HAL_FLASH_Lock();
  HAL_ICACHE_Enable();

  if (st != HAL_OK) return 0;

  //verifica leyendo lo grabado
  return (memcmp((const void *)dir, datos, bytes) == 0) ? 1 : 0;
}
