/*
 * diskio.c
 *
 *  Created on: Nov 2, 2026
 *      Author: ALCIDES_RAMOS
 *
 *  Driver de bajo nivel de FatFS (R0.12c) para la tarjeta SD por SDMMC1
 *  del STM32H5, usando el HAL en modo bloqueante (polling).
 *
 *  - Inicializa la tarjeta aqui (no hace falta que CubeMX llame a MX_SDMMC1_SD_Init):
 *    si no hay tarjeta devuelve error en vez de quedarse en Error_Handler().
 *  - Si salen errores de lectura/escritura (overrun/underrun), subir SD_CLOCK_DIV
 *    o activar "Hardware flow control" en CubeMX.
 */

#include "ff.h"
#include "diskio.h"

extern SD_HandleTypeDef hsd1;

#define SD_CLOCK_DIV   8      //SDMMC_CK = 250 MHz / (2 * 8) = 15.6 MHz
#define SD_TIMEOUT     1000   //ms

static volatile DSTATUS Stat = STA_NOINIT;


//espera a que la tarjeta este lista para otra operacion (1 = lista)
static uint8_t SD_espera_lista(uint32_t timeout)
{
  uint32_t t0 = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - t0) > timeout) return 0;
  }
  return 1;
}


DSTATUS disk_initialize(BYTE pdrv)
{
  if (pdrv) return STA_NOINIT;

  //si CubeMX ya la inicializo bien, no se repite
  if (hsd1.State != HAL_SD_STATE_READY)
  {
    //misma configuracion de CubeMX (por si no se llamo MX_SDMMC1_SD_Init)
    hsd1.Instance                 = SDMMC1;
    hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv            = SD_CLOCK_DIV;

    if (HAL_SD_Init(&hsd1) != HAL_OK)
    {
      Stat = STA_NOINIT;   //no hay tarjeta o no responde
      return Stat;
    }
  }

  if (SD_espera_lista(SD_TIMEOUT)) Stat &= (DSTATUS)~STA_NOINIT;
  else                             Stat = STA_NOINIT;

  return Stat;
}


DSTATUS disk_status(BYTE pdrv)
{
  if (pdrv) return STA_NOINIT;
  return Stat;
}


DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  if (!SD_espera_lista(SD_TIMEOUT)) return RES_ERROR;

  //el HAL convierte a direccion en bytes si la tarjeta es SDSC (<= 2 GB)
  if (HAL_SD_ReadBlocks(&hsd1, buff, sector, count, SD_TIMEOUT + 50u * count) != HAL_OK)
    return RES_ERROR;

  return SD_espera_lista(SD_TIMEOUT) ? RES_OK : RES_ERROR;
}


#if _USE_WRITE == 1
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv || !count) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  if (!SD_espera_lista(SD_TIMEOUT)) return RES_ERROR;

  if (HAL_SD_WriteBlocks(&hsd1, buff, sector, count, SD_TIMEOUT + 50u * count) != HAL_OK)
    return RES_ERROR;

  //la tarjeta sigue grabando internamente despues de recibir los datos
  return SD_espera_lista(SD_TIMEOUT) ? RES_OK : RES_ERROR;
}
#endif


#if _USE_IOCTL == 1
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  HAL_SD_CardInfoTypeDef info;

  if (pdrv) return RES_PARERR;
  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
    case CTRL_SYNC:
      return SD_espera_lista(SD_TIMEOUT) ? RES_OK : RES_ERROR;

    case GET_SECTOR_COUNT:
      HAL_SD_GetCardInfo(&hsd1, &info);
      *(DWORD *)buff = info.LogBlockNbr;
      return RES_OK;

    case GET_SECTOR_SIZE:
      HAL_SD_GetCardInfo(&hsd1, &info);
      *(WORD *)buff = (WORD)info.LogBlockSize;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;   //tamano de borrado desconocido
      return RES_OK;

    default:
      return RES_PARERR;
  }
}
#endif


//fecha de los archivos (sin RTC): 1 de enero de 2026, 00:00
DWORD get_fattime(void)
{
  return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
}
