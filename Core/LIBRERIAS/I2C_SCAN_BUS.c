/*
 * I2C_SACAN_BUS.c
 *
 *  Created on: 31/03/2026
 *      Author: Alcides Ramos
 */
#include "I2C_SCAN_BUS.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"


//DEBE PONERSE <<1  A LA DIRECCION ENCONTRADA MICRO
/* ANTERIOR
void I2C_SCAN(I2C_HandleTypeDef I2CMIO)
{
char buffer[32];
	for (uint8_t addr = 0; addr < 127; addr++)
	{
		if (HAL_I2C_IsDeviceReady(&I2CMIO, addr << 1, 2, 300) == HAL_OK)
		{
			sprintf(buffer, "DIR: 0x%02X\r\n", addr);
			uartx_write_text(&uart_i2c , buffer);
		}
	}
}
*/
//>>>>> CAMBIO: scan con diagnostico
// - antes de escanear lee SCL y SDA: en reposo AMBOS deben estar en 1 (pull-ups)
// - cuenta cuantos encontro y avisa si no hubo ninguno
// - si el bus esta trabado la HAL devuelve HAL_BUSY (no NACK), se reporta aparte
void I2C_SCAN(I2C_HandleTypeDef I2CMIO)
{
char buffer[48];
uint8_t encontrados=0;
uint8_t ocupado=0;
HAL_StatusTypeDef res;

	// PB6=SCL PB7=SDA (I2C1), cambiar si se usa otro I2C
	sprintf(buffer, "I2C SCAN  SCL=%d SDA=%d\r\n",
			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6), HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7));
	uartx_write_text(&uart_i2c , buffer);

	for (uint8_t addr = 1; addr < 128; addr++)
	{
		res = HAL_I2C_IsDeviceReady(&I2CMIO, addr << 1, 2, 10);
		if (res == HAL_OK)
		{
			sprintf(buffer, "DIR: 0x%02X\r\n", addr);
			uartx_write_text(&uart_i2c , buffer);
			encontrados++;
		}
		else if (res == HAL_BUSY) ocupado++;
	}

	if (ocupado)
	{
		sprintf(buffer, "BUS OCUPADO en %d dirs (SDA/SCL en 0?)\r\n", ocupado);
		uartx_write_text(&uart_i2c , buffer);
	}
	sprintf(buffer, "FIN SCAN: %d dispositivos\r\n", encontrados);
	uartx_write_text(&uart_i2c , buffer);
}


void  I2C_RESPONDE(I2C_HandleTypeDef I2CMIO,uint8_t dir)
{

	//inetnto 3 veces hasta por 300 ms  si no repsonde
	uint8_t si_estoy = HAL_I2C_IsDeviceReady(&I2CMIO, dir , 3, 300);

	if(si_estoy == HAL_OK)
	{
		uartx_write_text(&uart_i2c, "SI ESTOY\r\n");

	}
	else
		uartx_write_text(&uart_i2c, "NO EXISTO\r\n");


}


