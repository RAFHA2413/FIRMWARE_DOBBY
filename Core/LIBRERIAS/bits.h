/*
 * bits.h
 *
 *  Created on: Jul 25, 2023
 *      Author: alcid
 */

#ifndef SRC_LIBRERIAS_BITS_H_
#define SRC_LIBRERIAS_BITS_H_

#include "main.h"

//tipos de 8 bits bit a bit
typedef union {
    uint8_t byte;
    struct {
        uint8_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1;
    };
} uint8_tb;


//tipos de 16 bits bit a bit
typedef union {
    uint16_t word;
    struct {
        uint8_t b0:1, b1:1, b2:1, b3:1, b4:1, b5:1, b6:1, b7:1,b8:1, b9:1, b10:1, b11:1, b12:1, b13:1, b14:1, b15:1;
    };
} uint16_tb;



void PON_UNO (uint32_t  *registro, uint8_t bit1);
void PON_CERO(uint32_t  *registro, uint8_t bit1);
uint8_t LEEBIT (uint32_t  *registro, uint8_t bit1);
int32_t complemento2(uint32_t value, uint8_t length);



//para poder acceder a nivel de bits

#endif /* SRC_LIBRERIAS_BITS_H_ */
