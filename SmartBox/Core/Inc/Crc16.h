/*
 * Crc16.h
 *
 *  Created on: 05-May-2023
 *      Author: satish
 */

#ifndef INC_CRC16_H_
#define INC_CRC16_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef uint16_t    CRC16_t;

#define SMN_CRC316_POLYNOMIAL                   0x1021


uint16_t CRC16_ComputeCrc(uint8_t DataStream[], int DataSize);


#endif /* INC_CRC16_H_ */
