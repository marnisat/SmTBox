/*
 * RFIDHandler.h
 *
 *  Created on: 19-Apr-2023
 *      Author: Satish
 */

#ifndef INC_RFIDHANDLER_H_
#define INC_RFIDHANDLER_H_

#include <sys/_stdint.h>
#include "MFRC522.h"
#include "typedefs.h"

typedef struct
{
    Sts_t Found;
    uint8_t Uid[10];

    uint8_t Name[12];
    uint16_t Balance;
    uint8_t Resrv1[2];
    uint8_t Number1[10];
    uint8_t Number2[10];
    uint8_t Number3[10];
    uint8_t Resrv2;
    uint32_t AcceptenceId;
    uint32_t AcceptenceMask;
}SmartCard_t;

extern SmartCard_t SmartCard;


#endif /* INC_RFIDHANDLER_H_ */
