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
    int32_t CardFound;
    uint32_t Uid;
    uint8_t Name[12];
    uint16_t Balance;
    uint8_t Resrv1[2];
    uint8_t Number1[10];
    uint8_t Number2[10];
    uint8_t Number3[10];
    uint8_t Resrv2;
    uint32_t AcceptenceId;
    uint32_t AcceptenceMask;
}CustmerDetails_t;

extern CustmerDetails_t CustmerDetails;


#endif /* INC_RFIDHANDLER_H_ */
