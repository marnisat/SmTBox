#ifndef	_EE24_H
#define	_EE24_H

#ifdef __cplusplus
extern "C" {
#endif
/*
  Author:     Nima Askari
  WebSite:    http://www.github.com/NimaLTD
  Instagram:  http://instagram.com/github.NimaLTD
  Youtube:    https://www.youtube.com/channel/UCUhY7qY1klJm1d2kulr9ckw
  
  Version:    2.2.1
  
  
  Reversion History:
  
  (2.2.1)
  Fix erase chip bug.
  
  (2.2.0)
  Add erase chip function.
  
  (2.1.0)
  Fix write bytes.
  
  (2.0.0)
  Rewrite again.

*/

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "ee24Config.h"
#include  "Crc16.h"

#pragma push(pack,1)
typedef struct
{
    uint16_t Signature; /* 2 Bytes */
    uint16_t LockId;    /* 2 Bytes */
    SysCfg_t SysCfg;    /* 8 Bytes */
    CRC16_t Crc;        /* 2 Bytes */
}EEPage1_t; /* 14 Bytes */
#pragma pop(pack)

#define SIGNATURE   (0xA5A5u)
#define CFG_ADD     0x10u
#define CFG_SIZE    14u //(sizeof(EEPage1_t))


bool    ee24_isConnected(void);
bool    ee24_write(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);	
bool    ee24_read(uint16_t address, uint8_t *data, size_t lenInBytes, uint32_t timeout);
bool    ee24_eraseChip(void);
int32_t EE24_ReadConfig(SysCfg_t * SysCfg);
int32_t EE24_SetCfgDefaults(void);

#ifdef __cplusplus
}
#endif

#endif
