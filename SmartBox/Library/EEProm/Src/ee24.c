
/* #include "i2c.h" */
#include "ee24.h"
#if (_EEPROM_USE_FREERTOS == 1)
#include "cmsis_os.h"
#define ee24_delay(x)   osDelay(x)
#else
#define ee24_delay(x)   HAL_Delay(x)
#endif

#if (_EEPROM_SIZE_KBIT == 1) || (_EEPROM_SIZE_KBIT == 2)
#define _EEPROM_PSIZE     8
#elif (_EEPROM_SIZE_KBIT == 4) || (_EEPROM_SIZE_KBIT == 8) || (_EEPROM_SIZE_KBIT == 16)
#define _EEPROM_PSIZE     16
#else
#define _EEPROM_PSIZE     32
#endif

uint8_t ee24_lock = 0;
//################################################################################################################
bool ee24_isConnected(void)
{
#if (_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO,_EEPROM_WP_PIN,GPIO_PIN_SET);
#endif
    if( HAL_I2C_IsDeviceReady(&hi2c1, _EEPROM_ADDRESS, 2, 100) == HAL_OK )
        return true;
    else
        return false;
}
//################################################################################################################
bool ee24_write(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
    if( ee24_lock == 1 )
        return false;
    ee24_lock = 1;
    uint16_t w;
    uint32_t startTime = HAL_GetTick();
#if	(_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN,GPIO_PIN_RESET);
  #endif
    while (1)
    {
        w = _EEPROM_PSIZE - (address % _EEPROM_PSIZE);
        if( w > len )
            w = len;
#if ((_EEPROM_SIZE_KBIT==1) || (_EEPROM_SIZE_KBIT==2))
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==4)
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==8)
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK)
    #elif (_EEPROM_SIZE_KBIT==16)
        if( HAL_I2C_Mem_Write(&hi2c1, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff),
                I2C_MEMADD_SIZE_8BIT, data, w, 100) == HAL_OK )
        #else
    if (HAL_I2C_Mem_Write(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, w, 100) == HAL_OK)
    #endif
        {
            ee24_delay(10);
            len -= w;
            data += w;
            address += w;
            if( len == 0 )
            {
#if (_EEPROM_USE_WP_PIN==1)
        HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
                ee24_lock = 0;
                return true;
            }
            if( HAL_GetTick() - startTime >= timeout )
            {
                ee24_lock = 0;
                return false;
            }
        }
        else
        {
#if (_EEPROM_USE_WP_PIN==1)
      HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
#endif
            ee24_lock = 0;
            return false;
        }
    }
}
//################################################################################################################
bool ee24_read(uint16_t address, uint8_t *data, size_t len, uint32_t timeout)
{
    if( ee24_lock == 1 )
        return false;
    ee24_lock = 1;
#if (_EEPROM_USE_WP_PIN==1)
  HAL_GPIO_WritePin(_EEPROM_WP_GPIO, _EEPROM_WP_PIN, GPIO_PIN_SET);
  #endif
#if ((_EEPROM_SIZE_KBIT==1) || (_EEPROM_SIZE_KBIT==2))
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 4)
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0100) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT == 8)
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS | ((address & 0x0300) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK)
  #elif (_EEPROM_SIZE_KBIT==16)
    if( HAL_I2C_Mem_Read(&hi2c1, _EEPROM_ADDRESS | ((address & 0x0700) >> 7), (address & 0xff), I2C_MEMADD_SIZE_8BIT,
            data, len, 100) == HAL_OK )
    #else
  if (HAL_I2C_Mem_Read(&_EEPROM_I2C, _EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT, data, len, timeout) == HAL_OK)
  #endif
    {
        ee24_lock = 0;
        return true;
    }
    else
    {
        ee24_lock = 0;
        return false;
    }
}
//################################################################################################################
bool ee24_eraseChip(void)
{
  const uint8_t eraseData[32] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF\
    , 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint32_t bytes = 0;
  while ( bytes < (_EEPROM_SIZE_KBIT * 256))
  {
    if (ee24_write(bytes, (uint8_t*)eraseData, sizeof(eraseData), 100) == false)
      return false;
    bytes += sizeof(eraseData);           
  }
  return true;  
}
//################################################################################################################

int32_t EE24_ReadConfig(SysCfg_t * SysCfg)
{
    uint8_t RdBuff[16] = {0};
    CRC16_t CfcCrc = 0;
    CRC16_t RdCrc = 0;
    int32_t Result = false;
    Result = (int32_t)ee24_read(CFG_ADD,RdBuff,CFG_SIZE,100);
    if(true == Result)
    {
        if(((EEPage1_t*)RdBuff)->Signature == SIGNATURE)
        {
            CfcCrc = CRC16_ComputeCrc(RdBuff,(CFG_SIZE - 2u));
            RdCrc = ((EEPage1_t*)RdBuff)->Crc;
            if( ((EEPage1_t*)RdBuff)->Crc == CfcCrc )
            {
                SysCfg->Bid             = ((EEPage1_t*)RdBuff)->SysCfg.Bid;
                SysCfg->BoxActiveStatus = ((EEPage1_t*)RdBuff)->SysCfg.BoxActiveStatus;
                SysCfg->Mode            = ((EEPage1_t*)RdBuff)->SysCfg.Mode;
                SysCfg->UnitPrice       = ((EEPage1_t*)RdBuff)->SysCfg.UnitPrice;
                SysCfg->SCharge         = ((EEPage1_t*)RdBuff)->SysCfg.SCharge;
                Result = 0;
            }
            else
            {
                Result = -1;    /* Memory Corruption */
            }
        }
        else
        {
            Result = -1;    /* Not Formatted,(First PowerON) */
        }
    }
    return Result;
}

#define DEF_BID         0xFFFFFFFFu
#define DEF_BOX_ASTATUS -1
#define DEF_UnitPrice   30u
#define DEF_SerCharge   100u
#define DEF_Mode        0u



int32_t EE24_SetCfgDefaults(void)
{
    int32_t Result;
    uint8_t WrtBuff[16] = { 0 };
    ((EEPage1_t*) WrtBuff)->Signature               = SIGNATURE;
    ((EEPage1_t*) WrtBuff)->LockId                  = 0x1234;
    ((EEPage1_t*) WrtBuff)->SysCfg.Bid              = DEF_BID;
    ((EEPage1_t*) WrtBuff)->SysCfg.BoxActiveStatus  = DEF_BOX_ASTATUS;
    ((EEPage1_t*) WrtBuff)->SysCfg.UnitPrice        = DEF_UnitPrice;
    ((EEPage1_t*) WrtBuff)->SysCfg.SCharge          = DEF_SerCharge;
    ((EEPage1_t*) WrtBuff)->SysCfg.Mode             = DEF_Mode;
    ((EEPage1_t*) WrtBuff)->Crc = CRC16_ComputeCrc(WrtBuff, (CFG_SIZE - 2));
    Result = ee24_write(CFG_ADD, WrtBuff, CFG_SIZE, 100u);
    return Result;
}

