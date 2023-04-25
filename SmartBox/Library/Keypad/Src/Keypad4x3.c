/*
 * Keypad4x3.c
 *
 *  Created on: Mar 14, 2023
 *      Author: Satish
 */

#include "stm32f1xx_hal.h"
#include "main.h"
#include "Keypad4x3.h"


typedef struct
{
    GPIO_TypeDef *Port;
    uint16_t PortPin;
}KeyScanPort_t;

static void StoreKeyDat(uint8_t Key);


const KeyScanPort_t KeyScanPort[3] =
{
    {   KEY_SCAN1_GPIO_Port, KEY_SCAN1_Pin},
    {   KEY_SCAN2_GPIO_Port, KEY_SCAN2_Pin},
    {   KEY_SCAN3_GPIO_Port, KEY_SCAN3_Pin}
};


static uint8_t KeyReturnScanByte(void)
{
    uint8_t RetByte = 0u;
    GPIO_PinState PinState = GPIO_PIN_SET;

    PinState = HAL_GPIO_ReadPin(KEY_RET1_GPIO_Port, KEY_RET1_Pin);
    if( PinState == GPIO_PIN_RESET )
    {
        RetByte = (RetByte | 0x01u);
    }

    PinState = HAL_GPIO_ReadPin(KEY_RET2_GPIO_Port, KEY_RET2_Pin);
    if( PinState == GPIO_PIN_RESET )
    {
        RetByte = (RetByte | 0x02u);
    }

    PinState = HAL_GPIO_ReadPin(KEY_RET3_GPIO_Port, KEY_RET3_Pin);
    if( PinState == GPIO_PIN_RESET )
    {
        RetByte = (RetByte | 0x04u);
    }

    PinState = HAL_GPIO_ReadPin(KEY_RET4_GPIO_Port, KEY_RET4_Pin);
    if( PinState == GPIO_PIN_RESET )
    {
        RetByte = (RetByte | 0x08u);
    }

    RetByte = (RetByte & 0x0Fu);
    return RetByte;
}


const uint8_t Matrix[3][4] =
{
        {3,6,9,12},
        {2,5,8,0},
        {1,4,7,11}
};

typedef struct
{
    uint8_t Buffer[0x0Fu+1u];
    uint8_t WrtIndex;
    uint8_t RdIndex;
}KeyQueue_t;

static KeyQueue_t KeyQueue;

void Key_Scan(void)
{
    uint8_t CCount = 0u;
    uint8_t CStatus = 0u;
    uint8_t KeyFoundCount = 0;
    uint8_t Key = 0x0Fu;
    static uint8_t BounceCnt = 0;
    static uint8_t PrvKey = 0;
    static uint8_t KeyScanState = 0x0u;


    for( CCount = 0u; CCount < 3u; CCount++ )
    {
        HAL_GPIO_WritePin(KeyScanPort[CCount].Port, KeyScanPort[CCount].PortPin, GPIO_PIN_RESET);
        CStatus = KeyReturnScanByte();
        HAL_GPIO_WritePin(KeyScanPort[CCount].Port, KeyScanPort[CCount].PortPin, GPIO_PIN_SET);

        switch( CStatus )
        {
            case 0x01u:
                Key = Matrix[CCount][3];
                KeyFoundCount++;
                break;
            case 0x02u:
                Key = Matrix[CCount][0];
                KeyFoundCount++;
                break;
            case 0x04u:
                Key = Matrix[CCount][1];
                KeyFoundCount++;
                break;
            case 0x08u:
                Key = Matrix[CCount][2];
                KeyFoundCount++;
                break;
            default:
                break;
        }
    } /* CCount Loop */



    switch( KeyScanState )
    {
    case 0u: /* Wait for key */
        if( 1 == KeyFoundCount )
        {
            PrvKey = Key;
            BounceCnt = 0;
            KeyScanState = 1;
        }
        break;
    case 1u:
        if( PrvKey == Key )
        {
            if( BounceCnt > 5 )
            {
                KeyScanState = 2;
            }
            else
            {
                BounceCnt++;
            }
        }
        else
        {
            KeyScanState = 0;
            BounceCnt = 0;
        }
        break;
    case 2u:
        if( PrvKey == Key )
        {
            BounceCnt = 0u;
        }
        else
        {
            BounceCnt++;
            if( BounceCnt > 5 )
            {
                /* Push Key */
                KeyScanState = 0;
                StoreKeyDat(PrvKey);
            }
        }

        break;
    }
}


static void StoreKeyDat(uint8_t Key)
{
    KeyQueue.Buffer[KeyQueue.WrtIndex++] = Key;
    KeyQueue.WrtIndex = KeyQueue.WrtIndex & 0x0Fu;
}


int8_t Key_GetData(void)
{
    int8_t Key = -1;
    if(KeyQueue.WrtIndex != KeyQueue.RdIndex)
    {
        Key = KeyQueue.Buffer[KeyQueue.RdIndex++];
        KeyQueue.RdIndex = KeyQueue.RdIndex & 0x0Fu;
    }
    return Key;
}

void Key_Init(void)
{
    KeyQueue.WrtIndex = 0;
    KeyQueue.RdIndex = 0;
}
