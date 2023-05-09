/*
 * Crc16.c
 *
 *  Created on: 05-May-2023
 *      Author: satish
 */

#include "crc16.h"

static uint8_t ReverseByte(uint8_t DataByte);
static uint16_t Reverse16bitvalue(uint16_t WordData);

uint16_t CRC16_ComputeCrc(uint8_t DataStream[], int DataSize)
{
    int ForLoopCnt = 0;
    uint16_t CrcValue = 0xFFFF;
    int BitsForLoopCnt = 0;
    uint8_t Data = 0;
    const uint16_t Polynomial = SMN_CRC316_POLYNOMIAL;/* divisor is 16bit */

    for( ForLoopCnt = 0; ForLoopCnt < DataSize; ForLoopCnt++ )
    {
        Data = ReverseByte(DataStream[ForLoopCnt]);
        CrcValue = CrcValue ^ ((uint16_t) (Data << 8));/* move byte into MSB of 16bit CRC */

        for( BitsForLoopCnt = 0; BitsForLoopCnt < 8; BitsForLoopCnt++ )
        {
            if( (CrcValue & 0x8000) != 0 )/* test for MSB = bit 15 */
            {
                CrcValue = (uint16_t) ((CrcValue << 1) ^ Polynomial);
            }
            else
            {
                CrcValue = CrcValue << 1;
            }
        }
    }
    CrcValue = Reverse16bitvalue(CrcValue);
    CrcValue = CrcValue ^ 0xFFFF;

    return CrcValue;
}

static uint8_t ReverseByte(uint8_t DataByte)
{
    uint8_t RevByte = 0;
    uint8_t Temp = 0;
    int ForLooopCnt = 0;
    for( ForLooopCnt = 0; ForLooopCnt < 8; ForLooopCnt++ )
    {
        Temp = (DataByte & 1);
        RevByte = RevByte | (Temp << (7 - ForLooopCnt));
        DataByte = DataByte >> 1;
    }
    return (RevByte);
}

static uint16_t Reverse16bitvalue(uint16_t WordData)
{
    uint16_t RevWord = 0;
    uint16_t Temp = 0;
    int ForLooopCnt = 0;

    for( ForLooopCnt = 0; ForLooopCnt < 16; ForLooopCnt++ )
    {
        Temp = (WordData & 1);
        RevWord = RevWord | (Temp << (15 - ForLooopCnt));
        WordData = WordData >> 1;
    }
    return (RevWord);
}
