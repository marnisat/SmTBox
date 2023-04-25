/*
 * RFIDHandler.c
 *
 *  Created on: 19-Apr-2023
 *      Author: Satish
 */

#include "RFIDHandler.h"

CustmerDetails_t CustmerDetails;

void ExtractBcdPhoneNumber(uint8_t *BcdNumber, uint8_t *DecimalNumber)
{
    uint8_t Cnt;
    for( Cnt = 0; Cnt < 5; Cnt++ )
    {
        *DecimalNumber++ = (uint8_t) *BcdNumber >> 4;
        *DecimalNumber++ = (uint8_t) *BcdNumber & 0x0F;
        BcdNumber++;
    }
}

const uint8_t Mx1[6][4]=
{
    {0x12,0x45,0xF2,0xA8},
    {0xB2,0x6C,0x39,0x83},
    {0x55,0xE5,0xDA,0x18},
    {0x1F,0x09,0xCA,0x75},
    {0x99,0xA2,0x50,0xEC},
    {0x2C,0x88,0x7F,0x3D}
};

const uint8_t SectorKeyA[] = {0xFFu,0xFFu,0xFFu,0xFFu,0xFFu,0xFFu};

enum {INIT=0,IDLE,ANTICOLL,AUTH,EVER}RfIdState;
void UserTask1(void const *argument)
{
    uint8_t CardStr[20];
    uint8_t UID[4];
    uint8_t SectorKey[7];
    uint8_t Status = 0;
    while (1)
    {
        vTaskDelay(10);
        Key_Scan();

        switch(RfIdState)
        {
            case INIT:
                CustmerDetails.CardFound = NO;
                RfIdState = IDLE;
                break;

            case IDLE:
                Status = MFRC522_Request(PICC_REQIDL, CardStr);
                if( MI_OK == Status )
                {
                    Status = MFRC522_Anticoll(CardStr);
                    if( MI_OK == Status )
                    {
                        CustmerDetails.Uid = (uint8_t) CardStr[0];
                        CustmerDetails.Uid = CustmerDetails.Uid << 8;
                        CustmerDetails.Uid |= (uint8_t) CardStr[1];
                        CustmerDetails.Uid = CustmerDetails.Uid << 8;
                        CustmerDetails.Uid |= (uint8_t) CardStr[2];
                        CustmerDetails.Uid = CustmerDetails.Uid << 8;
                        CustmerDetails.Uid |= (uint8_t) CardStr[3];

                        Status = MFRC522_SelectTag(CardStr);
                        if( Status > 0 )
                        {
                            RfIdState = AUTH;
                        }
                    }
                }
                break;
            case AUTH:

                Status = MFRC522_Auth(0x60, 7, SectorKeyA, CardStr);
                if (Status == MI_OK)
                {
                    Status = MFRC522_Read(4u, &CardStr);
                    if(MI_OK == Status)
                    {
                        strncpy(CustmerDetails.Name,CardStr,12u);
                        CustmerDetails.Balance = *((uint16_t*)&CardStr[13]);
                    }
                    Status = MFRC522_Read(5u, &CardStr);
                    if(MI_OK == Status)
                    {
                        ExtractBcdPhoneNumber(&CardStr[0],CustmerDetails.Number1);
                        ExtractBcdPhoneNumber(&CardStr[5],CustmerDetails.Number2);
                        ExtractBcdPhoneNumber(&CardStr[10],CustmerDetails.Number3);
                    }
                    Status = MFRC522_Read(6u, &CardStr);
                    if(MI_OK == Status)
                    {
                        CustmerDetails.AcceptenceId   = *((uint32_t*)&CardStr[0]);
                        CustmerDetails.AcceptenceMask = *((uint32_t*)&CardStr[4]);
                        CustmerDetails.CardFound = YES;
                    }
                }
                RfIdState = EVER;
                break;
            case EVER:
                break;

        }
    }
}







//        Status = MFRC522_Request(PICC_REQIDL, CardStr);
//        if( MI_OK == Status )
//        {
//            Status = MFRC522_Anticoll(CardStr);
//            if( MI_OK == Status )
//            {
//                UID[0] = CardStr[0];
//                UID[1] = CardStr[1];
//                UID[2] = CardStr[2];
//                UID[3] = CardStr[3];
//
//                Status = MFRC522_SelectTag(CardStr);
//                if(Status > 0)
//                {
//#if 0
//                    SectorKey[0] = ((Mx1[0][0])^(UID[0])) + ((Mx1[0][1])^(UID[1])) + ((Mx1[0][2])^(UID[2])) + ((Mx1[0][3])^(UID[3]));// 0x11; //KeyA[0]
//                    SectorKey[1] = ((Mx1[1][0])^(UID[0])) + ((Mx1[1][1])^(UID[1])) + ((Mx1[1][2])^(UID[2])) + ((Mx1[1][3])^(UID[3]));// 0x11; //KeyA[0]
//                    SectorKey[2] = ((Mx1[2][0])^(UID[0])) + ((Mx1[2][1])^(UID[1])) + ((Mx1[2][2])^(UID[2])) + ((Mx1[2][3])^(UID[3]));// 0x11; //KeyA[0]
//                    SectorKey[3] = ((Mx1[3][0])^(UID[0])) + ((Mx1[3][1])^(UID[1])) + ((Mx1[3][2])^(UID[2])) + ((Mx1[3][3])^(UID[3]));// 0x11; //KeyA[0]
//                    SectorKey[4] = ((Mx1[4][0])^(UID[0])) + ((Mx1[4][1])^(UID[1])) + ((Mx1[4][2])^(UID[2])) + ((Mx1[4][3])^(UID[3]));// 0x11; //KeyA[0]
//                    SectorKey[5] = ((Mx1[5][0])^(UID[0])) + ((Mx1[5][1])^(UID[1])) + ((Mx1[5][2])^(UID[2])) + ((Mx1[5][3])^(UID[3]));// 0x11; //KeyA[0]
//#else
//                    SectorKey[0] = SectorKeyA[0];
//                    SectorKey[1] = SectorKeyA[1];
//                    SectorKey[2] = SectorKeyA[2];
//                    SectorKey[3] = SectorKeyA[3];
//                    SectorKey[4] = SectorKeyA[4];
//                    SectorKey[5] = SectorKeyA[5];
//#endif
//                    Status = MFRC522_Auth(0x60, 3, SectorKey, CardStr);
//                    if (Status == MI_OK)
//                    {
//
//#if 0
//                        Status = MFRC522_Read(1u, &CardStr);
//                        if(MI_OK == Status)
//                        {
//                            strncpy(CustmerDetails.Name,CardStr,12u);
//                            CustmerDetails.Balance = (uint16_t*)&CardStr[13];
//                        }
//                        Status = MFRC522_Read(2u, &CardStr);
//                        if(MI_OK == Status)
//                        {
//                            ExtractBcdPhoneNumber(&CardStr[0],CustmerDetails.Number1);
//                            ExtractBcdPhoneNumber(&CardStr[5],CustmerDetails.Number2);
//                            ExtractBcdPhoneNumber(&CardStr[10],CustmerDetails.Number3);
//                        }
//                        Status = MFRC522_Read(3u, &CardStr);
//                        if(MI_OK == Status)
//                        {
//                            CustmerDetails.AcceptenceId = (uint32_t*)&CardStr[0];
//                            CustmerDetails.AcceptenceMask = (uint32_t*)&CardStr[4];
//                        }
//#endif
//
//
//
//
//#if DEBUG == 1
//                        printf("Done\n");
//#endif
//                    }
//                }
//            }
//        }
//    }
//}



