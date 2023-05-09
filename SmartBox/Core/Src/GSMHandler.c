/*
 * GSMHandler.c
 *
 *  Created on: 20-Apr-2023
 *      Author: Satish
 */

#include "GSMHandler.h"


#if (_GSM_MAIN_POWER == 1)
void GSM_PowerRecycle(void)
{
    HAL_GPIO_WritePin(GSM_PWR_CTRL_GPIO_Port, GSM_PWR_CTRL_Pin, GPIO_PIN_RESET);
    gsm_delay(1000);
    HAL_GPIO_WritePin(GSM_PWR_CTRL_GPIO_Port, GSM_PWR_CTRL_Pin, GPIO_PIN_SET);
}
#endif

#define FN_ATD          0x11u
#define FN_ATH          0x22u
#define FN_DTONE_ON     0x33u
#define FN_DTONE_OFF    0x44u
#define FN_CLDTMF       0x55u




void TaskGsm(void const * argument)
{
    bool Result = false;

#if (_GSM_MAIN_POWER == 1)
    GSM_PowerRecycle();
#endif
    gsm_init();
    gsm_power(true);
#if(_GSM_SIM_DETECTOR == 1)
    if(HAL_GPIO_ReadPin(_GSM_SIM_DET_GPIO, _GSM_SIM_DET_PIN))
    {
      gsm_power(false);
      #if(_GSM_RTOS != 0)
        vTaskSuspend(NULL);
      #endif
    }
  #endif
    while (1)
    {
        gsm_loop();
        if( gsm.status.power == 1 )
        {
            osEvent Msk;
            Msk = osMessageGet(GsmQueue, 10);
            if( Msk.status == osEventMessage )
            {
                uint8_t *MsgBuffer;
                MsgBuffer = Msk.value.p;
                switch( MsgBuffer[0] )
                {
                    case FN_ATD:
                        Result = gsm_call_dial(&MsgBuffer[2], 20);
                        break;
                    case FN_ATH:
                        Result = gsm_call_end();
                        break;
                    case FN_DTONE_ON:
                        gsm_tonePlay(gsm_tone_dialTone, 10000, 50);
                        break;
                    case FN_DTONE_OFF:
                        gsm_toneStop();
                        break;
                    case FN_CLDTMF:
                        break;

                    default:
                        break;
                }
            }
        }
    }
}
