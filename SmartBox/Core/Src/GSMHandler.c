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

void TaskGsm(void const * argument)
{
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
    }
}
