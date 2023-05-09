/*
 * userinterface.c
 *
 *  Created on: 05-May-2023
 *      Author: satish
 */

#include "main.h"
#include "ili9341.h"

void USER_DrawStatusBar(void)
{
    ILI9341_FillRectangle(0u,0u, 320,18, ILI9341_COLOR565(255,128,0));
    ILI9341_FillRectangle(0u,19u, 320,(240-19), ILI9341_BLACK);
}

void USER_DrawAntenna(uint16_t color)
{
    //ILI9341_WriteChar(250, 5, 0 , FontDef font, color, ILI9341_COLOR565(255,128,0));
}

void USER_ClearDisplay(void)
{
    ILI9341_FillRectangle(0u,19u, 320,(240-19), ILI9341_BLACK);
}
