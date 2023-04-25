/*
 * Keypad4x3.h
 *
 *  Created on: Mar 14, 2023
 *      Author: Satish
 */

#ifndef KEYPAD_INC_KEYPAD4X3_H_
#define KEYPAD_INC_KEYPAD4X3_H_

#define KEY_STAR    10u
#define KEY_HASH    11u

void Key_Scan(void);
int8_t Key_GetData(void);


#endif /* KEYPAD_INC_KEYPAD4X3_H_ */
