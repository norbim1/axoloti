/*
 * Copyright 2013 Xavier Hosxe
 *
 * Author: Xavier Hosxe (xavier . hosxe (at) gmail . com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Mod for Axoloti local control by MN
 *
 */

#ifndef ENCODERS_H_
#define ENCODERS_H_

//#include "stm32f4xx.h"
#include "hal.h"

// Back button
#define BUTTON0_PIN  2
#define BUTTON0_PORT GPIOE
// Enter button
#define BUTTON1_PIN  3
#define BUTTON1_PORT GPIOE
// Up button
#define BUTTON2_PIN  4
#define BUTTON2_PORT GPIOE
// Left button
#define BUTTON3_PIN  5
#define BUTTON3_PORT GPIOE
// Right button
#define BUTTON4_PIN  6
#define BUTTON4_PORT GPIOE
// Down button
#define BUTTON5_PIN  7
#define BUTTON5_PORT GPIOE
// Shift button
#define BUTTON6_PIN  8
#define BUTTON6_PORT GPIOE

//#define BUTTON7_PIN  GPIO_Pin_1
//#define BUTTON7_PORT GPIOB

#define ENC1_0_PIN  10
#define ENC1_0_PORT GPIOE
#define ENC1_2_PIN  9
#define ENC1_2_PORT GPIOE
/*
#define ENC2_0_PIN  GPIO_Pin_2
#define ENC2_0_PORT GPIOA
#define ENC2_2_PIN  GPIO_Pin_4
#define ENC2_2_PORT GPIOC
#define ENC3_0_PIN  GPIO_Pin_2
#define ENC3_0_PORT GPIOE
#define ENC3_2_PIN  GPIO_Pin_4
#define ENC3_2_PORT GPIOE
#define ENC4_0_PIN  GPIO_Pin_6
#define ENC4_0_PORT GPIOE
#define ENC4_2_PIN  GPIO_Pin_14
#define ENC4_2_PORT GPIOC
*/

#define NUMBER_OF_ENCODERS 1
#define NUMBER_OF_BUTTONS 7

enum LastEncoderMove_t {
	LAST_MOVE_NONE = 0,
	LAST_MOVE_INC,
	LAST_MOVE_DEC
};

struct EncoderStatus {
	char value;
	bool_t b1;
};

void EncodersInit(void);
void EncoderscheckStatus(void);
//void EncoderscheckSimpleStatus();

void EncodersencoderTurned(int encoder, int ticks);
void EncodersbuttonPressed(int button);
void EncodersbuttonReleased(int button);


#endif /* ENCODERS_H_ */
