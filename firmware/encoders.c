/*
 * Copyright 2013 Xavier Hosxe
 *
 * Author: Xavier Hosxe (xavier <dot> hosxe (at) g m a i l <dot> com)
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

#include "encoders.h"
#include "ui.h"

int encoderState[NUMBER_OF_ENCODERS];
int timerAction[NUMBER_OF_ENCODERS];

enum LastEncoderMove_t lastMove[NUMBER_OF_ENCODERS];
int tickSpeed[NUMBER_OF_ENCODERS];

int buttonTimer[NUMBER_OF_BUTTONS];
//bool_t buttonUsedFromSomethingElse[NUMBER_OF_BUTTONS];
bool_t buttonPreviousState[NUMBER_OF_BUTTONS];
int firstButtonDown;
int encoderTimer;


	/*
			0: 0000 = 00
			1: 0001 = 00
			2: 0010 = 00
			3: 0011 = 00
			4: 0100 = 02
			5: 0101 = 00
			6: 0110 = 00
			7: 0111 = 01 // 1
			8: 1000 = 00
			9: 1001 = 00
			A: 1010 = 00
			B: 1011 = 02 // 2
			C: 1100 = 00
			D: 1101 = 00
			E: 1110 = 00
			F: 1111 = 00
	*/
	// PIC11 ... N24

//    int actionEnc [16] =    /* N12 */{ 0, 0, 0, 0, 2, 0, 0, 1, 1, 0, 0, 2, 0, 0, 0, 0};
static const int actionEnc [16] =      /* N24 */{ 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0};

static const unsigned short encoderBit1[NUMBER_OF_ENCODERS] = { ENC1_2_PIN}; //, ENC2_2_PIN, ENC2_0_PIN, ENC3_2_PIN, ENC3_0_PIN, ENC4_2_PIN, ENC4_0_PIN};
static const GPIO_TypeDef* encoderPort1[NUMBER_OF_ENCODERS] = { ENC1_2_PORT}; //, ENC2_2_PORT, ENC2_0_PORT, ENC3_2_PORT, ENC3_0_PORT, ENC4_2_PORT, ENC4_0_PORT};
static const unsigned short encoderBit2[NUMBER_OF_ENCODERS] = { ENC1_0_PIN }; //, ENC2_2_PIN, ENC2_0_PIN, ENC3_2_PIN, ENC3_0_PIN, ENC4_2_PIN, ENC4_0_PIN};
static const GPIO_TypeDef* encoderPort2[NUMBER_OF_ENCODERS] = { ENC1_0_PORT }; //, ENC2_2_PORT, ENC2_0_PORT, ENC3_2_PORT, ENC3_0_PORT, ENC4_2_PORT, ENC4_0_PORT};


//    buttons: ÿ( Up 2,Down 5,Left 3, Right 4,Enter 1, Shift 6,Back 0, Not used )

static const unsigned short buttonBit[NUMBER_OF_BUTTONS] = { BUTTON2_PIN, BUTTON5_PIN, BUTTON3_PIN, BUTTON4_PIN, BUTTON1_PIN, BUTTON6_PIN , BUTTON0_PIN }; //, BUTTON7_PIN};
static const GPIO_TypeDef* buttonPort[NUMBER_OF_BUTTONS] = { BUTTON2_PORT, BUTTON5_PORT, BUTTON3_PORT, BUTTON4_PORT, BUTTON1_PORT, BUTTON6_PORT , BUTTON0_PORT };//, BUTTON7_PORT};


void EncodersInit(void) {

	int k;
	for (k=0; k<NUMBER_OF_BUTTONS; k++) {
		palSetPadMode(buttonPort[k], buttonBit[k], PAL_MODE_INPUT_PULLUP);
	}
	for (k=0; k<(NUMBER_OF_ENCODERS); k++) {
		palSetPadMode( encoderPort1[k], encoderBit1[k], PAL_MODE_INPUT_PULLUP);
		palSetPadMode( encoderPort2[k], encoderBit2[k], PAL_MODE_INPUT_PULLUP);
	}

	//    int i,j;
	//    for (i=0; i<2; i++) {
	//        for (j=0; j<16; j++) {
	//            action[i][j] = actionToCopy[i][j];
	//       }
	//    }

	for (k=0; k<NUMBER_OF_ENCODERS; k++) {
//		encoderBit1[k] = encoderPins[k*2];//1 << (encoderPins[k*2] -1);
//		encoderBit2[k] = encoderPins[k*2+1];//1 << (encoderPins[k*2 + 1] -1);
//		encoderPort1[k] = encoderPorts[k*2];
//		encoderPort2[k] = encoderPorts[k*2+1];
		lastMove[k] = LAST_MOVE_NONE;
		tickSpeed[k] = 1;
	}

//		buttonBit = buttonPins;
//		buttonPort = buttonPorts;

	for (k=0; k<NUMBER_OF_BUTTONS; k++) {
		buttonPreviousState[k] = false;
		buttonTimer[k] = 16;		// > 15
//		buttonUsedFromSomethingElse[k] = false;
	}

	encoderTimer = 0;
	firstButtonDown = -1;
}

/*
void EncoderscheckSimpleStatus() {


	for (int k=0; k<NUMBER_OF_BUTTONS; k++) {
		// button is pressed ?
		bool_t b1 = (palReadPad(buttonPort[k], buttonBit[k]) == 0);//((registerBits & buttonBit[k]) == 0);
		// button is pressed
		if (b1) {
			if (buttonTimer[k] > 30) {
				buttonPressed(k);
			}
			buttonTimer[k]=0;
		}
		buttonTimer[k]++;
	}
}
*/

void EncoderscheckStatus(void) {

	int k;
	for (k=0; k<NUMBER_OF_ENCODERS; k++) {
		bool_t b1 = (palReadPad(encoderPort1[k], encoderBit1[k]) == 0);//((registerBits & encoderBit1[k]) == 0);
		bool_t b2 = (palReadPad(encoderPort2[k], encoderBit2[k]) == 0);//((registerBits & encoderBit2[k]) == 0);

		encoderState[k] <<= 2;
		encoderState[k] &= 0xf;
		if (b1) {
			encoderState[k] |= 1;
		}
		if (b2) {
			encoderState[k] |= 2;
		}

		if (actionEnc[encoderState[k]] == 1 && lastMove[k]!=LAST_MOVE_DEC) {
			EncodersencoderTurned(k, tickSpeed[k]);
			tickSpeed[k] +=3;
			lastMove[k] = LAST_MOVE_INC;
			timerAction[k] = 40;
		} else if (actionEnc[encoderState[k]] == 2 && lastMove[k]!=LAST_MOVE_INC) {
			EncodersencoderTurned(k, -tickSpeed[k]);
			tickSpeed[k] +=3;
			lastMove[k] = LAST_MOVE_DEC;
			timerAction[k] = 40;
		} else {
			if (timerAction[k] > 1) {
				timerAction[k] --;
			} else if (timerAction[k] == 1) {
				timerAction[k] --;
				lastMove[k] = LAST_MOVE_NONE;
			}
			if (tickSpeed[k] > 1 && ((encoderTimer & 0x1) == 0)) {
				tickSpeed[k] = tickSpeed[k] - 1;
			}
		}
		if (tickSpeed[k]>10) {
			tickSpeed[k] = 10;
		}
	}

	for (k=0; k<NUMBER_OF_BUTTONS; k++) {
		bool_t b1 = (palReadPad(buttonPort[k], buttonBit[k]) == 0);//((registerBits & buttonBit[k]) == 0);

		// button is pressed
		if (b1) {
			// just pressed ?
			if (!buttonPreviousState[k]) {
				if (buttonTimer[k] > 15) {
					EncodersbuttonPressed(k);
				}
			buttonTimer[k]=0;
			}
		} else {
			// Just unpressed ?
			if (buttonPreviousState[k]) {
				// Just released
				if (buttonTimer[k] > 15) {
					EncodersbuttonReleased(k);
				}
			buttonTimer[k]=0;
			}
		}
		buttonTimer[k]++;
		buttonPreviousState[k] = b1;
	}
	encoderTimer++;
}


void EncodersencoderTurned(int encoder, int ticks) {
    EncBuffer[encoder] += ticks;
	}

void EncodersbuttonPressed(int button) {
    Btn_Nav_Or.word |= (int32_t) 0x01 << button;
	}

void EncodersbuttonReleased(int button) {
    Btn_Nav_And.word &= ~((int32_t) 0x01 << button);
	}

