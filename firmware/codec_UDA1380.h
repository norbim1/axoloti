/**
 * Copyright (C) 2013, 2014 Johannes Taelman
 *
 * This file is part of Axoloti.
 *
 * Axoloti is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * Axoloti is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * Axoloti. If not, see <http://www.gnu.org/licenses/>.
 *
 * Modified for UDA1380 board 2015 Norbert Marosi
 *
 */
#include "hal.h"

#define CODEC_UDA1380_I2S_ENABLE rccEnableSPI2(FALSE)
#define CODEC_UDA1380_I2S_DISABLE rccDisableSPI2(FALSE)
#define CODEC_UDA1380_I2S SPI2
#define CODEC_UDA1380_I2Sext I2S2ext

extern void codec_UDA1380_i2s_init(uint16_t sampleRate);
extern void codec_UDA1380_hw_init(uint16_t samplerate);
extern void codec_UDA1380_hw_reset(void);
extern void codec_UDA1380_Stop(void);

extern int codec_interrupt_timestamp;


//#define	LOUD				      			0x0
//#define	SOFT				      			0x0f
//#define DUMB				      			0x3f


#define UDA1380_WRITE_ADDRESS     0x30

#define UDA1380_REG_EVALCLK	      0x00
#define UDA1380_REG_I2S		      0x01
#define UDA1380_REG_PWRCTRL	      0x02
#define UDA1380_REG_ANAMIX	      0x03
#define UDA1380_REG_HEADAMP	      0x04
#define UDA1380_REG_MSTRVOL	      0x10
#define UDA1380_REG_MIXVOL	      0x11
#define UDA1380_REG_MODEBBT	      0x12
#define UDA1380_REG_MSTRMUTE      0x13
#define UDA1380_REG_MIXSDO	      0x14
#define UDA1380_REG_DECVOL	      0x20
#define UDA1380_REG_PGA		      0x21
#define UDA1380_REG_ADC		      0x22
#define UDA1380_REG_AGC		      0x23

#define UDA1380_REG_L3		      0x7f
#define UDA1380_REG_HEADPHONE     0x18
#define UDA1380_REG_DEC		      0x28

/*
uint8_t  UDA1380_Configuration(void)
{
	uint8_t dev_addr = UDA1380_WRITE_ADDRESS;
	uint8_t i=0;
	uint8_t errorcode;
	CODEC_I2C_Configuration();
	while (UDA1380InitData[i][0]!=0xff)
	{
		errorcode = CODEC_I2C_Write(dev_addr,3,UDA1380InitData[i]);
		if(!errorcode)
		{
			i++;
		}
		else
		{
			printf("I2c ERROR : 0x%x\r\n",errorcode);
			return ERROR;
		}
	}
	printf("UDA1380 Init OK!\r\n");
	return SUCCESS;
}
*/
