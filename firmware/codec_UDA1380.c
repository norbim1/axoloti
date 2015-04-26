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
#include "codec_UDA1380.h"
#include "ch.h"
#include "hal.h"

#include "codec.h"
#include "stm32f4xx.h"
#include "axoloti_board.h"

#define STM_IS_I2S_MASTER true

extern void computebufI(int32_t *inp, int32_t *outp);

const stm32_dma_stream_t* i2sdma_UDA1380;
const stm32_dma_stream_t* i2sdma_UDA1380rx;

int codec_interrupt_timestamp;

uint8_t UDA1380InitData[][3] =
{
	/*
	 *Enable all power for now
	 */
	{UDA1380_REG_PWRCTRL,     0xA5, 0x1F},

	/*
	 *CODEC ADC and DAC clock from WSPLL, all clocks enabled
	 */
	{UDA1380_REG_EVALCLK,     0x0F, 0x02},

	/*
	 *I2S bus data I/O formats, use digital mixer for output
	 *BCKO is slave
	 */
	{UDA1380_REG_I2S,         0x00, 0x00},

	/*
	 *Zero mixer analog input gain
	 */
	{UDA1380_REG_ANAMIX,      0x3F, 0x3F},

	/*
	 *Enable headphone short circuit protection
	 */
	{UDA1380_REG_HEADAMP,     0x02, 0x02},

	/*
	 *Full master volume
	 */
	{UDA1380_REG_MSTRVOL,     0x00, 0x00},

	/*
	 *Enable full mixer volume on both channels
	 */
	{UDA1380_REG_MIXVOL,      0x00, 0x00},

	/*
	 *Bass and treble boost set to flat
	 */
	{UDA1380_REG_MODEBBT,     0x00, 0x00},

	/*
	 *Disable mute and de-emphasis
	 */
	{UDA1380_REG_MSTRMUTE,    0x00, 0x00},

	/*
	 *Mixer off, other settings off
	 */
	{UDA1380_REG_MIXSDO,      0x00, 0x00},

	/*
	 *ADC decimator volume to max
	 */
	{UDA1380_REG_DECVOL,      0x00, 0x00},

	/*
	 *No PGA mute, full gain
	 */
	{UDA1380_REG_PGA,         0x00, 0x00},

	/*
	 *Select line in and MIC, max MIC gain
	 */
	{UDA1380_REG_ADC,         0x00, 0x02},

	/*
	 *AGC
	 */
	{UDA1380_REG_AGC,         0x00, 0x00},

	/*
	 *Disable clocks to save power
     *{UDA1380_REG_EVALCLK,     0x00, 0x32},
     *disable power to input to save power
     *{UDA1380_REG_PWRCTRL,     0xA5, 0xC0},
	 */

    /*
     *End of list
	 */
	{0xFF,                    0xFF, 0xFF}
};

#define I2S2_TX_DMA_CHANNEL \
STM32_DMA_GETCHANNEL(STM32_SPI_SPI2_TX_DMA_STREAM /* STM32_DMA_STREAM_ID(1, 4) */, \
STM32_SPI2_TX_DMA_CHN /* 0000 */)

#define I2S2ext_RX_DMA_CHANNEL \
STM32_DMA_GETCHANNEL(STM32_DMA_STREAM_ID(1, 3), \
3)

static const SPIConfig spi1c_cfg = {NULL, /* HW dependent part.*/GPIOE, 8,
                                    SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2
                                        | SPI_CR1_CPOL | SPI_CR1_CPHA };


/* I2C interface #2 */
/* SDA : PB11
 * SCL : PB10
 */
static const I2CConfig i2cfg2 = {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2, };

static uint8_t i2crxbuf[8];
static uint8_t i2ctxbuf[8];
static systime_t tmo;

#define UDA1380_I2C_ADDR (0x30>>1)

void CheckI2CErrors(void) {
  volatile i2cflags_t errors;
  errors = i2cGetErrors(&I2CD2);
  (void)errors;
}

void UDA1380_I2CStart(void) {
  palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(4)| PAL_STM32_OTYPE_OPENDRAIN);
  palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(4)| PAL_STM32_OTYPE_OPENDRAIN);
  chMtxLock(&Mutex_DMAStream_1_7);
  i2cStart(&I2CD2, &i2cfg2);
}

void UDA1380_I2CStop(void) {
  i2cStop(&I2CD2);
  chMtxUnlock();
}

uint16_t UDA1380_ReadRegister(uint8_t RegisterAddr) {
  msg_t status;
  i2ctxbuf[0] = RegisterAddr;
    UDA1380_I2CStart();
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, UDA1380_I2C_ADDR, i2ctxbuf, 1,
                                    i2crxbuf, 2, tmo);
  if (status != RDY_OK) {
    CheckI2CErrors();
  }
  i2cReleaseBus(&I2CD2);
  UDA1380_I2CStop();
  chThdSleepMilliseconds(1);
  return (uint16_t) *i2crxbuf;
}

void UDA1380_WriteRegister(uint8_t RegisterAddr, uint16_t RegisterValue) {
  msg_t status;
  i2ctxbuf[0] = RegisterAddr;
  i2ctxbuf[1] = (uint8_t) RegisterValue>>8;
  i2ctxbuf[2] = (uint8_t) (RegisterValue & 0x00FF);

  UDA1380_I2CStart();
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, UDA1380_I2C_ADDR, i2ctxbuf, 3,
                                    i2crxbuf, 0, tmo);
  if (status != RDY_OK) {
    CheckI2CErrors();
    status = i2cMasterTransmitTimeout(&I2CD2, UDA1380_I2C_ADDR, i2ctxbuf, 3,
                                      i2crxbuf, 0, tmo);
    chThdSleepMilliseconds(1);
  }
  i2cReleaseBus(&I2CD2);
  UDA1380_I2CStop();
  chThdSleepMilliseconds(1);

/*
  uint16_t rd = UDA1380_ReadRegister(RegisterAddr);
  if (rd != RegisterValue) {
//    while(1){}
    palSetPad(GPIOA, 8);
  }
  chThdSleepMilliseconds(1);
*/
}

void UDA1380_WriteRegister2(uint8_t RegisterAddr, uint8_t RegisterValueH, uint8_t RegisterValueL) {
  msg_t status;
  i2ctxbuf[0] = RegisterAddr;
  i2ctxbuf[1] = RegisterValueH;
  i2ctxbuf[2] = RegisterValueL;

  UDA1380_I2CStart();
  i2cAcquireBus(&I2CD2);
  status = i2cMasterTransmitTimeout(&I2CD2, UDA1380_I2C_ADDR, i2ctxbuf, 3,
                                    i2crxbuf, 0, tmo);
  if (status != RDY_OK) {
    CheckI2CErrors();
    status = i2cMasterTransmitTimeout(&I2CD2, UDA1380_I2C_ADDR, i2ctxbuf, 3,
                                      i2crxbuf, 0, tmo);
    chThdSleepMilliseconds(1);
  }
  i2cReleaseBus(&I2CD2);
  UDA1380_I2CStop();
  chThdSleepMilliseconds(1);


}

void codec_UDA1380_hw_init(uint16_t samplerate) {

  palSetPadMode(GPIOD, 4, PAL_MODE_OUTPUT_PUSHPULL);

  palClearPad(GPIOD, 4);  //keep onboard codec (CS43L22) pins floating
  chThdSleepMilliseconds(10);

  palSetPadMode(GPIOD, 15, PAL_MODE_OUTPUT_PUSHPULL); //blue LED flashes

  int k;
  for (k = 0; k < 3; k++) {
    palSetPad(GPIOD, 15);
    chThdSleepMilliseconds(150);
    palClearPad(GPIOD, 15);
    chThdSleepMilliseconds(150);
  }

  tmo = MS2ST(4);
  chThdSleepMilliseconds(5);
  /*
   * 1. Power down the PLL.
   * 2. Reset the PLL control register.
   * 3. Start the PLL.
   * 4. Poll the lock bit.
   * 5. Assert the core clock enable bit after the PLL lock is acquired.
   */

  while (1) {
#ifdef STM_IS_I2S_MASTER
//    UDA1380_WriteRegister(UDA1380_REG_R0_CLKC, 0x01); // 256FS
    chThdSleepMilliseconds(10);
	uint8_t i=0;

	while (UDA1380InitData[i][0]!=0xff) {
		UDA1380_WriteRegister2(UDA1380InitData[i][0],UDA1380InitData[i][1],UDA1380InitData[i][2]);
		i++;
	}
#endif

    chThdSleepMilliseconds(100);

    break;
  }


  chThdSleepMilliseconds(10);

  /*
   i2cStop(&I2CD2);
   i2cStart(&I2CD2, &i2cfg2);
   UDA1380_WriteRegister(0x4000, 0x8); // 1024FS
   rd = UDA1380_ReadRegister(0x4000);
   if (rd != 0x08){
   while(1){};
   }

   i2cStop(&I2CD2);
   i2cStart(&I2CD2, &i2cfg2);


   // power down PLL
   uint8_t R1[6];
   R1[0]=0;R1[1]=0;R1[2]=0;
   R1[3]=0;R1[4]=0;R1[5]=0;
   UDA1380_WriteRegister6(UDA1380_REG_R1_PLLC,&R1[0]);

   i2cStop(&I2CD2);
   i2cStart(&I2CD2, &i2cfg2);


   // Integer PLL Parameter Settings for fS = 48 kHz
   // (PLL Output = 49.152 MHz = 1024 � fS)
   R1[4] = 0x20;
   R1[5] = 0x01;
   R1[1] = 0x20;
   R1[0] = 0x01;
   UDA1380_WriteRegister6(UDA1380_REG_R1_PLLC,&R1[0]);
   // poll lock bit
   i2cStop(&I2CD2);
   i2cStart(&I2CD2, &i2cfg2);


   UDA1380_WriteRegister(0x4000, 0xE); // 1024FS
   rd = UDA1380_ReadRegister(0x4000);
   if (rd != 0xE){
   while(1){};
   }

   i2cStop(&I2CD2);
   i2cStart(&I2CD2, &i2cfg2);

   while(1){
   UDA1380_ReadRegister6(UDA1380_REG_R1_PLLC);
   if (i2crxbuf[5] & 0x02) break;
   chThdSleepMilliseconds(5);
   }
   // mclk = 12.319MHz
   UDA1380_WriteRegister(0x4000, 0xE); // 1024FS
   rd = UDA1380_ReadRegister(0x4000);
   if (rd != 0xE){
   while(1){};
   }
   */

}

static void dma_i2s_interrupt(void* dat, uint32_t flags) {
  (void)dat;
  (void)flags;
  codec_interrupt_timestamp = hal_lld_get_counter_value();
  if ((i2sdma_UDA1380)->stream->CR & STM32_DMA_CR_CT ) {
    computebufI(rbuf, buf);
  }
  else {
    computebufI(rbuf2, buf2);
  }
  dmaStreamClearInterrupt(i2sdma_UDA1380);
}

static void dma_i2s_rxinterrupt(void* dat, uint32_t flags) {
  (void)dat;
  (void)flags;
  dmaStreamClearInterrupt(i2sdma_UDA1380rx);
}

static void codec_UDA1380_dma_init(void) {
  // TX
  i2sdma_UDA1380 = STM32_DMA_STREAM(STM32_SPI_SPI2_TX_DMA_STREAM);

  uint32_t i2stxdmamode = STM32_DMA_CR_CHSEL(I2S2_TX_DMA_CHANNEL) |
  STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
  STM32_DMA_CR_DIR_M2P |
  STM32_DMA_CR_TEIE |
  STM32_DMA_CR_TCIE |
  STM32_DMA_CR_DBM | // double buffer mode
      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_WORD;

      bool_t b = dmaStreamAllocate(i2sdma_UDA1380,
          STM32_SPI_SPI2_IRQ_PRIORITY,
          (stm32_dmaisr_t)dma_i2s_interrupt,
          (void *)&SPID2);

//  if (!b)
//  chprintf((BaseChannel*)&SD2, "DMA Allocated Successfully to I2S2\r\n");

      dmaStreamSetPeripheral(i2sdma_UDA1380, &(CODEC_UDA1380_I2S->DR));
// my double buffer test
      dmaStreamSetMemory0(i2sdma_UDA1380, buf);
      dmaStreamSetMemory1(i2sdma_UDA1380, buf2);
      dmaStreamSetTransactionSize(i2sdma_UDA1380, 64);
      dmaStreamSetMode(i2sdma_UDA1380, i2stxdmamode | STM32_DMA_CR_MINC);
//  dmaStreamSetFIFO(i2sdma,

      // RX
#if 1
      i2sdma_UDA1380rx = STM32_DMA_STREAM(STM32_SPI_SPI2_RX_DMA_STREAM);

      uint32_t i2srxdmamode = STM32_DMA_CR_CHSEL(3/*I2S2_RX_DMA_CHANNEL*/) |
      STM32_DMA_CR_PL(STM32_SPI_SPI2_DMA_PRIORITY) |
      STM32_DMA_CR_DIR_P2M |
//  STM32_DMA_CR_DMEIE |
      STM32_DMA_CR_TEIE |
      STM32_DMA_CR_TCIE |
      STM32_DMA_CR_DBM |// double buffer mode
      STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_WORD;

      b = dmaStreamAllocate(i2sdma_UDA1380rx,
          STM32_SPI_SPI2_IRQ_PRIORITY,
          (stm32_dmaisr_t)dma_i2s_rxinterrupt,
          (void *)&SPID2);

      while(b) {
        // failed
      }
//  if (!b)
//  chprintf((BaseChannel*)&SD2, "DMA Allocated Successfully to I2S2\r\n");

//  dmaStreamSetPeripheral(i2sdma_UDA1380rx, &(CODEC_UDA1380_I2Sext->DR));
      dmaStreamSetPeripheral(i2sdma_UDA1380rx, &(CODEC_UDA1380_I2Sext->DR));
// my double buffer test
      dmaStreamSetMemory0(i2sdma_UDA1380rx, rbuf2);
      dmaStreamSetMemory1(i2sdma_UDA1380rx, rbuf);
      dmaStreamSetTransactionSize(i2sdma_UDA1380rx, 64);//PLAYBACK_BUFFER_SIZE);
      dmaStreamSetMode(i2sdma_UDA1380rx, i2srxdmamode | STM32_DMA_CR_MINC);

      dmaStreamClearInterrupt(i2sdma_UDA1380rx);
      dmaStreamEnable(i2sdma_UDA1380rx);
#endif
      // enable
      dmaStreamClearInterrupt(i2sdma_UDA1380);
      dmaStreamEnable(i2sdma_UDA1380);
    }

void codec_UDA1380_i2s_init(uint16_t sampleRate) {

#if 0
  /* CODEC_I2S output pins configuration: WS, SCK SD0 and SDI pins ------------------*/
  GPIO_InitStructure.GPIO_Pin = CODEC_I2S_SCK_PIN | CODEC_I2S_SDO_PIN | CODEC_I2S_SDI_PIN | CODEC_I2S_WS_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(CODEC_I2S_GPIO, &GPIO_InitStructure);

  /* CODEC_I2S pins configuration: MCK pin */
  GPIO_InitStructure.GPIO_Pin = CODEC_I2S_MCK_PIN;
  GPIO_Init(CODEC_I2S_MCK_GPIO, &GPIO_InitStructure);

  /* Connect pins to I2S peripheral  */
  GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_WS_PINSRC, CODEC_I2S_GPIO_AF);
  GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SCK_PINSRC, CODEC_I2S_GPIO_AF);
  GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SDO_PINSRC, CODEC_I2S_GPIO_AF);
  GPIO_PinAFConfig(CODEC_I2S_GPIO, CODEC_I2S_SDI_PINSRC, CODEC_I2S_GPIO_AF);
  GPIO_PinAFConfig(CODEC_I2S_MCK_GPIO, CODEC_I2S_MCK_PINSRC, CODEC_I2S_GPIO_AF);
#endif
  /*
   palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL|PAL_MODE_ALTERNATE(5));
   palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_PUSHPULL|PAL_MODE_ALTERNATE(5));
   palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));
   palSetPadMode(GPIOB, 15, PAL_MODE_OUTPUT_PUSHPULL|PAL_MODE_ALTERNATE(5));
   palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_PUSHPULL|PAL_MODE_ALTERNATE(5));
   */

#if 1
  palSetPadMode(GPIOB, 12, PAL_MODE_ALTERNATE(5));
  // i2s2ws
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5));
  // i2s2ck
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));
  // i2s2_ext_sd
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5));
  // i2s2_sd
#else // test if codec is connected
  palSetPadMode(GPIOB, 12, PAL_MODE_INPUT);// i2s2ws
  palSetPadMode(GPIOB, 13, PAL_MODE_INPUT);// i2s2ck
  palSetPadMode(GPIOB, 14, PAL_MODE_INPUT);// i2s2_ext_sd
  palSetPadMode(GPIOB, 15, PAL_MODE_INPUT);// i2s2_sd
#endif

  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(5));
  // i2s2_mck

// SPI2 in I2S Mode, Master
  CODEC_UDA1380_I2S_ENABLE
  ;

#ifdef STM_IS_I2S_MASTER
  CODEC_UDA1380_I2S ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1 | SPI_I2SCFGR_DATLEN_1; /* MASTER TRANSMIT */

  uint16_t prescale;
  uint32_t pllfreq = STM32_PLLI2SVCO / STM32_PLLI2SR_VALUE;  //TODO Must set according to the datasheet!
  // Master clock mode Fs * 256
  prescale = (pllfreq * 10) / (256 * sampleRate) + 5;
  prescale /= 10;

  if (prescale > 0xFF || prescale < 2)
  prescale = 2;

  if (prescale & 0x01)
  CODEC_UDA1380_I2S ->I2SPR = SPI_I2SPR_MCKOE | SPI_I2SPR_ODD | (prescale >> 1);
  else
  CODEC_UDA1380_I2S ->I2SPR = SPI_I2SPR_MCKOE | (prescale >> 1);

  CODEC_UDA1380_I2Sext ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_0 | SPI_I2SCFGR_DATLEN_1; /* SLAVE RECEIVE*/
  CODEC_UDA1380_I2Sext ->I2SPR = 0x0002;

#else
  CODEC_UDA1380_I2S ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_DATLEN_1; /* SLAVE TRANSMIT, 32bit */

  // generate 8MHz clock on MCK pin with PWM...
  static const PWMConfig pwmcfg = {168000000, /* 400kHz PWM clock frequency.  */
                                   21, /* PWM period is 128 cycles.    */
                                   NULL, { {PWM_OUTPUT_ACTIVE_HIGH, NULL}, {
                                       PWM_OUTPUT_ACTIVE_HIGH, NULL},
                                          {PWM_OUTPUT_ACTIVE_HIGH, NULL}, {
                                              PWM_OUTPUT_ACTIVE_HIGH, NULL}},
                                   /* HW dependent part.*/
                                   0,
                                   0};
  palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(3));
  // i2s2_mck
  pwmStart(&PWMD8, &pwmcfg);
  pwmEnableChannel(&PWMD8, 0, 10);

  CODEC_UDA1380_I2Sext ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_0
      | SPI_I2SCFGR_DATLEN_1; /* SLAVE RECEIVE, 32bit*/
  CODEC_UDA1380_I2Sext ->I2SPR = 0x0002;

#endif
//  CODEC_UDA1380_I2S ->I2SPR = SPI_I2SPR_MCKOE |

//// FULL DUPLEX CONFIG

  ;

  codec_UDA1380_dma_init();

// Enable I2S DMA Request
  CODEC_UDA1380_I2S ->CR2 = SPI_CR2_TXDMAEN;  //|SPI_CR2_RXDMAEN;
//  CODEC_UDA1380_I2S ->CR2 = SPI_CR2_RXNEIE;
//  CODEC_UDA1380_I2S ->CR2 = SPI_CR2_TXEIE;

  CODEC_UDA1380_I2Sext ->CR2 = SPI_CR2_RXDMAEN;
//  CODEC_UDA1380_I2S ->CR2 = SPI_CR2_TXDMAEN;

// Now Enable I2S
  CODEC_UDA1380_I2S ->I2SCFGR |= SPI_I2SCFGR_I2SE;
  CODEC_UDA1380_I2Sext ->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void codec_UDA1380_Stop(void) {
  CODEC_UDA1380_I2S ->I2SCFGR = 0;
  CODEC_UDA1380_I2Sext ->I2SCFGR = 0;
  CODEC_UDA1380_I2S ->CR2 = 0;
  CODEC_UDA1380_I2Sext ->CR2 = 0;

}
