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
 */
#include "ch.h"
#include "hal.h"
#include "patch.h"
#include "sdcard.h"
#include "string.h"
#include "axoloti_board.h"

patchMeta_t patchMeta;

volatile int patchStatus;

void InitPatch0(void) {
  patchStatus = 2;
  patchMeta.fptr_patch_init = 0;
  patchMeta.fptr_patch_dispose = 0;
  patchMeta.fptr_dsp_process = 0;
  patchMeta.fptr_MidiInHandler = 0;
  patchMeta.fptr_applyPreset = 0;
  patchMeta.pPExch = NULL;
  patchMeta.numPEx = 0;
  patchMeta.pDisplayVector = 0;
  patchMeta.initpreset_size = 0;
  patchMeta.npresets = 0;
  patchMeta.npreset_entries = 0;
  patchMeta.pPresets = 0;
}

int dspLoadPct; // DSP load in percent
unsigned int CycleTime;
unsigned int DspTime;

static int32_t inbuf[32];
static int32_t *outbuf;

static WORKING_AREA(waThreadDSP, 7200) __attribute__ ((section (".ccmramend")));
static Thread *pThreadDSP = 0;
static msg_t ThreadDSP(void *arg) {
  (void)(arg);
#if CH_USE_REGISTRY
  chRegSetThreadName("dsp");
#endif
  while (1) {
    chEvtWaitOne((eventmask_t)1);
    static unsigned int tStart;
    CycleTime = RTT2US(hal_lld_get_counter_value() - tStart);
    tStart = hal_lld_get_counter_value();
    watchdog_feed();
    if (!patchStatus) { // running
#if ((BOARD_STM32F4DISCOVERY)||(BOARD_STM32F4DISCOVERY_1)||(BOARD_AXOLOTI_V03))
      // swap halfwords...
      int i;
      int32_t *p = inbuf;
      for (i = 0; i < 32; i++) {
        __ASM
        volatile ("ror %0, %1, #16" : "=r" (*p) : "r" (*p));
        p++;
      }
#endif
      (patchMeta.fptr_dsp_process)(inbuf, outbuf);
#if ((BOARD_STM32F4DISCOVERY)||(BOARD_STM32F4DISCOVERY_1)||(BOARD_AXOLOTI_V03))
      p = outbuf;
      for (i = 0; i < 32; i++) {
        __ASM
        volatile ("ror %0, %1, #16" : "=r" (*p) : "r" (*p));
        p++;
      }
#endif
    }
    else { // stopping or stopped
      patchStatus = 1;
      int i;
      for (i = 0; i < 32; i++) {
        outbuf[i] = 0;
      }
    }
    adc_convert();
    DspTime = RTT2US(hal_lld_get_counter_value() - tStart);
    dspLoadPct = (100 * DspTime) / CycleTime;
  }
  return (msg_t)0;
}

void StopPatch(void) {
  patchStatus = 2;
  while (pThreadDSP) {
    if (patchStatus == 1)
      break;
  }
  if (patchMeta.fptr_patch_dispose != 0)
    (patchMeta.fptr_patch_dispose)();
  UIGoSafe();
  InitPatch0();
}

void StartPatch(void) {
  KVP_ClearObjects();
  sdAttemptMountIfUnmounted();
  // reinit pin configuration for adc
  adc_configpads();
  patchMeta.fptr_dsp_process = 0;
  patchMeta.fptr_patch_init = (fptr_patch_init_t)(PATCHMAINLOC + 1);
  (patchMeta.fptr_patch_init)(GetFirmwareID());
  if (patchMeta.fptr_dsp_process == 0){
    // failed, incompatible firmwareID?
    return;
  }
  patchStatus = 0;
}

void start_dsp_thread(void){
  if (!pThreadDSP)
    pThreadDSP = chThdCreateStatic(waThreadDSP, sizeof(waThreadDSP), HIGHPRIO,
                                   ThreadDSP, NULL);
}

void computebufI(int32_t *inp, int32_t *outp) {
  int i;
  for (i = 0; i < 32; i++) {
    inbuf[i] = inp[i];
  }
  outbuf = outp;
  if (pThreadDSP) {
    chSysLockFromIsr()
    ;
    chEvtSignalI(pThreadDSP, (eventmask_t)1);
    chSysUnlockFromIsr();
  }
  else
    for (i = 0; i < 32; i++) {
      outp[i] = 0;
    }
}

void MidiInMsgHandler(uint8_t status, uint8_t data1, uint8_t data2) {
  if (!patchStatus) {
    (patchMeta.fptr_MidiInHandler)(status, data1, data2);
  }
}

// Thread to load a new patch from within a patch

static char loadFName[16];
static WORKING_AREA(waThreadLoader, 1024);
static Thread *pThreadLoader;
static msg_t ThreadLoader(void *arg) {
  (void)arg;
#if CH_USE_REGISTRY
  chRegSetThreadName("loader");
#endif
  while (1) {
    chEvtWaitOne((eventmask_t)1);
//    StopPatch();
    SDLoadPatch(loadFName);
  }
  return (msg_t)0;
}

void StartLoadPatchTread(void) {
  pThreadLoader = chThdCreateStatic(waThreadLoader, sizeof(waThreadLoader),
                                    NORMALPRIO, ThreadLoader, NULL);
}

void LoadPatch(char *name) {
  strcpy(loadFName, name);
  chEvtSignal(pThreadLoader, (eventmask_t)1);
}
