/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "palawan-shell.h"
#include "usbphy.h"

extern void cmd_usbmac(BaseSequentialStream *chp, int argc, char *argv[]);

void cmd_usbcap(BaseSequentialStream *chp, int argc, char *argv[])
{
  uint8_t samples[11];

  chprintf(chp, "Waiting for sample...");
  chSysLock();
  while (!(FGPIOD->PDIR & (1 << 5))) {
    ;
  }
  usbCapture(samples);
  chSysUnlock();
  chprintf(chp, " Done.\r\n");

  cmd_usbmac(chp, argc, argv);
}

palawan_command("usbcap", cmd_usbcap);
