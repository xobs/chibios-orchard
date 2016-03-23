#if 0
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
#include "usbmac.h"

extern void cmd_usbmac_print(BaseSequentialStream *chp, struct USBMAC *mac);

static struct USBPHY test_phy = {
  /* Pins J21 and J19 */
  /* PTD5 */
  .usbdpIAddr = &FGPIOD->PDIR,
  .usbdpSAddr = &FGPIOD->PSOR,
  .usbdpCAddr = &FGPIOD->PCOR,
  .usbdpDAddr = &FGPIOD->PDDR,
  .usbdpMask  = (1 << 5),
  .usbdpShift = 5,

  /* PTD6 */
  .usbdnIAddr = &FGPIOD->PDIR,
  .usbdnSAddr = &FGPIOD->PSOR,
  .usbdnCAddr = &FGPIOD->PCOR,
  .usbdnDAddr = &FGPIOD->PDDR,
  .usbdnMask  = (1 << 6),
  .usbdnShift = 6,
};

static struct USBMAC test_mac;

void cmd_usbcap(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argc;
  (void)argv;

  if (!usbPhyInitialized(&test_phy)) {
    usbMacInit(&test_mac);
    usbPhyInit(&test_phy, &test_mac);
  }

  chprintf(chp, "Waiting for sample...");
  chSysLock();
  while (!(FGPIOD->PDIR & (1 << 5))) {
    ;
  }
  usbCapture(&test_phy);
  chSysUnlock();
  chprintf(chp, " Done.\r\n");

  cmd_usbmac_print(chp, &test_mac);
}

palawan_command("usbcap", cmd_usbcap);
#endif
