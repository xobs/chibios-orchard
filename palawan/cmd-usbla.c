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

#if 0
static void usb_phy_decode(BaseSequentialStream *chp, uint8_t *samples, int count) {
  int i;

  for (i = 1; i < count; i++) {
    if ((samples[i - 1] == 0) && samples[i] == 0)
      chprintf(chp, " 8<");
    else if (samples[i] == 0)
      chprintf(chp, "-");
    else if (samples[i] == 3)
      chprintf(chp, "+");
    else if ((samples[i - 1] == 1) && (samples[i] == 2))
      chprintf(chp, "0");
    else if ((samples[i - 1] == 2) && (samples[i] == 1))
      chprintf(chp, "0");
    else if ((samples[i - 1] == 2) && (samples[i] == 2))
      chprintf(chp, "1");
    else if ((samples[i - 1] == 1) && (samples[i] == 1))
      chprintf(chp, "1");
    else
      chprintf(chp, "?(%x)", samples[i]);
  }
}
#endif

void cmd_usbla(BaseSequentialStream *chp, int argc, char *argv[])
{
  int count;
  int i;
  int run;
  const uint8_t *samples;
  static char *states = "0KJ1";

  (void)argc;
  (void)argv;

  const uint8_t *usb_get_buffer_num(int num, int *sz);
  
  for (run = 0; run < 16; run++) {
    chprintf(chp, " run %02d: ", run);
    samples = usb_get_buffer_num(run, &count);

    for (i = 0; i < count; i++)
      chprintf(chp, "%c", states[samples[i]]);
    chprintf(chp, "\r\n");

    int diff = samples[0];
    chprintf(chp, "diffs: ");
    for (i = 1; i < count; i++) {
      chprintf(chp, " %d", (samples[i] - diff) & 0x3);
      diff = samples[i];
    }
    chprintf(chp, "\r\n");
 //     chprintf(chp, " %02x", samples[i]);
//    chprintf(chp, "           ");
//    usb_phy_decode(chp, samples, count);
//    chprintf(chp, "\r\n");
  }

  (void)argc;
  (void)argv;
}

palawan_command("usbla", cmd_usbla);
