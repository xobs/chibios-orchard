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
#include "hal.h"
#include "shell.h"
#include "chprintf.h"

#include <strings.h>
#include <string.h>
#include <stdlib.h>

#include "palawan.h"
#include "palawan-shell.h"

#include "radio.h"
#include "radio-protocol.h"

#define MAX_CLIENTS 16

static struct {
  long long uid;
  uint32_t age;
} dhcp_entries[MAX_CLIENTS];

struct dhcp_request {
  uint8_t addr;
  long long uid;
} __attribute__((packed));

static int dhcp_server_running = 0;
static int dhcp_requesting = 0;

static long long get_uid(void) {
  long long uidh = *((uint32_t *)0x40048058);
  long long uidm = *((uint32_t *)0x4004805c);
  long long uidl = *((uint32_t *)0x40048060);
  long long uid;

  uid = (uidh << 48) | (uidm << 16) | (uidl >> 16);
  return uid;
}

static int prot_dhcp(uint8_t port,
                     uint8_t src,
                     uint8_t dst,
                     uint8_t length,
                     const void *data) {
  (void)port;
  (void)dst;
  (void)length;
  const struct dhcp_request *req = data;

  /* Server code */
  if (dhcp_server_running) {

    struct dhcp_request response;
    int addr = -1;
    int empty_offset = -1;
    int oldest_offset = -1;
    uint32_t i;
    systime_t oldest_time = chVTGetSystemTime();

    for (i = 0; i < ARRAY_SIZE(dhcp_entries); i++) {

      /* If the address exists in our table already, return that address */
      if (req->uid == dhcp_entries[i].uid) {
        dhcp_entries[i].age = chVTGetSystemTime();
        addr = i + 1;
        break;
      }

      /* If this entry is blank, save it in case this is a new address */
      if (dhcp_entries[i].uid == 0)
        empty_offset = i;

      /* Otherwise, mark this address for eviction */
      else if (dhcp_entries[i].age < oldest_time) {
        oldest_offset = i;
        oldest_time = dhcp_entries[i].age;
      }
    }

    /* Fill in using an empty slot */
    if ((addr == -1) && (empty_offset > 0)) {
      dhcp_entries[empty_offset].age = chVTGetSystemTime();
      dhcp_entries[empty_offset].uid = req->uid;
      addr = empty_offset + 1;
    }

    /* Evict an address */
    if (addr == -1) {
      dhcp_entries[oldest_offset].age = chVTGetSystemTime();
      dhcp_entries[oldest_offset].uid = req->uid;
      addr = oldest_offset + 1;
    }

    response.uid = req->uid;
    response.addr = addr;

    radioSend(radioDriver, src, 3, sizeof(response), &response);
  }

  /* Client code (e.g. we got a response) */
  else if (req->uid == get_uid()) {

    radioSetAddress(radioDriver, req->addr);
    dhcp_requesting = 0;
  }

  return 0;
}

radio_protocol("DHCP", 3, prot_dhcp);

void dhcpServerStart(void) {

  dhcp_server_running = 1;
}

int dhcpRequestAddress(int timeout_ms) {

  struct dhcp_request request;
  int ms = 0;

  request.uid = get_uid();
  request.addr = 0;

  dhcp_requesting = 1;
  radioSend(radioDriver, 0xff, 3, sizeof(request), &request);

  while (dhcp_requesting && (ms < timeout_ms)) {
    chThdSleepMilliseconds(1);
    ms++;
  }

  if (dhcp_requesting)
    ms = -1;
  dhcp_requesting = 0;

  return ms;
}

static void cmd_dhcp(BaseSequentialStream *chp, int argc, char *argv[]) {

  (void)argc;
  (void)argv;

  if (dhcp_server_running) {

    uint32_t i;

    chprintf(chp, "DHCP table:\r\n");

    chprintf(chp, "     Address   Lease updated   UID\r\n");
    for (i = 0; i < ARRAY_SIZE(dhcp_entries); i++) {
      if (dhcp_entries[i].uid)
        chprintf(chp, "\t%3d:      %10d   %08x-%08x\r\n",
                 i,
                 dhcp_entries[i].age,
                 dhcp_entries[i].uid >> 32,
                 dhcp_entries[i].uid);
      else
        chprintf(chp, "\t%3d:       available\r\n", i);
    }
  }
  else {

    chprintf(chp, "Requesting address... ");

    int ms = dhcpRequestAddress(100);

    if (ms < 0)
      chprintf(chp, "timeout\r\n");
    else
      chprintf(chp, "%d ms (%d)\r\n", ms, radioAddress(radioDriver));
  }
}
palawan_command("dhcp", cmd_dhcp);
