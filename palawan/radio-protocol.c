#include <string.h>

#include "osal.h"
#include "hal.h"
#include "spi.h"

#include "palawan.h"
#include "palawan-events.h"
#include "radio.h"
#include "radio-protocol.h"

static radio_handler_t default_handler;

radio_protocol_end();

void radioSetDefaultHandler(radio_handler_t handler) {
  default_handler = handler;
}

void radioPacketDispatch(uint8_t port, uint8_t src, uint8_t dst,
                         uint32_t len, const void *data) {

  const RadioProtocol *prot;
  bool handled = false;

  prot = radio_protocol_start();

  while (prot->handler) {
    if (prot->port == port) {
      if (!prot->handler(port, src, dst, len, data)) {
        handled = true;
        break;
      }
    }
    prot++;
  }

  /* If the packet wasn't handled, pass it to the default handler */
  if (!handled && default_handler)
    default_handler(port, src, dst, len, data);
}
