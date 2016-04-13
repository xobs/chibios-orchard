#include "hal.h"
#include "ext.h"
#include "palawan-events.h"

#include "usbphy.h"

extern void radioInterrupt(EXTDriver *extp, expchannel_t channel);

void usb_state_transition_cb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;
  //usbStateTransitionI();
}

static const EXTConfig ext_config = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, usb_state_transition_cb, PORTA, 4},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, radioInterrupt, PORTC, 4},
  }
};

void palawanEventsStart(void) {

  extStart(&EXTD1, &ext_config);
}
