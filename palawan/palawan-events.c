#include "ch.h"
#include "hal.h"
#include "ext.h"
#include "palawan-events.h"

#include "usbphy.h"

#if 0
event_source_t ble_rdy;
event_source_t rf_pkt_rdy;
event_source_t gpiox_rdy;

event_source_t celcius_rdy;
event_source_t mic_rdy;
event_source_t usbdet_rdy;

event_source_t radio_page;
event_source_t radio_sex;
event_source_t radio_app;

static void ble_rdyn_cb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;

  chSysLockFromISR();
  chEvtBroadcastI(&ble_rdy);
  chSysUnlockFromISR();
}
#endif

void usb_state_transition_cb(EXTDriver *extp, expchannel_t channel) {

  (void)extp;
  (void)channel;
  //usbStateTransitionI();
}

static const EXTConfig ext_config = {
  {
    {EXT_CH_MODE_BOTH_EDGES | EXT_CH_MODE_AUTOSTART, usb_state_transition_cb, PORTA, 4},
  }
};

void palawanEventsStart(void) {

  extStart(&EXTD1, &ext_config);
}
