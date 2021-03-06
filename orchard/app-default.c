#include "orchard-app.h"
#include "led.h"

static uint32_t led_init(OrchardAppContext *context) {

  (void)context;
  chprintf(stream, "LED: Initializing led app\r\n");
  return 0;
}

static void led_start(OrchardAppContext *context) {
  
  (void)context;
  chprintf(stream, "LED: Starting led app\r\n");

  listEffects();
  
  effectsSetPattern(0);  // pick default pattern

}

void led_event(OrchardAppContext *context, const OrchardAppEvent *event) {

  (void)context;
  uint8_t shift;
  
  chprintf(stream, "LED: Received %d event\r\n", event->type);

  if (event->type == keyEvent) {
    if ((event->key.code == keyLeft)  && (event->key.flags == keyDown) ) {
      shift = getShift();
      shift++;
      if (shift > 6)
        shift = 0;
      setShift(shift);
    }
    else if ( (event->key.code == keyRight) && (event->key.flags == keyDown))
      effectsNextPattern();
  }
}

static void led_exit(OrchardAppContext *context) {

  (void)context;
  chprintf(stream, "LED: Exiting led app\r\n");
}

orchard_app("Blinkies!", led_init, led_start, led_event, led_exit);


