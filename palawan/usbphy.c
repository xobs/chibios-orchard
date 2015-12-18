
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "palawan.h"

#define TOTAL_SAMPLES 64
#define TOTAL_RUNS 128
#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */
static uint8_t samples[TOTAL_RUNS][TOTAL_SAMPLES];

enum state {
  state_se0,
  state_k,
  state_j,
  state_se1,
};

static int led_on = 1;
static int run = 0;

/* GPIOB (PTB0) is wiggling a lot */

void usbStateTransitionI(void) {
  int i;

  usbPhyRead(samples[run], TOTAL_SAMPLES);

  if (led_on)
    palSetPad(GPIOD, 7);
  else
    palClearPad(GPIOD, 7);
  led_on = !led_on;

  for (i = 0; i < TOTAL_SAMPLES; i++) {
    if (samples[run][i] != state_j) {
      run++;
      if (run >= TOTAL_RUNS)
        run = 0;
      break;
    }
  }
}

uint8_t *usbGetSamples(int *count) {

  if (count)
    *count = TOTAL_SAMPLES;

  return samples[run];
}

uint8_t *usbGetSampleRun(int *count, int thisrun) {
  if (count)
    *count = TOTAL_SAMPLES;
  return samples[thisrun];
}

int usbGetRun(void) {
  return run;
}

int usbMaxRuns(void) {
  return TOTAL_RUNS;
}
