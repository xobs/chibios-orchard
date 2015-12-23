
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "palawan.h"

#define TOTAL_SAMPLES 128
#define TOTAL_RUNS 32
#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */
static uint8_t samples[TOTAL_RUNS][TOTAL_SAMPLES];

struct USBPHY {
  uint32_t gpioBase;
  uint32_t usbdpOffset;
  uint32_t usbdpMask;
  uint32_t usbdnOffset;
  uint32_t usbdnMask;
  uint32_t ticks;
  uint32_t usbdpShift;
  uint32_t usbdnShift;
} __attribute__((packed));

enum state {
  state_se0,
  state_k,
  state_j,
  state_se1,
};

static int led_on = 1;
static int run = 0;

/* GPIOB (PTB0) is wiggling a lot */

static struct USBPHY usbPhy = {
  .gpioBase = 0xF8000000,
  .usbdpOffset = 0x50,
  .usbdpMask = 0x01,
  .usbdnOffset = 0x10,
  .usbdnMask = 0x10,
  .ticks = 48000000 / USB_LS_RATE,
  .usbdpShift = 0x0,
  .usbdnShift = 0x3,
};

void usbStateTransitionI(void) {
  int i;

  *((uint32_t *)0xf80000cc) = 0x80;
  usbPhyRead(&usbPhy, samples[run], TOTAL_SAMPLES);

  /*
  if (led_on)
    palSetPad(GPIOD, 7);
  else
    palClearPad(GPIOD, 7);
  led_on = !led_on;
  */

  for (i = 0; i < TOTAL_SAMPLES; i++) {
    if (samples[run][i] != state_j) {
      run++;
      if (run >= TOTAL_RUNS)
        run--;
      break;
    }
  }
}

void usbResetRun(void) {
  run = 0;
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
