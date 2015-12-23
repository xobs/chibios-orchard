
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
  volatile void *usbdpAddr;
  uint32_t usbdpMask;
  uint32_t usbdpShift;

  volatile void *usbdnAddr;
  uint32_t usbdnMask;
  uint32_t usbdnShift;

  uint32_t ticks;
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
  /* PTB0 */
  .usbdpAddr = &FGPIOB->PDIR,
  .usbdpMask = 0x01,
  .usbdpShift = 0,

  /* PTA4 */
  .usbdnAddr = &FGPIOA->PDIR,
  .usbdnMask = 0x10,
  .usbdnShift = 4,

  .ticks = 48000000 / USB_LS_RATE,
};

//void usbPhyISR(void) {
//  port_lock_from_isr();
void usbStateTransitionI(void) {

  int i;

  *((volatile uint32_t *)0xf80000cc) = 0x80;
//  PORTA->PCR[3] = PORTx_PCRn_MUX(1);

  if (usbPhyRead(&usbPhy, samples[run], TOTAL_SAMPLES) < 0)
    goto err;

  for (i = 0; i < TOTAL_SAMPLES; i++) {
    if (samples[run][i] != state_j) {
      run++;
      if (run >= TOTAL_RUNS)
        run--;
      break;
    }
  }

err:
//  PORTA->PCR[3] = PORTx_PCRn_MUX(7);
//  port_unlock_from_isr();
  return;
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
