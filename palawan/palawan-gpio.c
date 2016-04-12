
#include "osal.h"
#include "palawan.h"
#include "hal.h"

struct palawan_input {
  ioportid_t port;/* E.g. IOPORT1 (for PTAx), IOPORT2 (for PTBx), etc. */
  int gpionum;    /* 0 for PTx0, 1 for PTx1, etc. */
  int adcmux;     /* Mux number for ADC, or 0 if none */
  int adcchan;    /* ADC channel number (for SE operation) */
  int adchalf;    /* Set to 1 if adcchan is ADCxB, 0 if it's ADCxA */
  int capmux;     /* Mux number for captouch, or 0 if none */
  int capchan;    /* Capacitive channel */
};

static struct palawan_input inputs[] = {
  { /* PTA2, SW1 */
    .port = IOPORT1,
    .gpionum = 2,
  },
  { /* PTA19, SW2 */
    .port = IOPORT1,
    .gpionum = 19,
  },
  { /* PTC3, SW3 */
    .port = IOPORT3,
    .gpionum = 3,
  },
  { /* PTD4, SW4 */
    .port = IOPORT4,
    .gpionum = 4,
  },
  { /* PTC1, SW5, ADC1, CAP5 (also LLWU (ALT1), and I2S0_TXD0 (ALT6)) */
    .port = IOPORT3,
    .gpionum = 1,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 15,
    .adchalf = 0,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 14,
  },
  { /* PTC2, SW6, ADC2, CAP6 (also I2S0_TX_FS (ALT6)) */
    .port = IOPORT3,
    .gpionum = 2,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 11,
    .adchalf = 0,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 15,
  },
  { /* PTD5, SW7, ADC3 */
    .port = IOPORT4,
    .gpionum = 5,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 6, /* 6b, specifically */
    .adchalf = 1, /* 6b */
  },
  { /* PTD6, SW8, ADC4 (also LLWU (ALT1)) */
    .port = IOPORT4,
    .gpionum = 6,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 7, /* 7b, specifically */
    .adchalf = 1, /* 7b */
  },
  { /* PTE0, SW9, ADC5 (also UART1_TX (ALT3)) */
    .port = IOPORT5,
    .gpionum = 0,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adchalf = 0,
  },
  { /* PTE1, SW10, ADC6 (also UART1_RX (ALT3)) */
    .port = IOPORT5,
    .gpionum = 1,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adchalf = 0,
  },
  { /* PTE18, SW11, ADC7 */
    .port = IOPORT5,
    .gpionum = 18,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 2,
    .adchalf = 0,
  },
  { /* PTE19, SW12, ADC8 */
    .port = IOPORT5,
    .gpionum = 19,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 6, /* 6a, specifically */
    .adchalf = 0, /* 6a */
  },
  { /* PTA1, SW13, CAP1 */
    .port = IOPORT1,
    .gpionum = 1,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 2,
  },
  { /* PTB1, SW14, ADC9, CAP2 */
    .port = IOPORT2,
    .gpionum = 1,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 9,
    .adchalf = 0,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 6,
  },
  { /* PTB2, SW15, ADC10, CAP3 */
    .port = IOPORT2,
    .gpionum = 2,
    .adcmux = PAL_MODE_INPUT_ANALOG,
    .adcchan = 12,
    .adchalf = 0,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 7,
  },
  { /* PTB17, SW16, CAP4 */
    .port = IOPORT2,
    .gpionum = 17,
    .capmux = PAL_MODE_INPUT_ANALOG,
    .capchan = 10,
  },
};

int palawanGpioInit(void) {

  unsigned int i;

  /* Set PTB0 to pushpull */
  palSetPadMode(IOPORT2, 0, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPad(IOPORT2, 0);

  /* There are several options available:

     - Generic GPIO
     - ADC, with a resistor attached
     - Captouch
   */
  for (i = 0; i < ARRAY_SIZE(inputs); i++) {
    struct palawan_input *input = &inputs[i];

    /* Enable the pullup, to charge the line, then set it to floating */
    palSetPadMode(input->port, input->gpionum, PAL_MODE_INPUT_PULLUP);

    //palSetPadMode(input->port, input->gpionum, PAL_MODE_OUTPUT_PUSHPULL);
    //palSetPad(input->port, input->gpionum);

    //palSetPadMode(input->port, input->gpionum, PAL_MODE_INPUT);
  }

  return 0;
}


THD_WORKING_AREA(waThread, 128);
#include "chprintf.h"
static THD_FUNCTION(palawan_gpio_thread, arg) {

  (void)arg;
  unsigned int i;
  uint8_t samples[ARRAY_SIZE(inputs)];

  /* Pre-fetch the samples */
  for (i = 0; i < ARRAY_SIZE(inputs); i++) {
    struct palawan_input *input = &inputs[i];
    samples[i] = !!palReadPad(input->port, input->gpionum);
  }

  while (1) {

    for (i = 0; i < ARRAY_SIZE(inputs); i++) {
      struct palawan_input *input = &inputs[i];
      uint8_t val;

      palSetPadMode(input->port, input->gpionum, PAL_MODE_INPUT_PULLUP);
      palSetPadMode(input->port, input->gpionum, PAL_MODE_INPUT);
      val = !!palReadPad(input->port, input->gpionum);
      if (val != samples[i]) {
        chprintf(stream, "Port %d changed %d -> %d\r\n", i, samples[i], val);
        samples[i] = val;

        //palSetPadMode(input->port, input->gpionum, PAL_MODE_OUTPUT_PUSHPULL);
        //palSetPad(input->port, input->gpionum);
      }
    }
    chThdSleepMilliseconds(10);
  }
}

int palawanGpioStart(void) {

  chThdCreateStatic(waThread, sizeof(waThread),
                    43, palawan_gpio_thread, NULL);

  return 0;
}
