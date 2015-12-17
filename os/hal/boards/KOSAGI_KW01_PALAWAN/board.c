/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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

#include "hal.h"
#include "adc.h"

#define RADIO_REG_DIOMAPPING2   (0x26)
#define RADIO_CLK_DIV1          (0x00)
#define RADIO_CLK_DIV2          (0x01)
#define RADIO_CLK_DIV4          (0x02)
#define RADIO_CLK_DIV8          (0x03)
#define RADIO_CLK_DIV16         (0x04)
#define RADIO_CLK_DIV32         (0x05)
#define RADIO_CLK_RC            (0x06)
#define RADIO_CLK_OFF           (0x07)

#define PAIR_CFG_ADC_NUM        (23)

#define PALAWAN_CFG_RESISTANCE_THRESH 128
#define PALAWAN_RX_VALUE 65536
#define PALAWAN_RX_PAIR_VALUE 0

/*

There are two variants, both of which use the same firmware.
The default connection is configured for the Tx board.  The ADC is read, and if
it at 22k or 0 ohms, then this board is assumed to be an Rx board, and the
alternate PAL configuration is implemented.

Note that the radio reset line can be in two places depending on which board we're on,

+-------+---------------+----------------------------------------+----------------------------------------+-------+-----+
| Pad   | Name          | Tx Function                            | Rx Function                            | Same? | AF  |
+-------+---------------+----------------------------------------+----------------------------------------+-------+-----+
| PTA0  | SWD_CLK       | SWD clock line                                                                  |   *   |  7  |
| PTA1  | CAP1          | Capacitive button 1                    | NC                                     |       |  1  |
| PTA2  | SW1           | Switch button 1                        | NC                                     |       |  1  |
| PTA3  | SWD_DIO       | SWD data line                                                                   |   *   |  7  |
| PTA4  | USBDN_RST     | Radio Reset assert                     | USB D- line                            |   *   |  1  |
| PTA18 | RF_CLK        | Radio RF clock input                                                            |   *   |  1  |
| PTA19 | SW2           | Switch button 2                        | NC                                     |       |  1  |
| PTA20 | MCU_RESET     | Hard reset switch                                                               |   *   |  7  |
| PTB0  | USBDP_LED     | White LED                              | USB D+ line                            |       |  1  |
| PTB1  | CAP2          | Capacitive button 2                    | NC                                     |       |  1  |
| PTB2  | CAP3          | Capacitive button 3                    | GPIO                                   |   *   |  1  |
| PTB17 | CAP4          | Capacitive button 4                    | GPIO                                   |   *   |  1  |
| PTC1  | SW5_ADC1      | Switch button 5 / ADC input 1          | GPIO                                   |   *   |  1  |
| PTC2  | SW6_ADC2      | Switch button 6 / ADC input 2          | GPIO                                   |   *   |  1  |
| PTC3  | SW3           | Switch button 3                        | GPIO                                   |   *   |  1  |
| PTC4  | RF_PKT_RDY    | GPIO input -- a radio packet is available                                       |   *   |  1  |
| PTC5  | SPI1_CLK      | Bottom pad, also internal connection to transciever SCK                         |   *   |  2  |
| PTC6  | SPI1_MISO     | Bottom pad, also internal connection to transciever MOSI                        |   *   |  2  |
| PTC7  | SPI1_MOSI     | Bottom pad, also internal connection to transciever MISO                        |   *   |  2  |
| PTD0  | SPI1_CS       | Bottom pad, also internal connection to transciever NSS                         |   *   |  2  |
| PTD4  | SW4           | Switch button 4                        | GPIO                                   |   *   |  1  |
| PTD5  | SW7_ADC3      | Switch button 7 / ADC input 3          | GPIO                                   |   *   |  1  |
| PTD6  | SW8_ADC4      | Switch button 8 / ADC input 4          | GPIO                                   |   *   |  1  |
| PTD7  | LED1          | Green LED                                                                       |   *   |  1  |
| PTE0  | SW9_ADC5      | Switch button 9 / ADC input 5          | GPIO                                   |   *   |  1  |
| PTE1  | SW10_ADC6     | Switch button 10 / ADC input 6         | GPIO                                   |   *   |  1  |
| PTE2  | -             | Internal connection to transciever GPIO DIO0 -- Floating                        |   *   |  1  |
| PTE3  | -             | Internal connection to transciever GPIO DIO1 -- GPIO                            |   *   |  1  |
| PTE16 | ADC9_UART2TX  | Switch button 9 or UART2 TX (depending on software configuration)               |   *   | 1/3 |
| PTE17 | ADC10_UART2RX | Switch button 10 or UART2 RX (depending on software configuration)              |   *   | 1/3 |
| PTE18 | SW11_ADC7     | Switch button 11 / ADC input 7         | GPIO                                   |   *   |  1  |
| PTE19 | SW12_ADC8     | Switch button 12 / ADC input 8         | Radio Reset assert                     |   *   |  1  |
| PTE30 | PAIR_CFG      | Pair button GPIO                       | ADC input                              |   *   |     |
+-------+---------------+----------------------------------------+----------------------------------------+-------+-----+
*/

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config =
{
  .ports = {
    {
      .port = IOPORT1,  // PORTA
      .pads = {
        /* PTA0*/ PAL_MODE_ALTERNATIVE_7,   /* PTA1*/ PAL_MODE_INPUT,           /* PTA2*/ PAL_MODE_INPUT,
        /* PTA3*/ PAL_MODE_ALTERNATIVE_7,   /* PTA4*/ PAL_MODE_INPUT,           /* PTA5*/ PAL_MODE_UNCONNECTED,
        /* PTA6*/ PAL_MODE_UNCONNECTED,     /* PTA7*/ PAL_MODE_UNCONNECTED,     /* PTA8*/ PAL_MODE_UNCONNECTED,
        /* PTA9*/ PAL_MODE_UNCONNECTED,     /*PTA10*/ PAL_MODE_UNCONNECTED,     /*PTA11*/ PAL_MODE_UNCONNECTED,
        /*PTA12*/ PAL_MODE_UNCONNECTED,     /*PTA13*/ PAL_MODE_UNCONNECTED,     /*PTA14*/ PAL_MODE_UNCONNECTED,
        /*PTA15*/ PAL_MODE_UNCONNECTED,     /*PTA16*/ PAL_MODE_UNCONNECTED,     /*PTA17*/ PAL_MODE_UNCONNECTED,
        /*PTA18*/ PAL_MODE_INPUT_ANALOG,    /*PTA19*/ PAL_MODE_INPUT,           /*PTA20*/ PAL_MODE_ALTERNATIVE_7,
        /*PTA21*/ PAL_MODE_UNCONNECTED,     /*PTA22*/ PAL_MODE_UNCONNECTED,     /*PTA23*/ PAL_MODE_UNCONNECTED,
        /*PTA24*/ PAL_MODE_UNCONNECTED,     /*PTA25*/ PAL_MODE_UNCONNECTED,     /*PTA26*/ PAL_MODE_UNCONNECTED,
        /*PTA27*/ PAL_MODE_UNCONNECTED,     /*PTA28*/ PAL_MODE_UNCONNECTED,     /*PTA29*/ PAL_MODE_UNCONNECTED,
        /*PTA30*/ PAL_MODE_UNCONNECTED,     /*PTA31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT2,  // PORTB
      .pads = {
        /* PTB0*/ PAL_MODE_INPUT,           /* PTB1*/ PAL_MODE_INPUT,           /* PTB2*/ PAL_MODE_INPUT,
        /* PTB3*/ PAL_MODE_UNCONNECTED,     /* PTB4*/ PAL_MODE_UNCONNECTED,     /* PTB5*/ PAL_MODE_UNCONNECTED,
        /* PTB6*/ PAL_MODE_UNCONNECTED,     /* PTB7*/ PAL_MODE_UNCONNECTED,     /* PTB8*/ PAL_MODE_UNCONNECTED,
        /* PTB9*/ PAL_MODE_UNCONNECTED,     /*PTB10*/ PAL_MODE_UNCONNECTED,     /*PTB11*/ PAL_MODE_UNCONNECTED,
        /*PTB12*/ PAL_MODE_UNCONNECTED,     /*PTB13*/ PAL_MODE_UNCONNECTED,     /*PTB14*/ PAL_MODE_UNCONNECTED,
        /*PTB15*/ PAL_MODE_UNCONNECTED,     /*PTB16*/ PAL_MODE_UNCONNECTED,     /*PTB17*/ PAL_MODE_INPUT,
        /*PTB18*/ PAL_MODE_UNCONNECTED,     /*PTB19*/ PAL_MODE_UNCONNECTED,     /*PTB20*/ PAL_MODE_UNCONNECTED,
        /*PTB21*/ PAL_MODE_UNCONNECTED,     /*PTB22*/ PAL_MODE_UNCONNECTED,     /*PTB23*/ PAL_MODE_UNCONNECTED,
        /*PTB24*/ PAL_MODE_UNCONNECTED,     /*PTB25*/ PAL_MODE_UNCONNECTED,     /*PTB26*/ PAL_MODE_UNCONNECTED,
        /*PTB27*/ PAL_MODE_UNCONNECTED,     /*PTB28*/ PAL_MODE_UNCONNECTED,     /*PTB29*/ PAL_MODE_UNCONNECTED,
        /*PTB30*/ PAL_MODE_UNCONNECTED,     /*PTB31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT3,  // PORTC
      .pads = {
        /* PTC0*/ PAL_MODE_INPUT_ANALOG,    /* PTC1*/ PAL_MODE_INPUT,           /* PTC2*/ PAL_MODE_INPUT,
        /* PTC3*/ PAL_MODE_INPUT,           /* PTC4*/ PAL_MODE_INPUT,           /* PTC5*/ PAL_MODE_ALTERNATIVE_2,
        /* PTC6*/ PAL_MODE_ALTERNATIVE_2,   /* PTC7*/ PAL_MODE_ALTERNATIVE_2,   /* PTC8*/ PAL_MODE_UNCONNECTED,
        /* PTC9*/ PAL_MODE_UNCONNECTED,     /*PTC10*/ PAL_MODE_UNCONNECTED,     /*PTC11*/ PAL_MODE_UNCONNECTED,
        /*PTC12*/ PAL_MODE_UNCONNECTED,     /*PTC13*/ PAL_MODE_UNCONNECTED,     /*PTC14*/ PAL_MODE_UNCONNECTED,
        /*PTC15*/ PAL_MODE_UNCONNECTED,     /*PTC16*/ PAL_MODE_UNCONNECTED,     /*PTC17*/ PAL_MODE_UNCONNECTED,
        /*PTC18*/ PAL_MODE_UNCONNECTED,     /*PTC19*/ PAL_MODE_UNCONNECTED,     /*PTC20*/ PAL_MODE_UNCONNECTED,
        /*PTC21*/ PAL_MODE_UNCONNECTED,     /*PTC22*/ PAL_MODE_UNCONNECTED,     /*PTC23*/ PAL_MODE_UNCONNECTED,
        /*PTC24*/ PAL_MODE_UNCONNECTED,     /*PTC25*/ PAL_MODE_UNCONNECTED,     /*PTC26*/ PAL_MODE_UNCONNECTED,
        /*PTC27*/ PAL_MODE_UNCONNECTED,     /*PTC28*/ PAL_MODE_UNCONNECTED,     /*PTC29*/ PAL_MODE_UNCONNECTED,
        /*PTC30*/ PAL_MODE_UNCONNECTED,     /*PTC31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT4,  // PORTD
      .pads = {
        /* PTD0*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTD1*/ PAL_MODE_UNCONNECTED,     /* PTD2*/ PAL_MODE_UNCONNECTED,
        /* PTD3*/ PAL_MODE_UNCONNECTED,     /* PTD4*/ PAL_MODE_INPUT,           /* PTD5*/ PAL_MODE_INPUT,
        /* PTD6*/ PAL_MODE_INPUT,           /* PTD7*/ PAL_MODE_OUTPUT_PUSHPULL, /* PTD8*/ PAL_MODE_UNCONNECTED,
        /* PTD9*/ PAL_MODE_UNCONNECTED,     /*PTD10*/ PAL_MODE_UNCONNECTED,     /*PTD11*/ PAL_MODE_UNCONNECTED,
        /*PTD12*/ PAL_MODE_UNCONNECTED,     /*PTD13*/ PAL_MODE_UNCONNECTED,     /*PTD14*/ PAL_MODE_UNCONNECTED,
        /*PTD15*/ PAL_MODE_UNCONNECTED,     /*PTD16*/ PAL_MODE_UNCONNECTED,     /*PTD17*/ PAL_MODE_UNCONNECTED,
        /*PTD18*/ PAL_MODE_UNCONNECTED,     /*PTD19*/ PAL_MODE_UNCONNECTED,     /*PTD20*/ PAL_MODE_UNCONNECTED,
        /*PTD21*/ PAL_MODE_UNCONNECTED,     /*PTD22*/ PAL_MODE_UNCONNECTED,     /*PTD23*/ PAL_MODE_UNCONNECTED,
        /*PTD24*/ PAL_MODE_UNCONNECTED,     /*PTD25*/ PAL_MODE_UNCONNECTED,     /*PTD26*/ PAL_MODE_UNCONNECTED,
        /*PTD27*/ PAL_MODE_UNCONNECTED,     /*PTD28*/ PAL_MODE_UNCONNECTED,     /*PTD29*/ PAL_MODE_UNCONNECTED,
        /*PTD30*/ PAL_MODE_UNCONNECTED,     /*PTD31*/ PAL_MODE_UNCONNECTED,
      },
    },
    {
      .port = IOPORT5,  // PORTE
      .pads = {
        /* PTE0*/ PAL_MODE_INPUT,           /* PTE1*/ PAL_MODE_INPUT,           /* PTE2*/ PAL_MODE_UNCONNECTED,
        /* PTE3*/ PAL_MODE_INPUT,           /* PTE4*/ PAL_MODE_UNCONNECTED,     /* PTE5*/ PAL_MODE_UNCONNECTED,
        /* PTE6*/ PAL_MODE_UNCONNECTED,     /* PTE7*/ PAL_MODE_UNCONNECTED,     /* PTE8*/ PAL_MODE_UNCONNECTED,
        /* PTE9*/ PAL_MODE_UNCONNECTED,     /*PTE10*/ PAL_MODE_UNCONNECTED,     /*PTE11*/ PAL_MODE_UNCONNECTED,
        /*PTE12*/ PAL_MODE_UNCONNECTED,     /*PTE13*/ PAL_MODE_UNCONNECTED,     /*PTE14*/ PAL_MODE_UNCONNECTED,
        /*PTE15*/ PAL_MODE_UNCONNECTED,     /*PTE16*/ PAL_MODE_ALTERNATIVE_3,   /*PTE17*/ PAL_MODE_ALTERNATIVE_3,
        /*PTE18*/ PAL_MODE_INPUT,           /*PTE19*/ PAL_MODE_INPUT,           /*PTE20*/ PAL_MODE_UNCONNECTED,
        /*PTE21*/ PAL_MODE_UNCONNECTED,     /*PTE22*/ PAL_MODE_UNCONNECTED,     /*PTE23*/ PAL_MODE_UNCONNECTED,
        /*PTE24*/ PAL_MODE_UNCONNECTED,     /*PTE25*/ PAL_MODE_UNCONNECTED,     /*PTE26*/ PAL_MODE_UNCONNECTED,
        /*PTE27*/ PAL_MODE_UNCONNECTED,     /*PTE28*/ PAL_MODE_UNCONNECTED,     /*PTE29*/ PAL_MODE_UNCONNECTED,
        /*PTE30*/ PAL_MODE_INPUT_ANALOG,    /*PTE31*/ PAL_MODE_UNCONNECTED,
      },
    },
  },
};
#endif

static int _model = palawan_unknown;


/* The Palawan model type is encoded in resistors hanging off of PTE30:

   - 22k: Palawan Rx
   - 0k: Palawan Rx (pairing mode)
   - ...
*/
enum palawan_model palawanModel(void) {
 
  if ((_model != palawan_tx) && (_model != palawan_rx)) {
    uint16_t gain;
    int resistance;
    int resistance_min;
    int resistance_max;

    /* Ungate PORTE, where the ADC we're using lives */
    SIM->SCGC5 |= SIM_SCGC5_PORTE;

    /* Configure PTE30 to be an input pad */
    pal_lld_setpadmode(IOPORT5, 30, PAL_MODE_INPUT_ANALOG);

    /* Ungate the ADC */
    SIM->SCGC6 |= SIM_SCGC6_ADC0;

    /* Quick-and-dirty calibration */
    ADC0->CFG1 =  ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_8) |
                  ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK_DIV_2);

    /* Software trigger, no DMA */
    ADC0->SC2 = 0;

    /* Enable hardware averaging over 32 samples, and run calibration */
    ADC0->SC3 = ADCx_SC3_AVGE |
                ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_32_SAMPLES) |
                ADCx_SC3_CAL;

    /* Wait for calibration to finish */
    while (!(ADC0->SC1A & ADCx_SC1n_COCO))
      ;

    /* Adjust gain according to reference manual */

    gain = ((ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 +
             ADC0->CLP3 + ADC0->CLP4 + ADC0->CLPS) / 2) | 0x8000;
    ADC0->PG = gain;

    gain = ((ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 +
             ADC0->CLM3 + ADC0->CLM4 + ADC0->CLMS) / 2) | 0x8000;
    ADC0->MG = gain;

    /* Reset Rn */
    (void)ADC0->RA;

    /* Configure for 16-bit conversion */
    ADC0->CFG1 =  ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_8) |
                  ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK_DIV_2) |
                  ADCx_CFG1_MODE(ADCx_CFG1_MODE_16_BITS);

    /* Perform the sample read, averaging over 32 samples */
    ADC0->SC3 = ADCx_SC3_AVGE |
                ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_32_SAMPLES);

    /* Begin the read */
    ADC0->SC1A = ADCx_SC1n_ADCH(PAIR_CFG_ADC_NUM);

    /* Wait for sample to finish */
    while (!(ADC0->SC1A & ADCx_SC1n_COCO))
      ;

    resistance = ADC0->RA;

    /* Figure out which threshold it falls into */
    resistance_min = resistance - (PALAWAN_CFG_RESISTANCE_THRESH / 2);
    resistance_max = resistance + (PALAWAN_CFG_RESISTANCE_THRESH / 2);

    if ((resistance_min < PALAWAN_RX_VALUE)
     && (resistance_max > PALAWAN_RX_VALUE))
      _model = palawan_rx;
    else if ((resistance_min < PALAWAN_RX_PAIR_VALUE)
          && (resistance_max > PALAWAN_RX_PAIR_VALUE))
      _model = palawan_rx;
  }

  return _model;
}

static void radio_reset(void)
{
  if (palawanModel() == palawan_tx)
    GPIOA->PSOR = (1 << 4);
  else if (palawanModel() == palawan_rx)
    GPIOE->PSOR = (1 << 19);
}

static void radio_enable(void)
{
  if (palawanModel() == palawan_tx)
    GPIOA->PCOR = (1 << 4);
  else if (palawanModel() == palawan_rx)
    GPIOE->PCOR = (1 << 19);
}

static void assert_cs(void)
{
  GPIOD->PCOR = (1 << 0);
}

static void deassert_cs(void)
{
  GPIOD->PSOR = (1 << 0);
}

static void spi_read_status(void)
{
  (void)SPI0->S;
}

/**
 * @brief   Send a byte and discard the response
 *
 * @notapi
 */
void spi_xmit_byte_sync(uint8_t byte)
{
  /* Send the byte */
  SPI0->DL = byte;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPTEF))
    asm("");

  /* Discard the response */
  (void)SPI0->DL;
}

/**
 * @brief   Send a dummy byte and return the response
 *
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t spi_recv_byte_sync(void)
{
  /* Send the byte */
  SPI0->DL = 0;

  /* Wait for the byte to be transmitted */
  while (!(SPI0->S & SPIx_S_SPRF))
    asm("");

  /* Discard the response */
  return SPI0->DL;
}


/**
 * @brief   Read a value from a specified radio register
 *
 * @param[in] addr      radio register to read from
 * @return              value read from said register
 *
 * @notapi
 */
uint8_t radio_read_register(uint8_t addr) {
  uint8_t val;

  spi_read_status();
  assert_cs();

  spi_xmit_byte_sync(addr);
  spi_recv_byte_sync();
  val = spi_recv_byte_sync();

  deassert_cs();

  return val;
}

/**
 * @brief   Write a value to a specified radio register
 *
 * @param[in] addr      radio register to write to
 * @param[in] val       value to write to said register
 *
 * @notapi
 */
void radio_write_register(uint8_t addr, uint8_t val) {

  spi_read_status();
  assert_cs();
  /* Send the address to write */
  spi_xmit_byte_sync(addr | 0x80);

  /* Send the actual value */
  spi_xmit_byte_sync(val);

  (void)spi_recv_byte_sync();
  deassert_cs();
}

static void early_usleep(int usec) {
  int j, k;

  for (j = 0; j < usec; j++)
    for (k = 0; k < 30; k++)
        asm("");
}

static void early_msleep(int msec) {
  int i, j, k;

  for (i = 0; i < msec; i++)
    for (j = 0; j < 1000; j++)
      for (k = 0; k < 30; k++)
        asm("");
}

/**
 * @brief   Power cycle the radio
 * @details Put the radio into reset, then wait 100 us.  Bring it out of
 *          reset, then wait 5 ms.  Ish.
 */
int radio_power_cycle(void)
{
  /* @AC RESET sequence from SX1233 datasheet:
   * RESET high Z
   * RESET = 1, then wait 100us
   * RESET = 0 (high Z), then wait 5ms before using the Radio. 
   */

  radio_reset();

  /* Cheesy usleep(100).*/
  early_usleep(100);

  radio_enable();

  /* Cheesy msleep(5).*/
  early_msleep(5);

  return 1;
}

/**
 * @brief   Set up mux ports, enable SPI, and set up GPIO.
 * @details The MCU communicates to the radio via SPI and a few GPIO lines.
 *          Set up the pinmux for these GPIO lines, and set up SPI.
 */
void early_init_radio(void)
{

  /* Enable Reset GPIO and SPI PORT clocks by unblocking
   * PORTC, PORTD, and PORTE.*/
  SIM->SCGC5 |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTC | SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);

  /* Map Reset to a GPIO, which is looped from PTE19 back into
     the RESET_B_XCVR port.*/
  if (palawanModel() == palawan_tx)
    pal_lld_setpadmode(IOPORT1, 4, PAL_MODE_OUTPUT_PUSHPULL);
  else if (palawanModel() == palawan_rx)
    pal_lld_setpadmode(IOPORT5, 19, PAL_MODE_OUTPUT_PUSHPULL);

  /* Enable SPI clock.*/
  SIM->SCGC4 |= SIM_SCGC4_SPI0;

  /* Mux PTD0 as a GPIO, since it's used for Chip Select.*/
  pal_lld_setpadmode(IOPORT4, 0, PAL_MODE_OUTPUT_PUSHPULL);
  GPIOD->PDDR |= (1 << 0);

  /* Mux PTC5 as SCK */
  pal_lld_setpadmode(IOPORT3, 5, PAL_MODE_ALTERNATIVE_2);

  /* Mux PTC6 as MISO */
  pal_lld_setpadmode(IOPORT3, 6, PAL_MODE_ALTERNATIVE_2);

  /* Mux PTC7 as MOSI */
  pal_lld_setpadmode(IOPORT3, 7, PAL_MODE_ALTERNATIVE_2);

  /* Keep the radio in reset.*/
  radio_reset();

  /* Initialize the SPI peripheral default values.*/
  SPI0->C1 = 0;
  SPI0->C2 = 0;
  SPI0->BR = 0;

  /* Enable SPI system, and run as a Master.*/
  SPI0->C1 |= (SPIx_C1_SPE | SPIx_C1_MSTR);

  spi_read_status();
  deassert_cs();
}



/**
 * @brief   Configure the radio to output a given clock frequency
 *
 * @param[in] osc_div   a factor of division, of the form 2^n
 *
 * @notapi
 */
static void radio_configure_clko(uint8_t osc_div)
{
  radio_write_register(RADIO_REG_DIOMAPPING2,
      (radio_read_register(RADIO_REG_DIOMAPPING2) & ~7) | osc_div);
}

#define SCL_PIN (1 << 1)
#define SDA_PIN (1 << 2)

static void set_scl(void) {
  FGPIOC->PSOR = SCL_PIN;
}

static void clr_scl(void) {
  FGPIOC->PCOR = SCL_PIN;
}

static void set_sda(void) {
  FGPIOC->PSOR = SDA_PIN;
}

static void clr_sda(void) {
  FGPIOC->PCOR = SDA_PIN;
}

/**
 * @brief   Get I2C into a known-good state
 * @details If the MCU is reset when I2C is in the middle of a transaction,
 *          the I2C slaves may hold the line low waiting for the clock line
 *          to transition.
 *          It is possible to "unblock" the bus by jamming at least 9 high
 *          pulses onto the line while toggling SCL.  This will cause the
 *          slave device to NAK when it's finished, thus freeing the bus.
 *
 * @notapi
 */
/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 */
void __early_init(void) {

  switch (palawanModel()) {
    case palawan_tx:
      break;
      
    case palawan_rx:
      break;
      
    default:
      osalSysHalt("Unable to determine Palawan model");
      break;
  }

  early_init_radio();
  radio_power_cycle();

  /* 32Mhz/4 = 8 MHz CLKOUT.*/
  radio_configure_clko(RADIO_CLK_DIV1);

  kl1x_clock_init();
}

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
}
