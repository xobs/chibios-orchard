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
#include "i2c.h"
#include "spi.h"

#include "chprintf.h"
#include "usbphy.h"
#include "usbmac.h"
#include "usblink.h"

#include "palawan.h"
#include "palawan-events.h"
#include "palawan-shell.h"

#include "string.h"

uint8_t pee_pbe(void);
uint8_t pbe_fbe(void);
int32_t fll_freq(int32_t fll_ref);
uint8_t fbe_fei(void);
void cpuStop(void);
void cpuVLLS0(void);

static const SPIConfig spi_config = {
  NULL,
  /* HW dependent part.*/
  GPIOD,
  0,
};

#if (CH_USE_RT == TRUE)
struct evt_table palawan_events;

static void shell_termination_handler(eventid_t id) {
  static int i = 1;
  (void)id;

  chprintf(stream, "\r\nRespawning shell (shell #%d, event %d)\r\n", ++i, id);
  palawanShellRestart();
}
#endif

/*
static void default_radio_handler(uint8_t type, uint8_t src, uint8_t dst,
                                  uint8_t length, const void *data) {

  chprintf(stream, "\r\nNo handler for packet found.  %02x -> %02x : %02x\r\n",
           src, dst, type);
  print_hex(stream, data, length, 0);
}
*/

static void print_mcu_info(void) {
  uint32_t sdid = SIM->SDID;
  const char *famid[] = {
    "KL0%d (low-end)",
    "KL1%d (basic)",
    "KL2%d (USB)",
    "KL3%d (Segment LCD)",
    "KL4%d (USB and Segment LCD)",
  };
  const uint8_t ram[] = {
    0,
    1,
    2,
    4,
    8,
    16,
    32,
    64,
  };

  const uint8_t pins[] = {
    16,
    24,
    32,
    36,
    48,
    64,
    80,
  };

  if (((sdid >> 20) & 15) != 1) {
    chprintf(stream, "Device is not Kinetis KL-series\r\n");
    return;
  }

  chprintf(stream, famid[(sdid >> 28) & 15], (sdid >> 24) & 15);
  chprintf(stream, " with %d kB of ram detected"
                   " (Rev: %04x  Die: %04x  Pins: %d).\r\n",
                   ram[(sdid >> 16) & 15],
                   (sdid >> 12) & 15,
                   (sdid >> 7) & 31,
                   pins[(sdid >> 0) & 15]);
}

/*
 * Application entry point.
 */
int loop_counter = 0;
int last_ret;

static const struct usb_device_descriptor usb_device_descriptor = {
  .bLength = sizeof(struct usb_device_descriptor),
  .bDescriptorType = DT_DEVICE,       /* DEVICE */
  .bcdUSB = 0x0200,           /* USB 2.0 */
  .bDeviceClass = 0x00,
  .bDeviceSubClass = 0x00,
  .bDeviceProtocol = 0x00,
  .bMaxPacketSize0 = 0x08,    /* 8-byte packets max */
  .idVendor = 0x1bcf,
  .idProduct = 0x05ce,
  .bcdDevice = 0xa014,        /* Device release 1.0 */
  .iManufacturer = 0x00,      /* No manufacturer string */
  .iProduct = 0x00,           /* Product name in string #2 */
  .iSerialNumber = 0x00,      /* No serial number */
  .bNumConfigurations = 0x01,
};

static const struct usb_configuration_descriptor usb_config_descriptor = {
  .bLength = sizeof(struct usb_configuration_descriptor),
  .bDescriptorType = DT_CONFIGURATION,
  .wTotalLength = sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor),
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0xa0,       /* Remote wakeup supported */
  .bMaxPower = 100/2,         /* 100 mA (in 2-mA units) */
  .data = {
    /* struct usb_interface_descriptor { */
    /*  uint8_t bLength;            */ sizeof(struct usb_interface_descriptor),
    /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
    /*  uint8_t bInterfaceNumber;   */ 0,
    /*  uint8_t bAlternateSetting;  */ 0,
    /*  uint8_t bNumEndpoints;      */ 1, /* One additional EP */
    /*  uint8_t bInterfaceClass;    */ 3, /* HID class */
    /*  uint8_t bInterfaceSubclass; */ 1, /* Boot Device subclass */
    /*  uint8_t bInterfaceProtocol; */ 1, /* 1 == keyboard, 2 == mouse */
    /*  uint8_t iInterface;         */ 0,
    /* }*/

    /* struct usb_hid_descriptor {        */
    /*  uint8_t  bLength;                 */ sizeof(struct usb_hid_descriptor),
    /*  uint8_t  bDescriptorType;         */ DT_HID,
    /*  uint16_t bcdHID;                  */ 0x00, 0x01,
    /*  uint8_t  bCountryCode;            */ 0,
    /*  uint8_t  bNumDescriptors;         */ 1, /* We have only one REPORT */
    /*  uint8_t  bReportDescriptorType;   */ DT_HID_REPORT,
    /*  uint16_t wReportDescriptorLength; */ 63, 0x00,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ sizeof(struct usb_endpoint_descriptor),
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x81,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ 10, /* Every 10 ms */
    /* }                              */
  },

  /*
         0x09 DT_CONFIGURATION 0x3B 0x00 0x02 0x01 0x00 0xA0 0x17

         0x09 DT_INTERFACE 0x00 0x00 0x01 0x03 0x01 0x01 0x00
         0x09 DT_HID 0x00 0x01 0x00 0x01 0x22 0x41 0x00
         0x07 DT_ENDPOINT 0x81 0x03 0x08 0x00 0x0A

         0x09 DT_INTERFACE 0x01 0x00 0x01 0x03 0x01 0x02 0x00
         0x09 DT_HID 0x00 0x01 0x00 0x01 0x22 0xB3 0x00
         0x07 DT_ENDPOINT 0x82 0x03 0x08 0x00 0x0A
  */
};

int main(void)
{
#if (CH_USE_RT == TRUE)
  evtTableInit(palawan_events, 32);
#endif

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   */
  halInit();
#if (CH_USE_RT == TRUE)
  chSysInit();
#endif

#if 0
    osalSysLock();
  while (1) {
    *((volatile uint32_t *)0xf80000cc) = 0x80; /* Toggle green LED */
    usbPhyWriteTest(usbPhyTestPhy());
//    chThdSleepMilliseconds(1000);
    int i;
    for (i = 0; i < 1000; i++)
      asm("nop");
  }
    osalSysUnlock();
#endif

  {
  }

  if (palawanModel() == palawan_rx) {
    int i;
    /* Set our ISR (PINA_IRQn) to the highest value (0), above all others */
    NVIC_SetPriority(SVCall_IRQn, 1);
    NVIC_SetPriority(PendSV_IRQn, 1);
    NVIC_SetPriority(SysTick_IRQn, 1);
    for (i = 0; i < CORTEX_NUM_VECTORS; i++) {
      if (NVIC_GetPriority(i) == 0)
        NVIC_SetPriority(i, 3);
    }
    NVIC_SetPriority(PINA_IRQn, 0);

    /* Initialize USB */
    usbMacInit(usbMacDefault(), &usb_device_descriptor, &usb_config_descriptor);
    usbPhyInit(usbPhyDefaultPhy(), usbMacDefault());
  }

  else if (palawanModel() == palawan_tx) {
 
    extern int palawanGpioInit(void);
    *((volatile uint32_t *)0xf80000cc) = 0x80; /* Toggle green LED */
    palawanGpioInit();

    extern int palawanGpioStart(void);
    palawanGpioStart();
  }

  palawanEventsStart();
  palawanShellInit();

  chprintf(stream, "\r\n\r\nPalawan shell.  Based on build %s\r\n", gitversion);
  print_mcu_info();

  spiStart(&SPID1, &spi_config);
  spiStart(&SPID2, &spi_config);

  evtTableHook(palawan_events, shell_terminated, shell_termination_handler);
  palawanShellRestart();

#if (CH_USE_RT == TRUE)
  while (TRUE)
    chEvtDispatch(evtHandlers(palawan_events), chEvtWaitOne(ALL_EVENTS));
#else
  int i;
  for (i = 0; i < 100000; i++)
    asm("nop");
  usbPhyAttach(usbPhyDefaultPhy());
  while (1) {
    extern void usbPhyWorker(struct USBPHY *phy);
    usbPhyWorker(usbPhyDefaultPhy());
  }
#endif
}

#if 0
void halt(void) {
  // subsystems to bring down, in this order:
  // touch
  // LEDs
  // accelerometer
  // bluetooth, reset, setup, sleep
  // OLED
  // charger
  // GPIOX
  // Radio
  // CPU

  captouchStop();
  chprintf(stream, "Captouch stopped\n\r");

  while(!effectsStop()) { // wait for the effects thread to exit
    chThdYield();
    chThdSleepMilliseconds(EFFECTS_REDRAW_MS * 10);
  }
  chprintf(stream, "Effects stopped\n\r");
  
  accelStop();
  chprintf(stream, "Accelerometer stopped\n\r");

  // for now BLE seems broken, leave it alone...
  bleStop(bleDriver);
  chprintf(stream, "BLE stopped\n\r");

  chargerStop();
  chprintf(stream, "Charger stopped\n\r");

  oledStop(&SPID2);
  chprintf(stream, "OLED stopped\n\r");

  gpioxStop();
  chprintf(stream, "GPIOX stopped\n\r");

  chprintf(stream, "Slowing clock, sleeping radio & halting CPU...\n\r");
  pee_pbe();  // transition through three states to get to fei mode
  pbe_fbe();
  fbe_fei();
  
  radioStop(radioDriver);
  cpuVLLS0();
}

// look in ../os/ext/CMSIS/KINETIS/kl17z.h for CPU register set
void cpuStop(void) {
  volatile unsigned int dummyread;
  /*The PMPROT register may have already been written by init
    code. If so, then this next write is not done since
    PMPROT is write once after RESET
    this write-once bit allows the MCU to enter the
    normal STOP mode.
    If AVLP is already a 1, VLPS mode is entered
    instead of normal STOP
    is SMC_PMPROT = 0 */
  /* Set the STOPM field to 0b000 for normal STOP mode
     For Kinetis L: if trying to enter Stop from VLPR user
     forced to VLPS low power mode */
  SMC->PMCTRL &= ~SMC_PMCTRL_STOPM_MASK;
  SMC->PMCTRL |= SMC_PMCTRL_STOPM(0);
  
  /*wait for write to complete to SMC before stopping core */
  dummyread =  SMC->PMCTRL;
  //#warning "check that this disassembles to a read"
  //  (void) SMC->PMCTRL; // check that this works
  dummyread++; // get rid of compiler warning

  /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  /* WFI instruction will start entry into STOP mode */
  asm("WFI");
}

void cpuVLLS0(void) {
  volatile unsigned int dummyread;
  
  /* Write to PMPROT to allow all possible power modes */
  SMC->PMPROT = 0x2A; // enable all  possible modes
  /* Set the STOPM field to 0b100 for VLLS0 mode */
  SMC->PMCTRL &= 0xF8;
  SMC->PMCTRL |= 0x04;
  /* set VLLSM = 0b00 */
  SMC->STOPCTRL = 0x03;  // POR detect allowed, normal stop mode
  /*wait for write to complete to SMC before stopping core */
  dummyread = SMC->STOPCTRL;
  dummyread++; // get rid of compiler warning
  
  /* Set the SLEEPDEEP bit to enable deep sleep mode (STOP) */
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  dummyread = SCB->SCR;
  /* WFI instruction will start entry into STOP mode */
  asm("WFI");
}

uint8_t pee_pbe(void) {
  int16_t i;
  
  // Check MCG is in PEE mode
  if (!((((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x3) && // check CLKS mux has selcted PLL output
	(!(MCG->S & MCG_S_IREFST)) &&                               // check FLL ref is external ref clk
	(MCG->S & MCG_S_PLLST)))                                    // check PLLS mux has selected PLL 
    {
      return 0x8;                                                       // return error code
    } 
  
  // As we are running from the PLL by default the PLL and external clock settings are valid
  // To move to PBE from PEE simply requires the switching of the CLKS mux to select the ext clock 
  // As CLKS is already 0 the CLKS value can simply be OR'ed into the register 
  MCG->C1 |= MCG_C1_CLKS(2); // switch CLKS mux to select external reference clock as MCG_OUT
  
  // Wait for clock status bits to update 
  for (i = 0 ; i < 2000 ; i++) {
    if (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2) break; // jump out early if CLKST shows EXT CLK slected before loop finishes
  }
  if (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2) return 0x1A; // check EXT CLK is really selected and return with error if not

  // Now in PBE mode
  return 0;
} // pee_pbe


uint8_t pbe_fbe(void) {
  uint16_t i;
  
  // Check MCG is in PBE mode
  if (!((((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2) && // check CLKS mux has selcted external reference
      (!(MCG->S & MCG_S_IREFST)) &&                               // check FLL ref is external ref clk
      (MCG->S & MCG_S_PLLST) &&                                   // check PLLS mux has selected PLL
      (!(MCG->C2 & MCG_C2_LP))))                                  // check MCG_C2[LP] bit is not set   
    {
      return 0x7;                                                       // return error code
    }

  // As we are running from the ext clock, by default the external clock settings are valid
  // To move to FBE from PBE simply requires the switching of the PLLS mux to disable the PLL 
  
  MCG->C6 &= ~MCG_C6_PLLS; // clear PLLS to disable PLL, still clocked from ext ref clk
  
  // wait for PLLST status bit to set
  for (i = 0 ; i < 2000 ; i++)  {
    if (!(MCG->S & MCG_S_PLLST)) break; // jump out early if PLLST clears before loop finishes
  }
  if (MCG->S & MCG_S_PLLST) return 0x15; // check bit is really clear and return with error if not clear  
  
  // Now in FBE mode  
  return 0;
 } // pbe_fbe
  
int32_t fll_freq(int32_t fll_ref) {
  int32_t fll_freq_hz;
  
  // Check that only allowed ranges have been selected
  if (((MCG->C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT) > 0x1)  {
    return 0x3B; // return error code if DRS range 2 or 3 selected
  }
  
  if (MCG->C4 & MCG_C4_DMX32) // if DMX32 set
    {
      switch ((MCG->C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT) // determine multiplier based on DRS
	{
	case 0:
	  fll_freq_hz = (fll_ref * 732);
	  if (fll_freq_hz < 20000000) {return 0x33;}
	  else if (fll_freq_hz > 25000000) {return 0x34;}
	  break;
	case 1:
	  fll_freq_hz = (fll_ref * 1464);
	  if (fll_freq_hz < 40000000) {return 0x35;}
	  else if (fll_freq_hz > 50000000) {return 0x36;}
	  break;
	case 2:
	  fll_freq_hz = (fll_ref * 2197);
	  if (fll_freq_hz < 60000000) {return 0x37;}
	  else if (fll_freq_hz > 75000000) {return 0x38;}
	  break;
	case 3:
	  fll_freq_hz = (fll_ref * 2929);
	  if (fll_freq_hz < 80000000) {return 0x39;}
	  else if (fll_freq_hz > 100000000) {return 0x3A;}
	  break;
	}
    }
  else // if DMX32 = 0
    {
      switch ((MCG->C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT) // determine multiplier based on DRS
	{
	case 0:
	  fll_freq_hz = (fll_ref * 640);
	  if (fll_freq_hz < 20000000) {return 0x33;}
	  else if (fll_freq_hz > 25000000) {return 0x34;}
	  break;
	case 1:
	  fll_freq_hz = (fll_ref * 1280);
	  if (fll_freq_hz < 40000000) {return 0x35;}
	  else if (fll_freq_hz > 50000000) {return 0x36;}
	  break;
	case 2:
	  fll_freq_hz = (fll_ref * 1920);
	  if (fll_freq_hz < 60000000) {return 0x37;}
	  else if (fll_freq_hz > 75000000) {return 0x38;}
	  break;
	case 3:
	    fll_freq_hz = (fll_ref * 2560);
	  if (fll_freq_hz < 80000000) {return 0x39;}
	  else if (fll_freq_hz > 100000000) {return 0x3A;}
	  break;
	}
    }    
  return fll_freq_hz;
} // fll_freq

uint8_t fbe_fei(void) {
  unsigned char temp_reg;
  short i;
  int mcg_out;
  int slow_irc_freq = 32768;
  
  // Check MCG is in FBE mode
  if (!((((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2) && // check CLKS mux has selcted external reference
      (!(MCG->S & MCG_S_IREFST)) &&                               // check FLL ref is external ref clk
      (!(MCG->S & MCG_S_PLLST)) &&                                // check PLLS mux has selected FLL
      (!(MCG->C2 & MCG_C2_LP))))                                  // check MCG_C2[LP] bit is not set   
    {
      return 0x4;                                                       // return error code
    }

// Check IRC frequency is within spec.
  if ((slow_irc_freq < 31250) || (slow_irc_freq > 39063))
    {
      return 0x31;
    }
  
  // Check resulting FLL frequency 
  mcg_out = fll_freq(slow_irc_freq); 
  if (mcg_out < 0x3C) {return mcg_out;} // If error code returned, return the code to calling function

  // Need to make sure the clockmonitor is disabled before moving to an "internal" clock mode
  MCG->C6 &= ~MCG_C6_CME0; //This assumes OSC0 is used as the external clock source
  
  // Move to FEI by setting CLKS to 0 and enabling the slow IRC as the FLL reference clock
  temp_reg = MCG->C1;
  temp_reg &= ~MCG_C1_CLKS_MASK; // clear CLKS to select FLL output
  temp_reg |= MCG_C1_IREFS; // select internal reference clock
  MCG->C1 = temp_reg; // update MCG_C1 
  
  // wait for Reference clock Status bit to set
  for (i = 0 ; i < 2000 ; i++)  {
    if (MCG->S & MCG_S_IREFST) break; // jump out early if IREFST sets before loop finishes
  }
  if (!(MCG->S & MCG_S_IREFST)) return 0x12; // check bit is really set and return with error if not set
  
  // Wait for clock status bits to show clock source is ext ref clk
  for (i = 0 ; i < 2000 ; i++)  {
    if (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x0) break; // jump out early if CLKST shows EXT CLK slected before loop finishes
  }
  if (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x0) return 0x18; // check EXT CLK is really selected and return with error if not

  // Now in FEI mode
  return mcg_out;
} // fbe_fei
#endif
