#include <string.h>

#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"

/* We need to reverse bits.  Use a 256-byte lookup table to speed things up. */
const uint8_t bit_reverse_table_256[] = {
#define R2(n)     n,     n + 2*64,     n + 1*64,     n + 3*64
#define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
#define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
  R6(0), R6(2), R6(1), R6(3)
#undef R2
#undef R4
#undef R6
};

static struct USBPHY defaultUsbPhy = {
#ifndef TEST_PHY
  /* PTB0 */
  .usbdnIAddr = &FGPIOB->PDIR,
  .usbdnSAddr = &FGPIOB->PSOR,
  .usbdnCAddr = &FGPIOB->PCOR,
  .usbdnDAddr = &FGPIOB->PDDR,
  .usbdnMask  = (1 << 0),
  .usbdnShift = 0,

  /* PTA4 */
  .usbdpIAddr = &FGPIOA->PDIR,
  .usbdpSAddr = &FGPIOA->PSOR,
  .usbdpCAddr = &FGPIOA->PCOR,
  .usbdpDAddr = &FGPIOA->PDDR,
  .usbdpMask  = (1 << 4),
  .usbdpShift = 4,
#else
  /* Pins J21 and J19 */
  /* PTD5 */
  .usbdpIAddr = &FGPIOD->PDIR,
  .usbdpSAddr = &FGPIOD->PSOR,
  .usbdpCAddr = &FGPIOD->PCOR,
  .usbdpDAddr = &FGPIOD->PDDR,
  .usbdpMask  = (1 << 5),
  .usbdpShift = 5,

  /* PTD6 */
  .usbdnIAddr = &FGPIOD->PDIR,
  .usbdnSAddr = &FGPIOD->PSOR,
  .usbdnCAddr = &FGPIOD->PCOR,
  .usbdnDAddr = &FGPIOD->PDDR,
  .usbdnMask  = (1 << 6),
  .usbdnShift = 6,
#endif
};

static struct USBPHY testWriteUsbPhy = {
  /* PTE0 */
  .usbdpIAddr = &FGPIOE->PDIR,
  .usbdpSAddr = &FGPIOE->PSOR,
  .usbdpCAddr = &FGPIOE->PCOR,
  .usbdpDAddr = &FGPIOE->PDDR,
  .usbdpMask = (1 << 0),
  .usbdpShift = 0,

  /* PTE1 */
  .usbdnIAddr = &FGPIOE->PDIR,
  .usbdnSAddr = &FGPIOE->PSOR,
  .usbdnCAddr = &FGPIOE->PCOR,
  .usbdnDAddr = &FGPIOE->PDDR,
  .usbdnMask = (1 << 1),
  .usbdnShift = 1,
};

int usbPhyInitialized(struct USBPHY *phy) {
  if (!phy)
    return 0;

  return phy->initialized;
}

static void usbCaptureI(struct USBPHY *phy) {

  uint8_t *samples;
  int ret;

  /* Toggle the green LED */
  //*((volatile uint32_t *)0xf80000cc) = 0x80;

  samples = (uint8_t *)phy->read_queue[phy->read_queue_head];

  ret = usbPhyReadI(phy, samples);

  if (ret == USB_DIP_IN) {
    if (!phy->queued_size) {
      uint8_t pkt[] = {USB_PID_NAK};
      usbPhyWriteI(phy, pkt, sizeof(pkt));
      goto out;
    }

    phy->read_queue_head++;
    usbPhyWriteI(phy, phy->queued_data, phy->queued_size);
    goto out;
  }
  else if (ret == USB_DIP_SETUP) {
    phy->read_queue_head++;
    goto out;
  }
  else if (ret == USB_DIP_OUT) {
    phy->read_queue_head++;
    goto out;
  }
  else if (ret == USB_DIP_ACK) {
    /* Allow the next byte to be sent */
    phy->queued_size = 0;
    usbMacTransferSuccess(phy->mac);
    phy->read_queue_head++;
    goto out;
  }

  else if ((ret == USB_DIP_DATA0) || (ret == USB_DIP_DATA1)) {
    phy->read_queue_head++;
    uint8_t pkt[] = {USB_PID_ACK};
    usbPhyWriteI(phy, pkt, sizeof(pkt));
    goto out;
  }

out:
  return;
}

/*
 * Note that this interrupt plays fast-and-loose with ChibiOS conventions.
 * It does not call port_lock_from_isr() on entry, nor does it call
 * port_unlock_from_isr() on exit.  As far as ChibiOS is concerned, this
 * function does not exist.
 *
 * This is because standard ChibiOS IRQ housekeeping adds too much overhead
 * to this process, and we need to respond as quickly as possible.
 */

/* Expose the "rev" instruction to C */
#ifdef REVERSE_BITS
static inline __attribute__((always_inline)) uint32_t __rev(uint32_t val) {
  asm("rev %0, %1" : "=r" (val) : "r" (val) );
  return val;
}
#else /* Don't reverse bits */
#define __rev(x) x
#endif

int usbPhyWritePrepare(struct USBPHY *phy, const void *buffer, int size) {

  phy->queued_data = buffer;
  phy->queued_size = size;
  return 0;
}


void usbPhyWriteTest(struct USBPHY *phy) {

  uint8_t buffer[] = {
//    0xC3, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00, 0xDD, 0x94,
//    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
//    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
//    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
//    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
//    0xc3, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xaa, 0x55,
//    0xd2
//    0x00
     0xC3, 0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x00, 0xE9, 0xA4,
  };
  static uint32_t test_num = 1;

  if (test_num > sizeof(buffer)) {
    uint8_t buffer[] = {
      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    };
    usbPhyWriteI(phy, buffer, sizeof(buffer));
    test_num = 1;
  }
  else {
    usbPhyWriteI(phy, buffer, test_num);
    test_num++;
  }
}

#ifdef TEST_PHY
static int usb_test_ret;
static uint32_t usb_test_samples[3];
static uint8_t usb_test_samples_fixed[12];
static const uint8_t chk_tbl[] = {0xc3, 0x80, 0x6, 0x0, 0x2, 0x0, 0x0, 0xff, 0x0, 0xe9, 0xa4, 0x0};
int phy_success = 0;
int phy_failures = 0;
#endif
void usbPhyWorker(struct USBPHY *phy) {
#ifdef TEST_PHY
  uint8_t *in_ptr = (uint8_t *)usb_test_samples;
  int32_t data_left;
  uint32_t data_copied;
  int i;
  memset(usb_test_samples, 0, sizeof(usb_test_samples));
  usb_test_ret = usbPhyReadI(phy, usb_test_samples);
  if (usb_test_ret == 0xc3) {

    data_left = in_ptr[11] - 1;
    data_copied = 0;

    if (data_left > 0) {
      phy_success++;
      while (data_left >= 0)
        usb_test_samples_fixed[data_copied++] = bit_reverse_table_256[in_ptr[data_left--]];

      for (i = 0; i < sizeof(chk_tbl); i++) {
        if (usb_test_samples_fixed[i] != chk_tbl[i]) {
          phy_success--;
          phy_failures++;
          asm("bkpt #0");
        }
      }
    }
  }
  return;
#else
  while (phy->read_queue_tail != phy->read_queue_head) {
    uint8_t *in_ptr = (uint8_t *)phy->read_queue[phy->read_queue_tail];
    uint8_t bytes[12];
    int32_t data_left;
    uint32_t data_copied;

    data_left = in_ptr[11] - 1;
    data_copied = 0;

    if (data_left > 0) {
      while (data_left >= 0)
        bytes[data_copied++] = bit_reverse_table_256[in_ptr[data_left--]];
      usbMacProcess(phy->mac, bytes, data_copied);
    }
    else if (data_left == 0) {
      bytes[0] = bit_reverse_table_256[in_ptr[0]];
      usbMacProcess(phy->mac, bytes, 1);
    }

    // Finally, move on to the next packet
    phy->read_queue_tail++;
  }
#endif
  return;
}

#if (CH_USE_RT == TRUE)
static THD_FUNCTION(usb_worker_thread, arg) {

  struct USBPHY *phy = arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
    osalSysLock();
    (void) osalThreadSuspendS(&phy->thread);
    osalSysUnlock();

    usbPhyWorker(phy);
  }

  return;
}
#endif

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac) {

  phy->mac = mac;
  usbMacSetPhy(mac, phy);
  phy->initialized = 1;
  usbPhyDetach(phy);
#if (CH_USE_RT == TRUE)
  chEvtObjectInit(&phy->data_available);
  chThdCreateStatic(phy->waThread, sizeof(phy->waThread),
                    HIGHPRIO, usb_worker_thread, phy);
#endif
}

#if (CH_USE_RT == TRUE)
void usbPhyDrainIfNecessary(void) {
  struct USBPHY *phy = &defaultUsbPhy;

  if (phy->read_queue_tail != phy->read_queue_head)
    osalThreadResumeI(&phy->thread, MSG_OK);
}
#endif

struct USBPHY *usbPhyDefaultPhy(void) {

  return &defaultUsbPhy;
}

struct USBPHY *usbPhyTestPhy(void) {

  return &testWriteUsbPhy;
}

OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {

  /* Note: We can't use ANY ChibiOS threads here.
   * This thread may interrupt the SysTick handler, which would cause
   * Major Problems if we called OSAL_IRQ_PROLOGUE().
   * To get around this, we simply examine the buffer every time SysTick
   * exits (via CH_CFG_SYSTEM_TICK_HOOK), and wake up the thread from
   * within the SysTick context.
   * That way, this function is free to preempt EVERYTHING without
   * interfering with the timing of the system.
   */
  struct USBPHY *phy = &defaultUsbPhy;
  usbCaptureI(phy);
  /* Clear all pending interrupts on this port. */
  PORTA->ISFR = 0xFFFFFFFF;
}

void usbPhyDetach(struct USBPHY *phy) {

  /* Set both lines to 0 (clear both D+ and D-) */
  *(phy->usbdpCAddr) = phy->usbdpMask;
  *(phy->usbdnCAddr) = phy->usbdnMask;

  /* Set both lines to output */
  *(phy->usbdpDAddr) |= phy->usbdpMask;
  *(phy->usbdnDAddr) |= phy->usbdnMask;
}

void usbPhyAttach(struct USBPHY *phy) {

  /* Set both lines to input */
  *(phy->usbdpDAddr) &= ~phy->usbdpMask;
  *(phy->usbdnDAddr) &= ~phy->usbdnMask;
}

void usbPhyDetachDefault(void) {
  usbPhyDetach(usbPhyDefaultPhy());
}

void usbPhyAttachDefault(void) {
  usbPhyAttach(usbPhyDefaultPhy());
}
