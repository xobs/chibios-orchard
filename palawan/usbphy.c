#include <string.h>

#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */

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

  samples = (uint8_t *)phy->byte_queue[phy->byte_queue_head];

  ret = usbPhyReadI(phy, (uint32_t *)samples);

  if (ret == USB_DIP_IN) {
    if (!phy->data_is_queued) {
      usbPhyWriteI(phy, &phyNak);
      goto out;
    }

    phy->byte_queue_head++;
    usbPhyWriteI(phy, &phy->queued_data);
    goto out;
  }
  else if (ret == USB_DIP_SETUP) {
    phy->byte_queue_head++;
    goto out;
  }
  else if (ret == USB_DIP_OUT) {
    phy->byte_queue_head++;
    goto out;
  }
  else if (ret == USB_DIP_ACK) {
    /* Allow the next byte to be sent */
    usbMacTransferSuccess(phy->mac);
    phy->byte_queue_head++;
    phy->data_is_queued = 0;
    goto out;
  }

  else if ((ret == USB_DIP_DATA0) || (ret == USB_DIP_DATA1)) {
    phy->byte_queue_head++;
    usbPhyWriteI(phy, &phyAck);
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

/*
   This LUT pre-computes various parameters that vary depending on the
   number of bits to copy.

   The internal data copies up to 11 bytes of USB packet data into three
   "cells" in the USBPHY struct.  Each cell contains up to 32-bits of data,
   plus an indicator of how much data is present in that cell.

   The first three bytes of the LUT are the number of bits in each of the
   three cells.  As each cell has a different amount of data, each cell
   gets its own bit count.

   The next byte of LUT indicates which cell to start on.

   The final three bytes of LUT indicate where, in the three-word buffer,
   each cell will pull data from.
 */
static uint8_t bit_num_lut[12][7] = {
  /* Bits per cell    Starting cell   Source array */
  {  0,  0,  0,       0,              0, 0, 0 },

  {  8,  0,  0,       1,              0, 0, 0 },
  { 16,  0,  0,       1,              0, 0, 0 },
  { 24,  0,  0,       1,              0, 0, 0 },
  { 32,  0,  0,       1,              0, 0, 0 },

  {  8, 32,  0,       2,              1, 0, 0 },
  { 16, 32,  0,       2,              1, 0, 0 },
  { 24, 32,  0,       2,              1, 0, 0 },
  { 32, 32,  0,       2,              1, 0, 0 },

  {  8, 32, 32,       3,              2, 1, 0 },
  { 16, 32, 32,       3,              2, 1, 0 },
  { 24, 32, 32,       3,              2, 1, 0 },
};

/* Expose the "rev" instruction to C */
#ifdef REVERSE_BITS
static inline __attribute__((always_inline)) uint32_t __rev(uint32_t val) {
  asm("rev %0, %1" : "=r" (val) : "r" (val) );
  return val;
}
#else /* Don't reverse bits */
#define __rev(x) x
#endif

static int usb_phy_write_prepare_internal(struct USBPHYInternalData *internal,
                                          const uint32_t buffer[3],
                                          int size) {
  uint8_t *num_bits = bit_num_lut[size];
  uint32_t *scratch = internal->scratch;

  /* Copy the number of bits in each slot from the LUT. */
  scratch[1] = *num_bits++;
  scratch[3] = *num_bits++;
  scratch[5] = *num_bits++;

  /* Copy the initial packet number over */
  scratch[6] = *num_bits++;

  /* Copy the actual words over, and invert them */
  scratch[0] = __rev(~buffer[*num_bits++]);
  scratch[2] = __rev(~buffer[*num_bits++]);
  scratch[4] = __rev(~buffer[*num_bits++]);
  
  return 0;
}

int usbPhyWritePrepare(struct USBPHY *phy,
                       const uint32_t buffer[3],
                       int size) {

  int ret;
  ret = usb_phy_write_prepare_internal(&phy->queued_data, buffer, size);
  phy->data_is_queued = 1;
  return ret;
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
  uint32_t test_num = sizeof(buffer);

  usbPhyWritePrepare(phy, (void *)buffer, test_num);
  usbPhyWriteI(phy, &phy->queued_data);

  test_num++;
  if (test_num > sizeof(buffer))
    test_num = 1;
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
  while (phy->byte_queue_tail != phy->byte_queue_head) {
    uint8_t *in_ptr = (uint8_t *)phy->byte_queue[phy->byte_queue_tail];
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
    phy->byte_queue_tail++;
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

  uint32_t buffer[3] = {};
  uint8_t *buffer_b = (uint8_t *)buffer;

  buffer_b[0] = USB_PID_ACK;
  usb_phy_write_prepare_internal(&phyAck, buffer, 1);

  buffer_b[0] = USB_PID_NAK;
  usb_phy_write_prepare_internal(&phyNak, buffer, 1);

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

  if (phy->byte_queue_tail != phy->byte_queue_head)
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
