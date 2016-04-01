#include <string.h>

#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

#ifdef ENABLE_LOGGING
uint32_t stats_timestamps[512];
uint32_t *stats_timestamps_ptr = stats_timestamps;
uint32_t stats_counter;
#define ADD_EVENT(phy, byte, tag) \
    *stats_timestamps_ptr++ = ((byte & 0xff) << 24) \
                            | ((tag & 0xff) << 16) \
                            | (SysTick->VAL & 0xffff);
#define STATS(x) do { x; } while(0);
#else
#define ADD_EVENT(phy, byte, tag)
#define STATS(x) (void)0
#endif

/*
#define ADD_EVENT(phy, byte, tag) \
  do { \
    phy->stats.timestamps[phy->stats.timestamp_count++] = ((byte & 0xff) << 24) \
                                                        | ((tag & 0xff) << 16) \
                                                        | (SysTick->VAL & 0xffff); \
    phy->stats.timestamp_count &= phy->stats.timestamp_mask; \
  } while(0);
*/
#define WR_EVENT_START 1
#define WR_EVENT_END 2
#define RD_EVENT_START 3
#define RD_EVENT_END 4
#define INSERT_EVENT_START 5
#define INSERT_EVENT_END 6
#define IRQ_EVENT_START 7
#define IRQ_EVENT_END 8
#define WRPREP_EVENT_START 9
#define WRPREP_EVENT_END 10


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

int usbPhyResetStatistics(struct USBPHY *phy) {
#ifdef ENABLE_LOGGING
  memset(&phy->stats, 0, sizeof(phy->stats));
  phy->stats.out_buffer_size = MAX_SEND_QUEUES;
#else
  (void)phy;
#endif
  return 0;
}

const struct usb_phy_statistics *usbPhyGetStatistics(struct USBPHY *phy) {
#ifdef ENABLE_LOGGING
  return &phy->stats;
#else
  (void)phy;
  return NULL;
#endif
}

int usbPhyInitialized(struct USBPHY *phy) {
  if (!phy)
    return 0;

  return phy->initialized;
}

int usbPhyQueue(struct USBPHY *phy, const uint8_t *buffer, int buffer_size) {

#if 0
  phy->queue_sizes[phy->write_head] = buffer_size;
  memcpy(phy->queues[phy->write_head], buffer, buffer_size);
  phy->write_head = (phy->write_head + 1) & MAX_SEND_QUEUES_MASK;

  /* Wake up the USB writing thread to actually send the data out */
  osalSysLock();
  osalThreadResumeI(&defaultUsbPhy.thread, MSG_OK);
  osalSysUnlock();
#else
  (void)phy;
  (void)buffer;
  (void)buffer_size;
#endif
  return 0;
}

/*
static void wait_for_transition(struct USBPHY *phy, int tries) {
  uint32_t start = *((uint32_t *)phy->usbdnIAddr) & phy->usbdnMask;
  while (tries-- && (start == (*((uint32_t *)phy->usbdnIAddr) & phy->usbdnMask)))
    ;
}
*/

#ifdef ENABLE_LOGGING
static int is_interesting;
#endif

//int usbCaptureI(struct USBPHY *phy) __attribute__((section (".ramtext")));
static int usbCaptureI(struct USBPHY *phy) {

  uint8_t *samples;

  int ret;
  int capture_tries;

  /* Toggle the green LED */
  //*((volatile uint32_t *)0xf80000cc) = 0x80;

  capture_tries = 0;

capture_again:
  samples = (uint8_t *)phy->byte_queue[phy->byte_queue_head];

  ret = usbPhyReadI(phy, (uint32_t *)samples);
  PORTA->ISFR = 0xFFFFFFFF;

  if (ret == USB_DIP_IN) {
    phy->byte_queue_head++;
    if (phy->data_is_queued) {
      phy->data_is_queued = 0;
      usbPhyWriteI(phy, &phy->queued_data);
      capture_tries = 20;
      goto capture_again;
    }
    else
      usbPhyWriteI(phy, &phyNak);
    ret = 1;
  }
  else if (ret == USB_DIP_SETUP) {
    phy->byte_queue_head++;
    capture_tries = 20;
    goto capture_again;
  }
  else if (ret == USB_DIP_OUT) {
    phy->byte_queue_head++;
    goto capture_again;
  }

  else if ((ret == USB_DIP_DATA0) || (ret == USB_DIP_DATA1)) {
    phy->byte_queue_head++;
    usbPhyWriteI(phy, &phyAck);
    ret = 1;

    /* During SETUP, we will be asked, very quickly, to supply data.
     * Prepare to NAK this.
     */
    if (capture_tries)
      goto capture_again;
  }

  else if (ret <= 0) {
    if (capture_tries > 0) {
      /* If we've got more captures to try, try again */
      capture_tries--;
      goto capture_again;
    }

    return -1;
  }

  return ret;

#if 0
    if (ret == -1)
      STATS(phy->stats.timeout++);

    else if (ret == -2)
      STATS(phy->stats.overflow++);

    else if (ret == 0)
      STATS(phy->stats.empty++);

    /* No end-of-sync can be returned when we're mis-aligned */
    else if (ret == -3)
      STATS(phy->stats.no_end_of_sync++);

    else
      STATS(phy->stats.errors++);
#endif
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
static void usbStateTransitionI(void) {

  int ret;
  struct USBPHY *phy = &defaultUsbPhy;

  do {
    ret = usbCaptureI(phy);
  } while (PORTA->ISFR);

  /*
  if (ret >= 0 && phy->initialized) {
    osalSysLockFromISR();
    osalThreadResumeI(&phy->thread, MSG_OK);
    osalSysUnlockFromISR();
  }
  */

  return;
}

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

  phy->data_is_queued = 1;
  return usb_phy_write_prepare_internal(&phy->queued_data, buffer, size);
}

int usbPhyWriteDirectI(struct USBPHY *phy, const uint32_t buffer[3], int size) {

  struct USBPHYInternalData data;

  usbPhyWritePrepare(phy, buffer, size);
  usbPhyWriteI(phy, &data);
  return 0;
}

int usbPhyWritePrepared(struct USBPHY *phy,
                        struct USBPHYInternalData *data) {
  __disable_irq();
  usbPhyWriteI(phy, data);
  __enable_irq();
  return 0;
}

void usbPhyWriteTest(struct USBPHY *phy) {

  struct USBPHYInternalData data;
  uint8_t buffer[] = {
//    0xC3, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00, 0xDD, 0x94,
//    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
//    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
//    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
//    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
    0xc3, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xaa, 0x55,
//    0xd2
  };
  static uint32_t test_num = sizeof(buffer);

  usbPhyWritePrepare(phy, (void *)buffer, test_num);
  usbPhyWriteI(phy, &data);

  test_num++;
  if (test_num > sizeof(buffer))
    test_num = 1;
}

static THD_FUNCTION(usb_worker_thread, arg) {

  struct USBPHY *phy = arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
//    osalSysLock();
//    (void) osalThreadSuspendS(&phy->thread);
//    osalSysUnlock();
    if (phy->byte_queue_head == phy->byte_queue_tail) {
      chThdSleepMilliseconds(1);
      continue;
    }

    if (!phy->initialized)
      continue;

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

      // Finally, move on to the next packet
      phy->byte_queue_tail++;
    }
    //chEvtBroadcast(&phy->data_available);
  }

  return;
}

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac) {

  uint32_t buffer[3] = {};
  uint8_t *buffer_b = (uint8_t *)buffer;

  buffer_b[0] = USB_PID_ACK;
  usb_phy_write_prepare_internal(&phyAck, buffer, 1);

  buffer_b[0] = USB_PID_NAK;
  usb_phy_write_prepare_internal(&phyNak, buffer, 1);

  phy->mac = mac;
  usbMacSetPhy(mac, phy);
  chEvtObjectInit(&phy->data_available);
  phy->initialized = 1;
  usbPhyDetach(phy);
  usbPhyResetStatistics(phy);
  chThdCreateStatic(phy->waThread, sizeof(phy->waThread),
                    HIGHPRIO, usb_worker_thread, phy);
}

struct USBPHY *usbPhyDefaultPhy(void) {

  return &defaultUsbPhy;
}

struct USBPHY *usbPhyTestPhy(void) {

  return &testWriteUsbPhy;
}

OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {
  /* Clear all pending interrupts on this port. */
//  OSAL_IRQ_PROLOGUE();
  __disable_irq();
  do {
    usbStateTransitionI();
  } while (PORTA->ISFR);
  __enable_irq();
//  OSAL_IRQ_EPILOGUE();
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
