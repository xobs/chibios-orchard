#include <string.h>

#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

uint32_t stats_timestamps[512];
uint32_t *stats_timestamps_ptr = stats_timestamps;
uint32_t stats_counter;

#define ADD_EVENT(phy, byte, tag)
/*
#define ADD_EVENT(phy, byte, tag) \
    *stats_timestamps_ptr++ = ((byte & 0xff) << 24) \
                            | ((tag & 0xff) << 16) \
                            | (SysTick->VAL & 0xffff);
*/
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
  .usbdpIAddr = &FGPIOB->PDIR,
  .usbdpSAddr = &FGPIOB->PSOR,
  .usbdpCAddr = &FGPIOB->PCOR,
  .usbdpDAddr = &FGPIOB->PDDR,
  .usbdpMask  = (1 << 0),
  .usbdpShift = 0,

  /* PTA4 */
  .usbdnIAddr = &FGPIOA->PDIR,
  .usbdnSAddr = &FGPIOA->PSOR,
  .usbdnCAddr = &FGPIOA->PCOR,
  .usbdnDAddr = &FGPIOA->PDDR,
  .usbdnMask  = (1 << 4),
  .usbdnShift = 4,
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
  memset(&phy->stats, 0, sizeof(phy->stats));
  phy->stats.out_buffer_size = MAX_SEND_QUEUES;
//  phy->stats.timestamp_mask = 0x1ff;
  return 0;
}

const struct usb_phy_statistics *usbPhyGetStatistics(struct USBPHY *phy) {
  return &phy->stats;
}

int usbPhyInitialized(struct USBPHY *phy) {
  if (!phy)
    return 0;

  return phy->initialized;
}

int usbPhyQueue(struct USBPHY *phy, const uint8_t *buffer, int buffer_size) {

  phy->queue_sizes[phy->write_head] = buffer_size;
  memcpy(phy->queues[phy->write_head], buffer, buffer_size);
  phy->write_head = (phy->write_head + 1) & MAX_SEND_QUEUES_MASK;

  /* Wake up the USB writing thread to actually send the data out */
  osalSysLock();
  osalThreadResumeI(&defaultUsbPhy.thread, MSG_OK);
  osalSysUnlock();
  return 0;
}

static int is_interesting;
int usbCaptureI(struct USBPHY *phy) {
  int bytes_read;
  uint8_t samples[12];
  int ret;
  int capture_tries = 0;

  /* Toggle the green LED */
  *((volatile uint32_t *)0xf80000cc) = 0x80;

capture_again:

  ADD_EVENT(phy, capture_tries, RD_EVENT_START);
  bytes_read = usbPhyReadI(phy, samples);
  ADD_EVENT(phy, bytes_read, RD_EVENT_END);

  /* If we've got more captures to try, try again */
  if ((bytes_read < 0) && (capture_tries --> 0))
      goto capture_again;

  if (bytes_read <= 0) {
    if (bytes_read == -1)
      phy->stats.timeout++;

    else if (bytes_read == -2)
      phy->stats.overflow++;

    else if (bytes_read == 0)
      phy->stats.empty++;

    /* No end-of-sync can be returned when we're mis-aligned */
    else if (bytes_read == -3)
      phy->stats.no_end_of_sync++;

    else
      phy->stats.errors++;
    goto err;
  }

  ADD_EVENT(phy, samples[bytes_read - 1], INSERT_EVENT_START);
  ret = usbMacInsertRevI(phy->mac, samples, bytes_read);
  ADD_EVENT(phy, ret, INSERT_EVENT_END);
  is_interesting = 1;

  /* If the Insert operation has indicated we're to expect another packet,
   * capture another packet.
   */
  if (!ret) {
    capture_tries = 10;
    goto capture_again;
  }

  return ret;

err:
  return -1;
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
  is_interesting = 0;

  ADD_EVENT(phy, 0, IRQ_EVENT_START);

  OSAL_IRQ_PROLOGUE();
  osalSysLockFromISR();
  ret = usbCaptureI(&defaultUsbPhy);
  if (ret >= 0 && phy->initialized)
    osalThreadResumeI(&phy->thread, MSG_OK);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();

  ADD_EVENT(phy, ret, IRQ_EVENT_END);

  if (!is_interesting)
    stats_timestamps_ptr -= 4;
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
static uint32_t bit_num_lut[12][7] = {
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

int usbPhyWritePrepare(struct USBPHYInternalData *internal,
                       const uint32_t buffer[3],
                       int size) {

  uint32_t *num_bits = bit_num_lut[size];
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

int usbPhyWriteDirectI(struct USBPHY *phy, const uint8_t buffer[12], int size) {

  struct USBPHYInternalData data;

  ADD_EVENT(phy, buffer[0], WRPREP_EVENT_START);
  usbPhyWritePrepare(&data, buffer, size);
  ADD_EVENT(phy, 0, WRPREP_EVENT_END);

  ADD_EVENT(phy, buffer[0], WR_EVENT_START);
  int ret = usbPhyWriteI(phy, &data);
  ADD_EVENT(phy, ret, WR_EVENT_END);
  return ret;
}

int usbPhyWritePreparedI(struct USBPHY *phy,
                         struct USBPHYInternalData *data) {
  ADD_EVENT(phy, 0, WR_EVENT_START);
  int ret = usbPhyWriteI(phy, data);
  ADD_EVENT(phy, ret, WR_EVENT_END);
  return ret;
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

  usbPhyWritePrepare(&data, buffer, test_num);
  usbPhyWriteI(phy, &data);

  test_num++;
  if (test_num > sizeof(buffer))
    test_num = 1;
}

static THD_FUNCTION(usb_busy_poll_thread, arg) {

  struct USBPHY *phy = arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
    osalSysLock();
    (void) osalThreadSuspendS(&phy->thread);
    osalSysUnlock();

    usbMacProcess(phy->mac);
    if (phy->initialized)
      chEvtBroadcast(&phy->data_available);
  }

  return;
}

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac) {

  phy->mac = mac;
  usbMacSetPhy(mac, phy);
  chEvtObjectInit(&phy->data_available);
  phy->initialized = 1;
  usbPhyDetach(phy);
  usbPhyResetStatistics(phy);
  chThdCreateStatic(phy->waThread, sizeof(phy->waThread),
                    HIGHPRIO, usb_busy_poll_thread, phy);
}

struct USBPHY *usbPhyDefaultPhy(void) {

  return &defaultUsbPhy;
}

struct USBPHY *usbPhyTestPhy(void) {

  return &testWriteUsbPhy;
}

OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {
  /* Clear all pending interrupts on this port. */
  usbStateTransitionI();
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
