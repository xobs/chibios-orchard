#include <string.h>

#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

#define BIT_BUFFER_SIZE 0 + \
    + 8             /* Sync */                          \
    + 8             /* PID */                           \
    + (8 * 8)       /* Data */                          \
    + 16            /* CRC16 */                         \
    + 2             /* SE0 End-of-packet */             \
    + 8             /* Extra padding (unnecessary?) */
#define MAX_SEND_QUEUES 16
#define MAX_SEND_QUEUES_MASK 0xf

/* Outgoing queues */
static uint8_t send_queues[MAX_SEND_QUEUES][BIT_BUFFER_SIZE];
static uint8_t send_queue_sizes[MAX_SEND_QUEUES];
static uint8_t send_queue_write_head;
static uint8_t send_queue_read_head;

static int usb_initialized;
event_source_t usb_phy_data_available;
static thread_reference_t usb_thread;

static int stats_errors;
static int stats_overflow;
static int stats_timeout;
static int stats_num_packets;
static int stats_no_end_of_sync;
static int stats_no_end_of_frame;
uint32_t stats_timestamps[256];
uint32_t stats_timestamps_offset;
#define stats_timestamps_offset_mask 0xff

#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */

const uint8_t bit_reverse_table_256[] = {
#define R2(n)     n,     n + 2*64,     n + 1*64,     n + 3*64
#define R4(n) R2(n), R2(n + 2*16), R2(n + 1*16), R2(n + 3*16)
#define R6(n) R4(n), R4(n + 2*4 ), R4(n + 1*4 ), R4(n + 3*4 )
  R6(0), R6(2), R6(1), R6(3)
#undef R2
#undef R4
#undef R6
};

struct USBPHY {
  uint32_t padding[2];
  uint32_t scratch[6];
  volatile void *usbdpIAddr;
  volatile void *usbdpSAddr;
  volatile void *usbdpCAddr;
  volatile void *usbdpDAddr;
  uint32_t usbdpShift;

  volatile void *usbdnIAddr;
  volatile void *usbdnSAddr;
  volatile void *usbdnCAddr;
  volatile void *usbdnDAddr;
  uint32_t usbdnShift;

  uint32_t usbdpMask;
  uint32_t usbdnMask;
} __attribute__((packed));

enum state {
  state_se0,
  state_k,
  state_j,
  state_se1,
};

//#define USB_PHY_LL_DEBUG
static struct USBPHY usbPhyPins = {
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

static struct USBPHY usbPhyTestPins = {
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
};

static struct USBPHY usbPhyTestPatternPins = {
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

int usbPhyResetStatistics(void) {
  stats_errors = 0;
  stats_num_packets = 0;
  stats_overflow = 0;
  stats_timeout = 0;
  stats_no_end_of_sync = 0;
  stats_no_end_of_frame = 0;
  stats_timestamps_offset = 0;
  return 0;
}

void usbPhyGetStatistics(struct usb_phy_statistics *stats) {
  stats->num_packets = stats_num_packets;
  stats->errors = stats_errors;
  stats->overflow = stats_overflow;
  stats->timeout = stats_timeout;
  stats->no_end_of_sync = stats_no_end_of_sync;
  stats->no_end_of_frame = stats_no_end_of_frame;
  stats->out_read_head = send_queue_read_head;
  stats->out_write_head = send_queue_write_head;
  stats->out_buffer_size = MAX_SEND_QUEUES;
  stats->timestamps = stats_timestamps;
  stats->timestamp_count = stats_timestamps_offset;
}

int usbPhyQueue(const uint8_t *buffer, int buffer_size) {

  send_queue_sizes[send_queue_write_head] = buffer_size;
  memcpy(send_queues[send_queue_write_head], buffer, buffer_size);
  send_queue_write_head = (send_queue_write_head + 1) & MAX_SEND_QUEUES_MASK;
  osalSysLock();
  osalThreadResumeI(&usb_thread, MSG_OK);
  osalSysUnlock();
  return 0;
}

int usbCaptureTest(uint8_t samples[11]) {
  int bytes_read;

  /* Toggle the green LED */
  *((volatile uint32_t *)0xf80000cc) = 0x80;

  bytes_read = usbPhyRead(&usbPhyTestPins, samples);

  if (bytes_read < 0) {
    if (bytes_read == -1)
      stats_timeout++;
    else if (bytes_read == -2)
      stats_overflow++;
    else if (bytes_read == -3)
      stats_no_end_of_sync++;
    else
      stats_errors++;
    goto err;
  }
  stats_num_packets++;

  //stats_timestamps[stats_timestamps_offset++] = 0x00010000 | SysTick->VAL;
  //stats_timestamps_offset &= stats_timestamps_offset_mask;
  usbMacInsert(samples, bytes_read);

  return bytes_read;

err:
  return -1;
}

int usbCapture(uint8_t samples[11]) {
  int bytes_read;

  /* Toggle the green LED */
  *((volatile uint32_t *)0xf80000cc) = 0x80;

  bytes_read = usbPhyRead(&usbPhyPins, samples);

  if (bytes_read < 0) {
    if (bytes_read == -1)
      stats_timeout++;
    else if (bytes_read == -2)
      stats_overflow++;
    else if (bytes_read == -3)
      stats_no_end_of_sync++;
    else
      stats_errors++;
    goto err;
  }
  stats_num_packets++;

  //stats_timestamps[stats_timestamps_offset++] = 0x00010000 | SysTick->VAL;
  //stats_timestamps_offset &= stats_timestamps_offset_mask;
  usbMacInsert(samples, bytes_read);

  return bytes_read;

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
void usbStateTransitionI(void) {

  uint8_t samples[11];
  int bytes_read;

  bytes_read = usbCapture(samples);

  return;

err:
  ;
  OSAL_IRQ_PROLOGUE();
  ;
  OSAL_IRQ_EPILOGUE();
  return;
}

int usbPhyWriteDirect(const uint8_t *buffer, int size) {
  //stats_timestamps[stats_timestamps_offset++] = 0x00020000 | SysTick->VAL;
  //stats_timestamps_offset &= stats_timestamps_offset_mask;
  return usbPhyWrite(&usbPhyPins, buffer, size);
}

static int test_num = 0;
void usbPhyWriteTest(void) {
  uint8_t buffer[] = {
//    0xC3, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00, 0xDD, 0x94,
//    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
//    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//    0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 
//    0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
//    0xc3, 0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0xaa, 0x55,
    0xd2
  };
  usbPhyWrite(&usbPhyTestPatternPins, buffer, sizeof(buffer));
  test_num++;
  if (test_num > 10)
    test_num = 0;
  //usbPhyWriteTestPattern(&usbPhyTestPatternPins);
}

static THD_WORKING_AREA(waUsbBusyPollThread, 256);
static THD_FUNCTION(usb_busy_poll_thread, arg) {

  (void)arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
    osalSysLock();
    (void) osalThreadSuspendS(&usb_thread);
    osalSysUnlock();

    if (usb_initialized)
      chEvtBroadcast(&usb_phy_data_available);
  }

  return;
}

void usbInit(void) {
  chEvtObjectInit(&usb_phy_data_available);
  usb_initialized = 1;
  chThdCreateStatic(waUsbBusyPollThread, sizeof(waUsbBusyPollThread),
                    LOWPRIO, usb_busy_poll_thread, NULL);
}

OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {
  /* Clear all pending interrupts on this port. */
  PORTA->ISFR = 0xFFFFFFFF;
  usbStateTransitionI();
}
