#include <string.h>

#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"

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
  if (ret <= 0)
    goto out;

  /* Save the byte counter for later inspection */
  samples[11] = ret;

  if (samples[0] == USB_PID_IN) {
    if (!phy->queued_size) {
      uint8_t pkt[] = {USB_PID_NAK};
      usbPhyWriteI(phy, pkt, sizeof(pkt));
      goto out;
    }

    phy->read_queue_head = (phy->read_queue_head + 1) & PHY_READ_QUEUE_MASK;
    usbPhyWriteI(phy, phy->queued_data, phy->queued_size);
    goto out;
  }
  else if (samples[0] == USB_PID_SETUP) {
    phy->read_queue_head = (phy->read_queue_head + 1) & PHY_READ_QUEUE_MASK;
    goto out;
  }
  else if (samples[0] == USB_PID_OUT) {
    phy->read_queue_head = (phy->read_queue_head + 1) & PHY_READ_QUEUE_MASK;
    goto out;
  }
  else if (samples[0] == USB_PID_ACK) {
    /* Allow the next byte to be sent */
    phy->queued_size = 0;
    usbMacTransferSuccess(phy->mac);
    phy->read_queue_head = (phy->read_queue_head + 1) & PHY_READ_QUEUE_MASK;
    goto out;
  }

  else if ((samples[0] == USB_PID_DATA0) || (samples[0] == USB_PID_DATA1)) {
    phy->read_queue_head = (phy->read_queue_head + 1) & PHY_READ_QUEUE_MASK;
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


#if defined(ENABLE_TEST_WRITE)
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

struct USBPHY *usbPhyTestPhy(void) {

  return &testWriteUsbPhy;
}
#else /* defined(ENABLE_TEST_WRITE) */

struct USBPHY *usbPhyTestPhy(void) {

  return NULL;
}
#endif

void usbPhyWorker(struct USBPHY *phy) {
  while (phy->read_queue_tail != phy->read_queue_head) {
    uint8_t *in_ptr = (uint8_t *)phy->read_queue[phy->read_queue_tail];
    int count = in_ptr[11];

    usbMacProcess(phy->mac, in_ptr, count);

    // Advance to the next packet
    phy->read_queue_tail = (phy->read_queue_tail + 1) & PHY_READ_QUEUE_MASK;
  }
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
