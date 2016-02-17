
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
#define MAX_BIT_BUFFERS 16
#define MAX_BIT_BUFFERS_MASK 0xf

/* Outgoing queues */
static uint8_t bit_queues[MAX_BIT_BUFFERS][BIT_BUFFER_SIZE];
static uint8_t bit_queue_sizes[MAX_BIT_BUFFERS];
static uint8_t bit_queue_write_head;
static uint8_t bit_queue_read_head;

/* Incoming buffers */
static uint8_t bit_buffers[MAX_BIT_BUFFERS][BIT_BUFFER_SIZE];
static uint8_t bit_buffer_sizes[MAX_BIT_BUFFERS];
static uint8_t bit_buffer_write_head;
static uint8_t bit_buffer_read_head;

static int usb_initialized;
event_source_t usb_phy_data_available;

static int stats_errors;
static int stats_underflow;
static int stats_overflow;
static int stats_timeout;
static int stats_num_packets;

#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */

struct USBPHY {
  volatile void *usbdpIAddr;
  volatile void *usbdpSAddr;
  volatile void *usbdpCAddr;
  volatile void *usbdpDAddr;
  uint32_t usbdpMask;
  uint32_t usbdpShift;

  volatile void *usbdnIAddr;
  volatile void *usbdnSAddr;
  volatile void *usbdnCAddr;
  volatile void *usbdnDAddr;
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

static const struct USBPHY usbPhy = {
#if !defined(USB_PHY_LL_DEBUG)
  /* PTB0 */
  .usbdpIAddr = &FGPIOB->PDIR,
  .usbdpSAddr = &FGPIOB->PSOR,
  .usbdpCAddr = &FGPIOB->PCOR,
  .usbdpDAddr = &FGPIOB->PDDR,
  .usbdpMask = (1 << 0),
  .usbdpShift = 0,

  /* PTA4 */
  .usbdnIAddr = &FGPIOA->PDIR,
  .usbdnSAddr = &FGPIOA->PSOR,
  .usbdnCAddr = &FGPIOA->PCOR,
  .usbdnDAddr = &FGPIOA->PDDR,
  .usbdnMask = (1 << 4),
  .usbdnShift = 4,
#else
  /* Pins J21 and J19 */
  /* PTD5 */
  .usbdpIAddr = &FGPIOD->PDIR,
  .usbdpSAddr = &FGPIOD->PSOR,
  .usbdpCAddr = &FGPIOD->PCOR,
  .usbdpDAddr = &FGPIOD->PDDR,
  .usbdpMask = (1 << 5),
  .usbdpShift = 5,

  /* PTD6 */
  .usbdnIAddr = &FGPIOD->PDIR,
  .usbdnSAddr = &FGPIOD->PSOR,
  .usbdnCAddr = &FGPIOD->PCOR,
  .usbdnDAddr = &FGPIOD->PDDR,
  .usbdnMask = (1 << 6),
  .usbdnShift = 6,
#endif

  .ticks = 48000000 / USB_LS_RATE,
};

/* Convert the PHY data to MAC, removing headers and bit stuffing,
 * and converting bits into bytes.
 */
static int usb_convert_phy_to_mac(uint8_t *input, uint8_t *output, int bits) {

  int bytes;
  int bit_num;
  uint8_t last_bit;
  int run_length;
  uint8_t acc;  /* Bit accumulator */
  int byte_pos;
  uint8_t bit;
  
  bytes = 0;

  /* Look for the end-of-sync indicator, which is a duplicated bit (0b11) */
  last_bit = input[0];
  for (bit_num = 1; bit_num < bits; bit_num++) {
    if (input[bit_num] == last_bit)
      break;
    last_bit = input[bit_num];
  }

  /* Error! Couldn't find end-of-sync */
  if (bit_num >= bits)
    return -1;

//  for (bytes = 0; bit_num < bits; bytes++, bit_num++)
//    output[bytes] = input[bit_num];

  /* 00:KKJJJKKJKJKJKJKJKJKJKKJKJ */
  /*     10110100 0000000000001000 */

  /* Convert bits to bytes, and unstuff runs */
  run_length = 0;
  last_bit = input[bit_num++]; /* Don't need to re-sample bit, since it's the same */
  while (bit_num < bits) {

    acc = 0;  /* Reset the accumulator */

//    for (byte_pos = 0; byte_pos < 8; byte_pos++) {
    for (byte_pos = 7; byte_pos >= 0; byte_pos--) {
      if (bit_num > bits)
        return -2;
      bit = input[bit_num++];

      /* No transition equals a 1 */
      if (bit == last_bit) {
        acc |= (1 << byte_pos);
        run_length++;
      }

      /* A transition indicates a 0, or a stuffed bit */
      else {
        /* A change indicates a 0, or could indicate a stuffed bit */
        if (run_length > 6)
          byte_pos--; /* Redo this bit, because it's stuffed */
        else
          acc |= (0 << byte_pos);

        /* It was a transition, so reset the run length counter */
        run_length = 0;
      }
      last_bit = bit;
    }

    output[bytes++] = acc;
  }

  return bytes;
}

static int usb_convert_mac_to_phy(const uint8_t *input,
                                  uint8_t *output,
                                  int bytes) {

  int out_bits;
  int input_byte;
  int input_bit;
  int run_length;
  enum state last_state;
  enum state cur_state;
  
  out_bits = 0;
  input_bit = 0;
  input_byte = 0;
  run_length = 0;

  /* Generate the start-of-frame pattern */
  output[out_bits++] = state_k;
  output[out_bits++] = state_j;
  output[out_bits++] = state_k;
  output[out_bits++] = state_j;
  output[out_bits++] = state_k;
  output[out_bits++] = state_j;
  output[out_bits++] = state_k;
  output[out_bits++] = state_k;

  last_state = cur_state = state_k;
  run_length = 2;

  for (input_byte = 0; input_byte < bytes; input_byte++) {
    for (input_bit = 7; input_bit >= 0; input_bit--) {
      if (input[input_byte] & (1 << input_bit)) {
        cur_state = last_state;
        run_length++;

        /* Bit-stuff.  More than 6 1s in a row get turned into a 0 */
        if (run_length > 6) {
          if (cur_state == state_j)
            cur_state = state_k;
          else
            cur_state = state_j;
          run_length = 0;
        }
      }
      else {
        run_length = 0;
        if (cur_state == state_j)
          cur_state = state_k;
        else
          cur_state = state_j;
      }
      output[out_bits++] = cur_state;
      last_state = cur_state;
    }
  }

  /* Generate the end-of-frame pattern */
  output[out_bits++] = state_se0;
  output[out_bits++] = state_se0;
  return out_bits;

  return bytes;
}

int usbPhyResetStatistics(void) {
  stats_errors = 0;
  stats_underflow = 0;
  stats_num_packets = 0;
  stats_overflow = 0;
  stats_timeout = 0;
  return 0;
}

void usbPhyGetStatistics(struct usb_phy_statistics *stats) {
  stats->num_packets = stats_num_packets;
  stats->errors = stats_errors;
  stats->underflow = stats_underflow;
  stats->overflow = stats_overflow;
  stats->timeout = stats_timeout;
  stats->in_read_head = bit_buffer_read_head;
  stats->in_write_head = bit_buffer_write_head;
  stats->in_buffer_size = MAX_BIT_BUFFERS;
  stats->out_read_head = bit_queue_read_head;
  stats->out_write_head = bit_queue_write_head;
  stats->out_buffer_size = MAX_BIT_BUFFERS;
}

/* Convert one PHY packet to a USB MAC packet */
int usb_convert_one_phy_to_mac(void) {

  uint8_t temp_packet[11]; /* 8-bit PID, 8-bytes of data, 2-byte checksum */
  int packet_size;
  int ret;

  /* No packets to process */
  if (bit_buffer_read_head == bit_buffer_write_head)
    return 0;

  packet_size = usb_convert_phy_to_mac(bit_buffers[bit_buffer_read_head],
                                      temp_packet,
                                      bit_buffer_sizes[bit_buffer_read_head]);
  /* Advance the read head */
  bit_buffer_read_head = (bit_buffer_read_head + 1) & MAX_BIT_BUFFERS_MASK;

  /* If the packet size is negative, then there weren't an even number of
   * bits in a byte, after unstuffing and removing the header.
   */
  if (packet_size < 0) {
    stats_errors++;
    return -1;
  }

  ret = usbMacInsert(temp_packet, packet_size);
  if (!ret)
    return ret;

  if (ret < 0)
    return ret;

  return ret;
}

int usbProcessIncoming(void) {
  
  int count;

  count = 0;
  while ((bit_buffer_read_head != bit_buffer_write_head) 
      /*&& (usb_packet_read_head != usb_packet_write_head)*/) {
    count++;
    usb_convert_one_phy_to_mac();
  }
  return count;
}

int usbPhyQueue(const uint8_t *buffer, int buffer_size) {

  int bit_length = usb_convert_mac_to_phy(buffer,
                                          bit_queues[bit_queue_write_head],
                                          buffer_size);
  if (bit_length <= 0)
    return bit_length;

  bit_queue_sizes[bit_queue_write_head] = bit_length;
  bit_queue_write_head = (bit_queue_write_head + 1) & MAX_BIT_BUFFERS_MASK;
  return 0;
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

  int bits_read;
  int next_write_pos;

  /* Toggle the green LED */
  *((volatile uint32_t *)0xf80000cc) = 0x80;

  if (!usb_initialized)
    return;

  next_write_pos = (bit_buffer_write_head + 1) & MAX_BIT_BUFFERS_MASK;
  if (next_write_pos == bit_buffer_read_head) {
    /* Fell off the end of the buffer.  Underflow. */
    stats_underflow++;
    goto err;
  }

  bits_read = usbPhyRead(&usbPhy,
                         bit_buffers[bit_buffer_write_head],
                         BIT_BUFFER_SIZE);
  if (bits_read < 0) {
    if (bits_read == -1)
      stats_timeout++;
    else if (bits_read == -2)
      stats_overflow++;
    else
      stats_errors++;
    goto err;
  }

  /* Store the bit size for conversion at a later time. */
  bit_buffer_sizes[bit_buffer_write_head] = bits_read;

  /* Increment the write head */
  bit_buffer_write_head = next_write_pos;

  /* If a packet is queued, write it out */
  if (bit_queue_read_head != bit_queue_write_head) {
    usbPhyWrite(&usbPhy,
                bit_queues[bit_queue_read_head],
                bit_queue_sizes[bit_queue_read_head]);
    bit_queue_read_head = (bit_queue_read_head + 1) & MAX_BIT_BUFFERS_MASK;
  }

err:
  return;
}

int usbPhyWriteDirect(const uint8_t *buffer, int size) {
  return usbPhyWrite(&usbPhy, buffer, size);
}

static THD_WORKING_AREA(waUsbBusyPollThread, 64);
static THD_FUNCTION(usb_busy_poll_thread, arg) {

  (void)arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
    chThdSleepMilliseconds(1);
    if (bit_buffer_write_head != bit_buffer_read_head)
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
