
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "palawan.h"

#define BIT_BUFFER_SIZE 0 + \
    + 8             /* Sync */                          \
    + 8             /* PID */                           \
    + (8 * 8)       /* Data */                          \
    + 16            /* CRC16 */                         \
    + 2             /* SE0 End-of-packet */             \
    + 8             /* Extra padding (unnecessary?) */
#define MAX_BIT_BUFFERS 4
#define MAX_BIT_BUFFERS_MASK 0x3

static uint8_t bit_buffers[MAX_BIT_BUFFERS][BIT_BUFFER_SIZE];
static uint8_t bit_buffer_sizes[MAX_BIT_BUFFERS];
static uint8_t bit_buffer_write_head;
static uint8_t bit_buffer_read_head;

#define MAX_PACKET_BUFFERS 16
#define USB_PACKET_BUFFER_MASK 0xf
struct usb_packet {
  uint8_t pid;
  uint8_t data[8];
  uint8_t size; /* Not including pid (so may be 0) */
  /* Checksum omitted */
};

static struct usb_packet usb_packets[MAX_PACKET_BUFFERS];
static uint8_t usb_packet_write_head;
static uint8_t usb_packet_read_head;

static int phy_errors;
static int phy_underflow;
static int phy_packets;
static int mac_errors;
static int mac_underflow;
static int mac_crc5_errs;
static int mac_crc16_errs;
static int mac_illegal_pid;
static int mac_packets;

#define USB_FS_RATE 12000000 /* 12 MHz */
#define USB_LS_RATE (USB_FS_RATE / 8) /* 1.5 MHz */

struct USBPHY {
  volatile void *usbdpAddr;
  uint32_t usbdpMask;
  uint32_t usbdpShift;

  volatile void *usbdnAddr;
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

static struct USBPHY usbPhy = {
  /* PTB0 */
  .usbdpAddr = &FGPIOB->PDIR,
  .usbdpMask = (1 << 0),
  .usbdpShift = 0,

  /* PTA4 */
  .usbdnAddr = &FGPIOA->PDIR,
  .usbdnMask = (1 << 4),
  .usbdnShift = 4,

  .ticks = 48000000 / USB_LS_RATE,
};

const unsigned char crc5Table4[] =
{
  0x00, 0x0E, 0x1C, 0x12, 0x11, 0x1F, 0x0D, 0x03,
  0x0B, 0x05, 0x17, 0x19, 0x1A, 0x14, 0x06, 0x08
};
const unsigned char crc5Table0[] =
{
  0x00, 0x16, 0x05, 0x13, 0x0A, 0x1C, 0x0F, 0x19,
  0x14, 0x02, 0x11, 0x07, 0x1E, 0x08, 0x1B, 0x0D
};

//---------------
static int crc5Check(const unsigned char * data) {
  unsigned char b = data[0] ^ 0x1F;
  unsigned char crc = crc5Table4[b & 0x0F] ^ crc5Table0[(b >> 4) & 0x0F];
  b = data[1] ^ crc;
  return (crc5Table4[b & 0x0F] ^ crc5Table0[(b>>4) & 0x0F]) == 0x06;
}

/* On entry, addr=>start of data
             num = length of data
             crc = incoming CRC     */
static uint16_t crc16(const char *addr, int num, int crc, int poly)
{
  int i;

  for (; num > 0; num--)               /* Step through bytes in memory */
  {
    crc = crc ^ (*addr++ << 8);      /* Fetch byte from memory, XOR into CRC top byte*/
    for (i = 0; i < 8; i++)              /* Prepare to rotate 8 bits */
    {
      crc = crc << 1;                /* rotate */
      if (crc & 0x10000)             /* bit 15 was set (now bit 16)... */
        crc = (crc ^ poly) & 0xFFFF; /* XOR with XMODEM polynomic */
                                     /* and ensure CRC remains 16-bit value */
    }                              /* Loop for 8 bits */
  }                                /* Loop until num=0 */
  return ~crc;                      /* Return updated CRC */
}

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

static int validate_token(const uint8_t *data, int size) {
  if (size != 2)
    return -1;
  return crc5Check(data);
}

static int validate_data(const uint8_t *data, int size) {
  uint16_t result;
  uint16_t compare;

  result = crc16(data, size - 2, 0xffff, 0x8005);
  compare = ((data[size - 2] << 8) & 0xff00) | ((data[size - 1] << 0) & 0x00ff);

  return compare - result;
}

int usbResetStatistics(void) {
  phy_errors = 0;
  phy_underflow = 0;
  phy_packets = 0;
  mac_underflow = 0;
  mac_errors = 0;
  mac_crc5_errs = 0;
  mac_crc16_errs = 0;
  mac_illegal_pid = 0;
  mac_packets = 0;
}

void usbGetStatistics(int *phy_errors_, int *phy_underflow_,
                      int *mac_errors_, int *mac_underflow_,
                      int *mac_crc5_errs_, int *mac_crc16_errs_,
                      int *bit_buffer_read_head_, int *bit_buffer_write_head_,
                      int *usb_packet_read_head_, int *usb_packet_write_head_,
                      int *mac_illegal_pid_, int *mac_packets_,
                      int *phy_packets_) {
  if (phy_errors_)
    *phy_errors_ = phy_errors;

  if (phy_underflow_)
    *phy_underflow_ = phy_underflow;

  if (mac_errors_)
    *mac_errors_ = mac_errors;

  if (mac_undreflow_)
    *mac_underflow_ = mac_underflow;

  if (mac_crc5_errs_)
    *mac_crc5_errs_ = mac_crc5_errs;

  if (mac_crc16_errs_)
    *mac_crc16_errs_ = mac_crc16_errs;

  if (bit_buffer_read_head_)
    *bit_buffer_read_head_ = bit_buffer_read_head;

  if (bit_buffer_write_head_)
    *bit_buffer_write_head_ = bit_buffer_write_head;

  if (usb_packet_read_head_)
    *usb_packet_read_head_ = usb_packet_read_head;

  if (usb_packet_write_head_)
    *usb_packet_write_head_ = usb_packet_write_head;

  if (mac_illegal_pid_)
    *mac_illegal_pid_ = mac_illegal_pid;

  if (mac_packets_)
    *mac_packets_ = mac_packets;

  if (phy_packets_)
    *phy_packets_ = phy_packets;
}

static void usb_mac_insert(uint8_t *tmep_packet, int packet_size) {

  /* Figure out which PID we're dealing with */
  switch (temp_packet[0]) {

  /* Data packets, with CRC-16 */
  case 0xf0: /* MDATA */
  case 0xe1: /* DATA2 */
  case 0xd2: /* DATA1 */
  case 0xc3: /* DATA0 */
    if (validate_data(temp_packet + 1, packet_size - 1)) {
      mac_crc16_errs++;
#warning "CRC-16 failed, copying packet anyway"
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];

    /* Packet size less CRC-16 and 1-byte PID */
    usb_packets[usb_packet_write_head].size = packet_size - 3;
    memcpy(usb_packets[usb_packet_write_head],
           temp_packet + 1,
           packet_size - 3);
    break;

  /* Token packets, with CRC-5 */
  case 0xb4: /* SETUP */
  case 0xa5: /* SOF */
  case 0x96: /* IN */
  case 0x87: /* OUT */
    /* Packet size is 8-bit PID + 11 bits of data + CRC-5 */
    if (packet_size != 3) {
      mac_errors++;
      return -1;
    }
    if (validate_token(temp_packet + 1, packet_size - 1)) {
      mac_crc5_errs++;
#warning "CRC-5 failed, copying packet anyway"
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];

    /* Packet size less 1-byte PID */
    usb_packets[usb_packet_write_head].size = packet_size - 1;
    memcpy(usb_packets[usb_packet_write_head],
           temp_packet + 1,
           packet_size - 1);
    break;

  /* One-byte packets, no CRC required */
  case 0x78: /* STALL */
  case 0x69: /* NYET */
  case 0x5a: /* NAK */
  case 0x4b: /* ACK */
    /* Packet size is 8-bit PID only */
    if (packet_size != 1) {
      mac_errors++;
      return -1;
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];
    usb_packets[usb_packet_write_head].size = 0;
    break;

  /* Special cases (also no CRC?) */
  case 0x3c: /* PRE */
  case 0x2d: /* PING */
  case 0x1e: /* SPLIT */
  case 0x0f: /* RESERVED */
    if (packet_size != 2) {
      mac_errors++;
      return -1;
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];
    usb_packets[usb_packet_write_head].size = 0;
    break;

  default:
    mac_illegal_pid++;
    return -1;
  }

  mac_packets++;
  usb_packet_write_head = next_packet_idx;
  return 1;
}

/* Convert one PHY packet to a USB MAC packet */
int usb_convert_one_phy_to_mac(void) {

  uint8_t temp_packet[11]; /* 8-bit PID, 8-bytes of data, 2-byte checksum */
  int packet_size;
  int next_packet_idx;
  int ret;

  /* No packets to process */
  if (bit_buffer_read_head == bit_buffer_write_head)
    return 0;

  next_packet_idx = (usb_packet_write_head + 1) & USB_PACKET_BUFFER_MASK;
  if (next_packet_idx == usb_packet_read_head) {
    mac_underflow++;
    return -1;
  }

  packet_size = usb_convert_phy_to_mac(bit_buffers[bit_buffer_read_head],
                                      temp_packet,
                                      bit_buffer_sizes[bit_buffer_read_head]);
  /* Advance the read head */
  bit_buffer_read_head = (bit_buffer_read_head + 1) & MAX_BIT_BUFFERS_MASK;

  /* If the packet size is negative, then there weren't an even number of
   * bits in a byte, after unstuffing and removing the header.
   */
  if (packet_size < 0) {
    phy_errors++;
    return -1;
  }

  ret = usb_mac_insert(temp_packet, packet_size);
  if (!ret)
    return ret;

  if (ret < 0)
    return ret;
}

int usb_convert_all_phy_to_mac(void) {
  int count;

  count = 0;
  while (bit_buffer_read_head != bit_buffer_write_head) {
    count++;
    usb_convert_one_phy_to_mac();
  }
}

//  port_lock_from_isr();
void usbStateTransitionI(void) {

  int bits_read;

  /* Toggle the green LED */
  *((volatile uint32_t *)0xf80000cc) = 0x80;

  if (((bit_buffer_write_head + 1) & MAX_BIT_BUFFERS_MASK)
      == bit_biffer_read_head) {
    /* Fell off the end of the buffer.  Underflow. */
    phy_underflow++;
    goto err;
  }

  bits_read = usbPhyRead(&usbPhy,
                         bit_buffers[bit_buffer_write_head],
                         sizeof(BIT_BUFFER_SIZE));
  if (bits_read < 0) {
    phy_errors++;
    goto err;
  }

  /* Store the bit size for conversion at a later time. */
  bit_buffer_sizes[bit_buffer_write_head++] = bits_read;
  bit_buffer_write_head &= MAX_BIT_BUFFERS_MASK;

err:
//  PORTA->PCR[3] = PORTx_PCRn_MUX(7);
//  port_unlock_from_isr();
  return;
}

OSAL_IRQ_HANDLER(KINETIS_PORTA_IRQ_VECTOR) {
//  PORT_IRQ_PROLOGUE();

  /* Clear all pending interrupts on this port. */
  PORTA->ISFR = 0xFFFFFFFF;
  usbStateTransitionI();

//  PORT_IRQ_EPILOGUE();
}
