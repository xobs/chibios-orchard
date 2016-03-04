#include <string.h>
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

#define MAX_PACKET_BUFFERS 128
#define USB_PACKET_BUFFER_MASK 0x7f

static struct usb_packet usb_packets[MAX_PACKET_BUFFERS];
static uint8_t usb_packet_write_head;
static uint8_t usb_packet_read_head;

static int stats_errors;
static int stats_underflow;
static int stats_crc5_errors;
static int stats_crc16_errors;
static int stats_illegal_pids;
static int stats_num_packets;

static const uint32_t crc5Table4[] =
{
  0x00, 0x0E, 0x1C, 0x12, 0x11, 0x1F, 0x0D, 0x03,
  0x0B, 0x05, 0x17, 0x19, 0x1A, 0x14, 0x06, 0x08
};
static const uint32_t crc5Table0[] =
{
  0x00, 0x16, 0x05, 0x13, 0x0A, 0x1C, 0x0F, 0x19,
  0x14, 0x02, 0x11, 0x07, 0x1E, 0x08, 0x1B, 0x0D
};

//---------------
static int crc5Check(const unsigned char * data) {
  unsigned int b = data[0] ^ 0x1F;
  unsigned int crc = crc5Table4[b & 0x0F] ^ crc5Table0[(b >> 4) & 0x0F];
  b = data[1] ^ crc;
  return (crc5Table4[b & 0x0F] ^ crc5Table0[(b>>4) & 0x0F]) == 0x06;
}

/* On entry, addr=>start of data
             num = length of data
             crc = incoming CRC     */
static uint16_t crc16(const unsigned char *addr, int num, int crc, int poly) {
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

static int validate_token(const uint8_t *data, int size) {
  return 0;
  if (size != 2)
    return -1;
  return crc5Check(data);
}

static int validate_data(const uint8_t *data, int size) {
  uint16_t result;
  uint16_t compare;

  if (size < 2)
    return -1;

  result = crc16(data, size - 2, 0xffff, 0x8005);
  compare = ((data[size - 2] << 8) & 0xff00) | ((data[size - 1] << 0) & 0x00ff);

  return compare - result;
}

const char *usbPidToStr(uint8_t pid) {
  switch (pid) {

    /* Data packets */
    case USB_PID_DATA0:     return "DATA0";
    case USB_PID_DATA1:     return "DATA1";
    case USB_PID_DATA2:     return "DATA2";
    case USB_PID_MDATA:     return "MDATA";

    /* Token packets, with CRC-5 */
    case USB_PID_OUT:       return "OUT";
    case USB_PID_IN:        return "IN";
    case USB_PID_SOF:       return "SOF";
    case USB_PID_SETUP:     return "SETUP";

    /* One-byte packets, no CRC required */
    case USB_PID_ACK:       return "ACK";
    case USB_PID_NAK:       return "NAK";
    case USB_PID_NYET:      return "NYET";
    case USB_PID_STALL:     return "STALL";

    /* Special cases (also no CRC?) */
    case USB_PID_SPLIT:     return "SPLIT";
    case USB_PID_PING:      return "PING";
    case USB_PID_ERR:       return "ERR";

    case USB_PID_RESERVED:  return "RESERVED";

    default:                return "UNKNOWN";
  }
}

int usbMacResetStatistics(void) {
  stats_underflow = 0;
  stats_errors = 0;
  stats_crc5_errors = 0;
  stats_crc16_errors = 0;
  stats_illegal_pids = 0;
  stats_num_packets = 0;
  return 0;
}

void usbMacGetStatistics(struct usb_mac_statistics *stats) {

  stats->num_packets = stats_num_packets;
  stats->errors = stats_errors;
  stats->underflow = stats_underflow;
  stats->crc5_errors = stats_crc5_errors;
  stats->crc16_errors = stats_crc16_errors;
  stats->illegal_pids = stats_illegal_pids;
  stats->read_head = usb_packet_read_head;
  stats->write_head = usb_packet_write_head;
  stats->buffer_size = MAX_PACKET_BUFFERS;
}

static void usb_send_ack(void) {
    uint8_t ak = USB_PID_ACK;
    usbPhyWriteDirect(&ak, 1);
}

int usbMacInsert(uint8_t *temp_packet, int packet_size) {

  int next_packet_idx = (usb_packet_write_head + 1) & USB_PACKET_BUFFER_MASK;
  if (next_packet_idx == usb_packet_read_head) {
    stats_underflow++;
    return -1;
  }

  /* Figure out which PID we're dealing with */
  switch (temp_packet[0]) {

  /* Data packets, with CRC-16 */
  case 0xc3: /* DATA0 */
  case 0x4b: /* DATA1 */
  case 0x87: /* DATA2 */
  case 0x0f: /* MDATA */

    usb_send_ack();
    if (validate_data(temp_packet + 1, packet_size - 1)) {
      stats_crc16_errors++;
      return -2;
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];

    /* Packet size less CRC-16 and 1-byte PID */
    usb_packets[usb_packet_write_head].size = packet_size - 3;
    memcpy(usb_packets[usb_packet_write_head].data,
           temp_packet + 1,
           packet_size - 3);

    break;

  /* Token packets, with CRC-5 */
  case 0xe1: /* OUT */
  case 0x69: /* IN */
    usb_send_ack();
  case 0xa5: /* SOF */
  case 0x2d: /* SETUP */
    /* Packet size is 8-bit PID + 11 bits of data + CRC-5 */
    if (validate_token(temp_packet + 1, packet_size - 1)) {
      stats_crc5_errors++;
#warning "CRC-5 failed, copying packet anyway"
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];

    /* Packet size less 1-byte PID */
    usb_packets[usb_packet_write_head].size = packet_size - 1;
    memcpy(usb_packets[usb_packet_write_head].data,
           temp_packet + 1,
           packet_size - 1);
    break;

  /* One-byte packets, no CRC required */
  case 0xd2: /* ACK */
  case 0x5a: /* NAK */
  case 0x96: /* NYET */
  case 0x1e: /* STALL */
    /* Packet size is 8-bit PID only */
    if (packet_size != 1) {
      stats_errors++;
      return -1;
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];
    usb_packets[usb_packet_write_head].size = 0;
    break;

  /* Special cases (also no CRC?) */
  case 0x78: /* SPLIT */
  case 0xb4: /* PING */
  case 0x3c: /* ERR */
    if (packet_size != 2) {
      stats_errors++;
      return -1;
    }
    usb_packets[usb_packet_write_head].pid = temp_packet[0];
    usb_packets[usb_packet_write_head].size = 0;
    break;

  default:
    stats_illegal_pids++;
    return -1;
  }

  /* Packet processing was successful, move the write head forward by one */
  stats_num_packets++;
  usb_packet_write_head = next_packet_idx;

  return 1;
}

const struct usb_packet *usbMacGetPacket(void) {
  if (usb_packet_read_head == usb_packet_write_head)
    return NULL;

#warning Lock here
  struct usb_packet *packet = &usb_packets[usb_packet_read_head];
  usb_packet_read_head = (usb_packet_read_head + 1) & USB_PACKET_BUFFER_MASK;
#warning Unlock here
  return packet;
}

int usbMacSendPacket(const struct usb_packet *packet) {
  uint8_t buffer[1 + 8 + 2]; /* PID + data + CRC-16 (worst-case) */
  int buffer_size;
  uint16_t crc16_calc;

  memset(buffer, 0, sizeof(buffer));
  buffer_size = -1;

  switch (packet->pid) {
  case 0xf0: /* MDATA */
  case 0xe1: /* DATA2 */
  case 0xd2: /* DATA1 */
  case 0xc3: /* DATA0 */
    buffer[0] = packet->pid;
    memcpy(buffer + 1, packet->data, packet->size);
    crc16_calc = crc16(buffer + 1, packet->size, 0xffff, 0x8005);
    buffer[9] = crc16_calc >> 8;
    buffer[10] = crc16_calc;
    buffer_size = 11;
    break;

  /* Token packets, with CRC-5 */
  case 0xb4: /* SETUP */
  case 0xa5: /* SOF */
  case 0x96: /* IN */
  case 0x87: /* OUT */
#warning "Implement CRC-5"
    return -3;
    break;

  /* One-byte packets, no CRC required */
  case 0x78: /* STALL */
  case 0x69: /* NYET */
  case 0x5a: /* NAK */
  case 0x4b: /* ACK */
    buffer[0] = packet->pid;
    buffer_size = 1;
    break;

  /* Special cases (also no CRC?) */
  case 0x3c: /* PRE */
  case 0x2d: /* PING */
  case 0x1e: /* SPLIT */
  case 0x0f: /* RESERVED */
    buffer[0] = packet->pid;
    buffer_size = 1;
    break;

  default:
    return -1;
  }

  if (buffer_size < 0)
    return -2;

  return usbPhyQueue(buffer, buffer_size);
}
