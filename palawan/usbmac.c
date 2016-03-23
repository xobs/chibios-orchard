#include <string.h>
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

static struct USBMAC default_mac;

static const uint8_t crc5Table4[] =
{
  0x00, 0x0E, 0x1C, 0x12, 0x11, 0x1F, 0x0D, 0x03,
  0x0B, 0x05, 0x17, 0x19, 0x1A, 0x14, 0x06, 0x08
};
static const uint8_t crc5Table0[] =
{
  0x00, 0x16, 0x05, 0x13, 0x0A, 0x1C, 0x0F, 0x19,
  0x14, 0x02, 0x11, 0x07, 0x1E, 0x08, 0x1B, 0x0D
};

// Taken from http://www.michael-joost.de/crc5check.pdf
static int crc5Check(const unsigned char * data) {
  unsigned char b = data[0] ^ 0x1F;
  unsigned char crc = crc5Table4[b & 0x0F] ^ crc5Table0[(b >> 4) & 0x0F];
  b = data[1] ^ crc;
  return (crc5Table4[b & 0x0F] ^ crc5Table0[(b>>4) & 0x0F]) != 0x06;
}

/* On entry, addr=>start of data
             num = length of data
             crc = incoming CRC     */
static uint16_t crc16(const unsigned char *addr, int num, int crc, int poly) {
  int i;

  for (; num > 0; num--)               /* Step through bytes in memory */
  {
    uint8_t c = *addr++;
    
//    crc = crc ^ (*addr++ << 8);      /* Fetch byte from memory, XOR into CRC top byte*/
    for (i = 0; i < 8; i++)              /* Prepare to rotate 8 bits */
    {
      if ((crc ^ c) & 1)
        crc = (crc >> 1) ^ poly;
      else
        crc >>= 1;
      c >>= 1;
    }                              /* Loop for 8 bits */
  }                                /* Loop until num=0 */
  return ~crc;                      /* Return updated CRC */
}

static int validate_token(const uint8_t *data, int size) {
  if (size != 2)
    return -1;
  return crc5Check(data);
}

static int validate_data(const uint8_t *data, int size) {
  uint16_t result;
  uint16_t compare;

  if (size < 2)
    return -1;

  result = crc16(data, size - 2, 0xffff, 0xa001);
  compare = ((data[size - 1] << 8) & 0xff00) | ((data[size - 2] << 0) & 0x00ff);

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

int usbMacResetStatistics(struct USBMAC *mac) {

  /* Reset all stats to 0, except buffer size */
  memset(&mac->stats, 0, sizeof(mac->stats));

  return 0;
}

const struct usb_mac_statistics *usbMacGetStatistics(struct USBMAC *mac) {

  if (!mac)
    return NULL;
  return &mac->stats;
}

static void usb_send_ack_i(struct USBMAC *mac) {
    uint8_t ak = USB_PID_ACK;
    usbPhyWriteDirectI(mac->phy, &ak, 1);
}

static void usb_send_nak_i(struct USBMAC *mac) {
    uint8_t ak = USB_PID_NAK;
    usbPhyWriteDirectI(mac->phy, &ak, 1);
}

static void usb_send_data_i(struct USBMAC *mac) {
  struct usb_packet packet;
  uint16_t crc;

  if (mac->data_buffer++ & 1)
    packet.pid = USB_PID_DATA0;
  else
    packet.pid = USB_PID_DATA1;

  /* Keep the packet size to 8 bytes max */
  packet.size = mac->data_out_left;
  if (mac->data_out_left > 8)
    packet.size = 8;
  mac->data_out_left -= packet.size;

  /* Copy over data bytes */
  memcpy(packet.data, mac->data_out, packet.size);
  mac->data_out += packet.size;

  /* Calculate and copy the crc16 */
  crc = crc16(packet.data, packet.size, 0xffff, 0x8005);
  packet.data[packet.size++] = crc >> 8;
  packet.data[packet.size++] = crc;

  /* Write the packet out, including the PID at the end */
  usbPhyWriteDirectI(mac->phy, packet.raw_data, packet.size + 1);
}

static int usb_copy_data(struct USBMAC *mac, uint8_t *data, unsigned int size) {

  /* Ensure we don't overflow mac->data_in */
  if ((mac->data_in_size + size) > sizeof(mac->data_in))
    return -1;

  /* Copy bytes to the data_in buffer */
  memcpy(mac->data_in + mac->data_in_size, data, size);
  mac->data_in_size += size;

  return 0;
}

extern const uint8_t bit_reverse_table_256[];

int usbMacInsertI(struct USBMAC *mac, uint8_t *temp_packet, int packet_size) {

  /* Figure out if the packet we're processing is valid, and if we expect
   * more data to follow.
   */
  switch (temp_packet[packet_size - 1]) {
    case USB_DIP_IN:
      mac->is_rx = 0; /* This packet will transmit data */

      /* If there is no data currently, NAK The request so they will retry */
      if ((!mac->data_out) || (!mac->data_out_left)) {
        usb_send_nak_i(mac);
        return 1; /* This packet was dropped, so ignore it. */
      }

      usb_send_data_i(mac);
      return 0; /* Return 0, since we just sent data and the host will ACK */

    case USB_DIP_DATA0:
    case USB_DIP_DATA1:
    case USB_DIP_DATA2:
    case USB_DIP_MDATA:
      /* Always ack data packets (ignoring crc, but oh well) */
      usb_send_ack_i(mac);

      if (usb_copy_data(mac, temp_packet + 1, packet_size - 3)) {
        /* Indicate there's an overflow error */
        mac->stats.overflow++;
        return -1;
      }
      return 1;   /* Return 1, since we have received data and we've ACKed */

    case USB_DIP_SOF:
    case USB_DIP_ACK:
    case USB_DIP_NAK:
    case USB_DIP_NYET:
    case USB_DIP_STALL:
    case USB_DIP_SPLIT:
    case USB_DIP_PING:
//      mac->stats.num_packets++;
      return 1;     /* Return 1, as there is not more data coming */

    case USB_DIP_OUT:
    case USB_DIP_SETUP:
      mac->is_rx = 1; /* We will be receiving data */
//      mac->stats.num_packets++;
      return 0;   /* Return 0, as DATA will soon follow */

    default:
      mac->stats.illegal_pids++;
      return -4;  /* Unrecognized PID */
  }
}

int usbMacProcessNext(struct USBMAC *mac) {

#if 0
  struct usb_packet *packet;
  int next_packet_idx = (mac->read_head + 1) & USB_PACKET_BUFFER_MASK;
  if (mac->write_head == mac->read_head) {
    mac->stats.underflow++;
    return -1;
  }

  /* Snag the current packet into our usb_packet struct */
  packet = &mac->packets[mac->read_head];

  /* Figure out which PID we're dealing with */
  switch (packet->pid) {

  /* Data packets, with CRC-16 */
  case USB_PID_DATA0: /* DATA0 */
  case USB_PID_DATA1: /* DATA1 */
  case USB_PID_DATA2: /* DATA2 */
  case USB_PID_MDATA: /* MDATA */
    if (validate_data(packet->data, packet->size)) {
      mac->stats.crc16_errors++;
      goto out;
    }
    if (mac->is_rx) {
      usb_send_ack_i(mac);
#warning "Handle receiving data here"
    }
    else {
#warning "Handle transmitting data here"
//      usb_send_nak_i(mac);
    }
    break;

  /* Token packets, with CRC-5 */
  case USB_PID_OUT: /* OUT */
  case USB_PID_IN: /* IN */
    if (validate_token(packet->data, packet->size)) {
      mac->stats.crc5_errors++;
      goto out;
    }
#warning "Figure out address and endpoint here"
    goto out;

  case USB_PID_SOF: /* SOF */
  case USB_PID_SETUP: /* SETUP */
    if (validate_token(packet->data, packet->size)) {
      mac->stats.crc5_errors++;
      goto out;
    }
#warning "Figure out what SOF or SETUP data they want"
    goto out;

  /* One-byte packets, no CRC required */
  case USB_PID_ACK: /* ACK */
  case USB_PID_NAK: /* NAK */
  case USB_PID_NYET: /* NYET */
  case USB_PID_STALL: /* STALL */
    /* Packet size is 8-bit PID only */
    if (packet->size != 0) {
      mac->stats.errors++;
      goto out;
    }
    goto out;

  /* Special cases (also no CRC?) */
  case USB_PID_SPLIT: /* SPLIT */
  case USB_PID_PING: /* PING */
  case USB_PID_ERR: /* ERR */
    if (packet->size != 1) {
      mac->stats.errors++;
      goto out;
    }
    goto out;

  default:
    mac->stats.illegal_pids++;
    goto out;
  }

out:

  /* Move on to the next packet */
#warning Lock here
  mac->read_head = next_packet_idx;
#warning Unlock here
#endif
  return 0;
}

const struct usb_packet *usbMacGetPacket(struct USBMAC *mac) {

  return NULL;
}

int usbMacSendPacket(struct USBMAC *mac, const struct usb_packet *packet) {
  uint8_t buffer[1 + 8 + 2]; /* PID + data + CRC-16 (worst-case) */
  int buffer_size;
  uint16_t crc16_calc;

  memset(buffer, 0, sizeof(buffer));
  buffer_size = -1;

  switch (packet->pid) {
  case USB_PID_MDATA:
  case USB_PID_DATA2:
  case USB_PID_DATA1:
  case USB_PID_DATA0:
    buffer[0] = packet->pid;
    memcpy(buffer + 1, packet->data, packet->size);
    crc16_calc = crc16(buffer + 1, packet->size, 0xffff, 0x8005);
    buffer[9] = crc16_calc >> 8;
    buffer[10] = crc16_calc;
    buffer_size = 11;
    break;

  /* Token packets, with CRC-5 */
  case USB_PID_SETUP:
  case USB_PID_SOF:
  case USB_PID_IN:
  case USB_PID_OUT:
#warning "Implement CRC-5"
    return -3;
    break;

  /* One-byte packets, no CRC required */
  case USB_PID_STALL:
  case USB_PID_NYET:
  case USB_PID_NAK:
  case USB_PID_ACK:
    buffer[0] = packet->pid;
    buffer_size = 1;
    break;

  /* Special cases (also no CRC?) */
  case USB_PID_ERR:
  case USB_PID_PING:
  case USB_PID_SPLIT:
    buffer[0] = packet->pid;
    buffer_size = 1;
    break;

  default:
    return -1;
  }

  if (buffer_size < 0)
    return -2;

  return usbPhyQueue(mac->phy, buffer, buffer_size);
}

int usbMacProcess(struct USBMAC *mac) {

#if 0
  usb_send_ack_i(mac);
  while (mac->write_head != mac->read_head)
    usbMacProcessNext(mac);
#endif
  return 0;
}

void usbMacInit(struct USBMAC *mac, const char *usb_descriptor) {

  usbMacResetStatistics(mac);
  mac->descriptor = usb_descriptor;
  mac->data_out = NULL;
  mac->data_out_left = 0;
}

void usbMacSetPhy(struct USBMAC *mac, struct USBPHY *phy) {

  mac->phy = phy;
}

struct USBPHY *usbMacPhy(struct USBMAC *mac) {

  return mac->phy;
}

struct USBMAC *usbMacDefault(void) {

  return &default_mac;
}
