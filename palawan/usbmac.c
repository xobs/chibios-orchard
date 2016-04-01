#include <string.h>
#include "ch.h"
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "palawan.h"

static struct USBMAC default_mac;

/* Pre-processed internal data structures for fast responses */
struct USBPHYInternalData phyAck;
struct USBPHYInternalData phyNak;

extern const uint8_t bit_reverse_table_256[];

#if 0
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
#endif

static uint16_t crc16_add(uint16_t crc, uint8_t c, uint16_t poly)
{
  uint8_t  i;
  
  for (i=0; i<8; i++)
  {
    if ((crc ^ c) & 1) 
    { 
      crc = (crc >> 1) ^ poly;
    } 
    else 
    {
      crc >>= 1;
    }
    c >>= 1;
  }
  return crc;
}

static uint16_t crc16(const uint8_t *data, uint32_t count,
                      uint16_t init, uint32_t poly) {

  while (count--)
    init = crc16_add(init, *data++, poly);

  return init;
}

static void usb_mac_process_data(struct USBMAC *mac) {

  struct usb_packet packet;
  uint16_t crc;

  if (mac->phy_internal_is_ready)
    return;
  if (!mac->data_out)
    return;
  if (!mac->data_out_left)
    return;

  if (mac->data_buffer++ & 1)
    packet.pid = USB_PID_DATA1;
  else
    packet.pid = USB_PID_DATA0;

  /* Keep the packet size to 8 bytes max */
  packet.size = mac->data_out_left;
  if (mac->data_out_left > 8)
    packet.size = 8;
  mac->data_out_left -= packet.size;

  /* Copy over data bytes */
  memcpy(packet.data, mac->data_out, packet.size);
  mac->data_out += packet.size;

  /* Calculate and copy the crc16 */
  crc = ~crc16(packet.data, packet.size, 0xffff, 0xa001);
  packet.data[packet.size++] = crc;
  packet.data[packet.size++] = crc >> 8;

  /* Prepare the packet, including the PID at the end */
  usbPhyWritePrepare(mac->phy, packet.raw_data_32, packet.size + 1);
  mac->phy_internal_is_ready = 1;
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

void usbSendAckI(struct USBMAC *mac) {
    usbPhyWriteI(mac->phy, &phyAck);
}

void usbSendNakI(struct USBMAC *mac) {
    usbPhyWriteI(mac->phy, &phyNak);
}

void usbSendDataI(struct USBMAC *mac) {

  mac->phy_internal_is_ready = 0;
  usbPhyWriteI(mac->phy, &mac->phy_internal);
}

#if 0
static inline void usb_mac_parse_token_rev(struct USBMAC *mac, uint8_t packet[2]) {

  mac->addr = packet[1] >> 1;
  mac->epnum = (packet[0] >> 5) | ((packet[1] << 5) & 1);
}

typedef int (*pidHandler)(struct USBMAC *mac, uint8_t *rev_pkt, int size);

static int dip_handler_in(struct USBMAC *mac, uint8_t *rev_pkt, int size) {

  /* If there is no data currently, NAK the request so they will retry */
  /*
  (void)size;
  if (!mac->phy_internal_is_ready) {
    usbSendNakI(mac);
    mac->packet_type = packet_type_in;
    usb_mac_parse_token_rev(mac, rev_pkt);
    return 0;
  }

  usbSendDataI(mac);
  */
  return 1;
}

static int dip_handler_out(struct USBMAC *mac, uint8_t *rev_pkt, int size) {

  (void)size;
  mac->packet_type = packet_type_out;
  usb_mac_parse_token_rev(mac, rev_pkt);
  return 1;
}

static int dip_handler_setup(struct USBMAC *mac, uint8_t *rev_pkt, int size) {

  (void)size;
  mac->packet_type = packet_type_setup;
  usb_mac_parse_token_rev(mac, rev_pkt);
  return 1;
}

static int dip_handler_data(struct USBMAC *mac, uint8_t *rev_pkt, int size) {

  int data_copied, data_left;
  uint8_t *data;

  /* Always ack data packets (ignoring crc, but oh well) */
  usbSendAckI(mac);

  data = mac->data_in;
  data_left = size - 2; /* Ignore the two bytes of crc */
  for (data_copied = 0; data_copied < size - 3; data_copied++)
    data[data_copied] = bit_reverse_table_256[rev_pkt[data_left--]];
  mac->data_in_size = data_copied;

  return 0;
}

static int dip_handler_ign(struct USBMAC *mac, uint8_t *rev_pkt, int size) {

  (void)rev_pkt;
  (void)mac;
  (void)size;

  return 0;
}

static pidHandler dip_handlers[16] = {
  [USB_DIP_IN & 0xf] = dip_handler_in,
  [USB_DIP_OUT & 0xf] = dip_handler_out,
  [USB_DIP_SETUP & 0xf] = dip_handler_setup,
  [USB_DIP_DATA0 & 0xf] = dip_handler_data,
  [USB_DIP_DATA1 & 0xf] = dip_handler_data,
  [USB_DIP_ACK & 0xf] = dip_handler_ign,
  [USB_DIP_NAK & 0xf] = dip_handler_ign,

  [USB_DIP_SPLIT & 0xf] = dip_handler_ign,
  [USB_DIP_PING & 0xf] = dip_handler_ign,
  [USB_DIP_PRE & 0xf] = dip_handler_ign,
  [USB_DIP_NYET & 0xf] = dip_handler_ign,
  [USB_DIP_STALL & 0xf] = dip_handler_ign,
  [USB_DIP_SOF & 0xf] = dip_handler_ign,
  [USB_DIP_DATA2 & 0xf] = dip_handler_ign,
  [USB_DIP_MDATA & 0xf] = dip_handler_ign,
  [0xf] = dip_handler_ign,
};


int usbMacInsertRevI(struct USBMAC *mac, uint32_t *rev_pkt, int size) {

  uint8_t dip = ((uint8_t *)rev_pkt)[size - 1]; /* reversed PID */

  if (!isValidPID(dip))
    return 0;

  /* Figure out if the packet we're processing is valid, and if we expect
   * more data to follow.
   */
  return dip_handlers[dip & 0xf](mac, (uint8_t *)rev_pkt, size);
}
#endif

static int usb_mac_send_data(struct USBMAC *mac, const void *data, int count) {

  mac->data_out = data;
  mac->data_out_left = count;

  return 0;
}

static int usb_mac_process_setup(struct USBMAC *mac, const uint8_t packet[10]) {

  const struct usb_mac_setup_packet *setup;
  setup = (const struct usb_mac_setup_packet *)packet;

  switch (setup->bRequest) {

    case 6: /* GET_DESCRIPTOR */
      usb_mac_send_data(mac, mac->descriptor, sizeof(*mac->descriptor));
      break;

    default:
      break;
  }

  return 0;
}

static inline void usb_mac_parse_token(struct USBMAC *mac,
                                       const uint8_t packet[2]) {

  mac->addr = packet[0] >> 1;
  mac->epnum = (packet[1] >> 5) | ((packet[0] << 5) & 1);
}

static void usb_mac_parse_data(struct USBMAC *mac,
                               const uint8_t packet[10],
                               uint32_t count) {
  switch (mac->packet_type) {

  case packet_type_setup:
    usb_mac_process_setup(mac, packet);
    break;

  case packet_type_in:

    break;

  case packet_type_out:

    break;

  case packet_type_none:
    break;

  default:
    break;
  }
}

int usbMacProcess(struct USBMAC *mac,
                  const uint8_t packet[11],
                  uint32_t count) {

  switch(packet[0]) {
  case USB_PID_SETUP:
    mac->packet_type = packet_type_setup;
    usb_mac_parse_token(mac, packet + 1);
    break;

  case USB_PID_DATA0:
  case USB_PID_DATA1:
    usb_mac_parse_data(mac, packet + 1, count - 1);
    mac->packet_type = packet_type_none;
    break;

  default:
    break;
  }

  /* Pre-process any data that's remaining, so it can be sent out
   * in case an IN token comes.
   */
  usb_mac_process_data(mac);

  return 0;
}

void usbMacInit(struct USBMAC *mac,
                const struct usb_mac_device_descriptor *usb_descriptor) {

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
