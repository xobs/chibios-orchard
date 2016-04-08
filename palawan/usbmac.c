#include <string.h>
#include "hal.h"
#include "usbphy.h"
#include "usbmac.h"
#include "usblink.h"
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
  
  for (i = 0; i < 8; i++) {
    if ((crc ^ c) & 1) 
      crc = (crc >> 1) ^ poly;
    else 
      crc >>= 1;
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

  uint32_t raw_data_32[3];
  struct usb_packet *packet = (struct usb_packet *)raw_data_32;
  uint16_t crc;

  /* Don't allow us to re-prepare data */
  if (mac->phy->data_is_queued)
    return;

  /* If there's no data to send, then don't send any */
  if (mac->data_out_left < 0)
    return;

  if (mac->data_buffer++ & 1)
    packet->pid = USB_PID_DATA1;
  else
    packet->pid = USB_PID_DATA0;

  /* If there's no data, prepare a special NULL packet */
  if (mac->data_out_left == 0) {
    mac->data_out_left = -1;
    packet->data[0] = 0;
    packet->data[1] = 0;
    usbPhyWritePrepare(mac->phy, raw_data_32, 2 + 1);
    return;
  }

  /* Keep the packet size to 8 bytes max */
  packet->size = mac->data_out_left;
  if (mac->data_out_left > 8)
    packet->size = 8;

  /* Copy over data bytes */
  memcpy(packet->data, mac->data_out, packet->size);

  /* Reduce the amount of data left.
   * If the packet is divisible by 8, this will cause one more call
   * to this function with mac->data_out_left == 0.  This will send
   * a NULL packet, which indicates end-of-transfer.
   */
  mac->data_out_left -= 8;
  mac->data_out += 8;

  /* Calculate and copy the crc16 */
  crc = ~crc16(packet->data, packet->size, 0xffff, 0xa001);
  packet->data[packet->size++] = crc;
  packet->data[packet->size++] = crc >> 8;

  /* Prepare the packet, including the PID at the end */
  usbPhyWritePrepare(mac->phy, raw_data_32, packet->size + 1);
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

static int usb_mac_send_data(struct USBMAC *mac, const void *data, int count) {

  /* Don't allow for dereferencing NULL pointers.  Probably an uninteresting
   * check, because there's no MMU already.
   */
  if (!data && count)
    return -1;

  mac->data_out = data;
  mac->data_out_left = count;

  return 0;
}

static int usb_mac_process_setup_read(struct USBMAC *mac,
                                      const struct usb_mac_setup_packet *setup)
{
  const void *response = NULL;
  uint32_t len = 0;

  switch (setup->bmRequestType) {
  case 0x80:  /* Device-to-host, standard, read from device */
    switch (setup->bRequest) {

    /* GET_DESCRIPTOR */
    case 6:

      switch (setup->wValueL) {

      /* GET_DEVICE_DESCRIPTOR */
      case 1:
        len = mac->device_descriptor->bLength;
        response = mac->device_descriptor;
        break;

      /* GET_CONFIGURATION_DESCRIPTOR */
      case 2:
        len = mac->configuration_descriptor->wTotalLength;
        response = mac->configuration_descriptor;
        break;

      /* GET_STRING_DESCRIPTOR */
      case 3:
        if (setup->wValueH == 0) {
          static const uint8_t en_us[] = {0x04, DT_STRING, 0x09, 0x04};
          len = sizeof(en_us);
          response = en_us;
        }
        else {
          static const uint8_t str[] = {
              0x06, DT_STRING, 0x65, 0x00, 0x66, 0x00,
          };
          len = sizeof(str);
          response = str;
        }
        break;

      }
      break;
    }
    break;
  case 0x81: /* Device-to-host, standard, read from interface */
    switch(setup->bRequest) {

    /* GET_CLASS_DESCRIPTOR */
    case 6: {
      static const uint8_t report_descriptor[] = {
        0x05, 0x01, /* USAGE_PAGE (Generic Desktop)           */
        0x09, 0x06, /* USAGE (Keyboard)                       */
        0xa1, 0x01, /* COLLECTION (Application)               */
        0x05, 0x07, /*   USAGE_PAGE (Keyboard)                */
        0x19, 0xe0, /*   USAGE_MINIMUM (Keyboard LeftControl) */
        0x29, 0xe7, /*   USAGE_MAXIMUM (Keyboard Right GUI)   */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0)                  */
        0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                  */
        0x75, 0x01, /*   REPORT_SIZE (1)                      */
        0x95, 0x08, /*   REPORT_COUNT (8)                     */
        0x81, 0x02, /*   INPUT (Data,Var,Abs)                 */
        0x95, 0x01, /*   REPORT_COUNT (1)                     */
        0x75, 0x08, /*   REPORT_SIZE (8)                      */
        0x81, 0x03, /*   INPUT (Cnst,Var,Abs)                 */
        0x95, 0x05, /*   REPORT_COUNT (5)                     */
        0x75, 0x01, /*   REPORT_SIZE (1)                      */
        0x05, 0x08, /*   USAGE_PAGE (LEDs)                    */
        0x19, 0x01, /*   USAGE_MINIMUM (Num Lock)             */
        0x29, 0x05, /*   USAGE_MAXIMUM (Kana)                 */
        0x91, 0x02, /*   OUTPUT (Data,Var,Abs)                */
        0x95, 0x01, /*   REPORT_COUNT (1)                     */
        0x75, 0x03, /*   REPORT_SIZE (3)                      */
        0x91, 0x03, /*   OUTPUT (Cnst,Var,Abs)                */
        0x95, 0x06, /*   REPORT_COUNT (6)                     */
        0x75, 0x08, /*   REPORT_SIZE (8)                      */
        0x15, 0x00, /*   LOGICAL_MINIMUM (0)                  */
        0x25, 0x65, /*   LOGICAL_MAXIMUM (101)                */
        0x05, 0x07, /*   USAGE_PAGE (Keyboard)                */
        0x19, 0x00, /*   USAGE_MINIMUM (Reserved)             */
        0x29, 0x65, /*   USAGE_MAXIMUM (Keyboard Application) */
        0x81, 0x00, /*   INPUT (Data,Ary,Abs)                 */
        0xc0    /* END_COLLECTION                         */
      };
      len = sizeof(report_descriptor);
      response = report_descriptor;
      break;
      }
    }
    break;

  case 0x82: /* Device-to-host, standard, read from endpoint */
    break;

  case 0x83: /* Device-to-host, standard, read from other */
    break;

  case 0xa0: /* Device-to-host, class, read from device */
    break;

  case 0xa1: /* Device-to-host, class, read from interface */
    break;

  case 0xa2: /* Device-to-host, class, read from endpoint */
    break;

  case 0xa3: /* Device-to-host, class, read from other */
    break;

  case 0xc0: /* Device-to-host, vendor, read from device */
    break;

  case 0xc1: /* Device-to-host, vendor, read from interface */
    break;

  case 0xc2: /* Device-to-host, vendor, read from endpoint */
    break;

  case 0xc3: /* Device-to-host, vendor, read from other */
    break;
  }

  if (len > setup->wLength)
    len = setup->wLength;

  usb_mac_send_data(mac, response, len);
  return 0;
}

static int usb_mac_process_setup_write(struct USBMAC *mac,
                                       const struct usb_mac_setup_packet *setup)
{
  const void *response = NULL;
  uint8_t len = 0;

  switch (setup->bmRequestType) {
  case 0x00:  /* Device-to-host, standard, write to host */
    switch (setup->bRequest) {
    case 5: /* SET_ADDRESS */
      mac->address = setup->wValue;
      break;
    case 9: /* SET_CONFIGURATION */
      mac->config_num = setup->wValue;
      break;
    }
    break;

  case 0x01: /* Device-to-host, standard, write to host */
    break;

  case 0x02: /* Device-to-host, standard, write to host */
    break;

  case 0x03: /* Device-to-host, standard, write to host */
    break;

  case 0x20: /* Device-to-host, class, write to host */
    break;

  case 0x21: /* Device-to-host, class, write to host */
    switch (setup->bRequest) {
    case 0x0a: /* Thingy? */
      /* This happens in Set Idle */
      break;
    }
    break;

  case 0x22: /* Device-to-host, class, write to host */
    break;

  case 0x23: /* Device-to-host, class, write to host */
    break;

  case 0x40: /* Device-to-host, vendor, write to host */
    break;

  case 0x41: /* Device-to-host, vendor, write to host */
    break;

  case 0x42: /* Device-to-host, vendor, write to host */
    break;

  case 0x43: /* Device-to-host, vendor, write to host */
    break;
  }

  /* We must always send a response packet.  If there's ever a time when
   * we shouldn't send a packet, simply "return" rather than "break" above.
   */
  usb_mac_send_data(mac, response, len);

  return 0;
}

static int usb_mac_process_setup(struct USBMAC *mac, const uint8_t packet[10]) {

  const struct usb_mac_setup_packet *setup;

  setup = (const struct usb_mac_setup_packet *)packet;

  if (setup->bmRequestType & 0x80)
    /* Device-to-Host */
    return usb_mac_process_setup_read(mac, setup);

  else
    return usb_mac_process_setup_write(mac, setup);

  return 0;
}

static inline void usb_mac_parse_token(struct USBMAC *mac,
                                       const uint8_t packet[2]) {

  mac->tok_addr = packet[0] >> 1;
  mac->tok_epnum = (packet[1] >> 5) | ((packet[0] << 5) & 1);
}

static void usb_mac_parse_data(struct USBMAC *mac,
                               const uint8_t packet[10],
                               uint32_t count) {
  (void)count;

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
    mac->data_buffer = 1;
    usb_mac_parse_data(mac, packet + 1, count - 1);
    mac->packet_type = packet_type_none;
    break;

  case USB_PID_DATA1:
    mac->data_buffer = 0;
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
                const struct usb_device_descriptor *device_descriptor,
                const struct usb_configuration_descriptor *config) {

  mac->device_descriptor = device_descriptor;
  mac->configuration_descriptor = config;
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
