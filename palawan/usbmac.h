#ifndef __USB_MAC_H__
#define __USB_MAC_H__

enum usb_pids {
  USB_PID_RESERVED = 0xf0,
  USB_PID_SPLIT = 0x78,
  USB_PID_PING = 0xb4,
  USB_PID_ERR = 0x3c,
  USB_PID_ACK = 0xd2,
  USB_PID_NAK = 0x5a,
  USB_PID_NYET = 0x96,
  USB_PID_STALL = 0x1e,
  USB_PID_OUT = 0xe1,
  USB_PID_IN = 0x69,
  USB_PID_SOF = 0xa5,
  USB_PID_SETUP = 0x2d,
  USB_PID_DATA0 = 0xc3,
  USB_PID_DATA1 = 0x4b,
  USB_PID_DATA2 = 0x87,
  USB_PID_MDATA = 0x0f,
};

struct usb_packet {
  uint8_t pid;
  uint8_t data[8];
  uint8_t size; /* Not including pid (so may be 0) */
  /* Checksum omitted */
};

struct usb_mac_statistics {
  int num_packets;
  int errors;
  int underflow;
  int crc5_errors;
  int crc16_errors;
  int illegal_pids;
  int read_head;
  int write_head;
  int buffer_size;
};

const char *usbPidToStr(uint8_t pid);
void usbMacGetStatistics(struct usb_mac_statistics *stats);
int usbMacResetStatistics(void);
int usbMacInsert(uint8_t *temp_packet, int packet_size);
const struct usb_packet *usbMacGetPacket(void);
int usbMacSendPacket(const struct usb_packet *packet);

#endif /* __USB_MAC_H__ */
