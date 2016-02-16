#ifndef __USB_MAC_H__
#define __USB_MAC_H__

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

void usbMacGetStatistics(struct usb_mac_statistics *stats);
int usbMacResetStatistics(void);
int usbMacInsert(uint8_t *temp_packet, int packet_size);
const struct usb_packet *usbMacGetPacket(void);

#endif /* __USB_MAC_H__ */
