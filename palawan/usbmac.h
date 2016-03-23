#ifndef __USB_MAC_H__
#define __USB_MAC_H__

enum usb_pids {
  USB_PID_RESERVED = 0xf0,
  USB_PID_OUT = 0xe1,
  USB_PID_ACK = 0xd2,
  USB_PID_DATA0 = 0xc3,
  USB_PID_PING = 0xb4,
  USB_PID_SOF = 0xa5,
  USB_PID_NYET = 0x96,
  USB_PID_DATA2 = 0x87,
  USB_PID_SPLIT = 0x78,
  USB_PID_IN = 0x69,
  USB_PID_NAK = 0x5a,
  USB_PID_DATA1 = 0x4b,
  USB_PID_ERR = 0x3c,
  USB_PID_SETUP = 0x2d,
  USB_PID_STALL = 0x1e,
  USB_PID_MDATA = 0x0f,
};

enum usb_pids_rev {
  USB_DIP_SPLIT = 0x1e,
  USB_DIP_PING = 0x2d,
  USB_DIP_PRE = 0x3c,
  USB_DIP_ACK = 0x4b,
  USB_DIP_NAK = 0x5a,
  USB_DIP_NYET = 0x69,
  USB_DIP_STALL = 0x78,
  USB_DIP_OUT = 0x87,
  USB_DIP_IN = 0x96,
  USB_DIP_SOF = 0xa5,
  USB_DIP_SETUP = 0xb4,
  USB_DIP_DATA0 = 0xc3,
  USB_DIP_DATA1 = 0xd2,
  USB_DIP_DATA2 = 0xe1,
  USB_DIP_MDATA = 0xf0,
};

struct usb_packet {
  union {
    struct {
      uint8_t pid;
      uint8_t data[10]; /* Including CRC */
    };
    uint8_t raw_data[11];
  };
  uint8_t size; /* Not including pid (so may be 0) */
  /* Checksum omitted */
} __attribute__((packed));

struct usb_mac_statistics {
  int num_packets;
  int errors;
  int underflow;
  int overflow;
  int crc5_errors;
  int crc16_errors;
  int illegal_pids;
  int empty_packets;
  int big_packets;
  int read_head;
  int write_head;
  int buffer_size;
};

enum usb_state {
  state_rx,
  state_tx,
};

struct USBMAC {
  const char *descriptor;
  struct USBPHY *phy;
  uint32_t read_head;
  uint32_t write_head;
  uint8_t data_buffer;  /* Whether we're sending DATA0 or DATA1 */
  uint8_t is_rx;        /* Whether receiving (SETUP or OUT) or sending (IN) */

  uint8_t *data_out;
  uint32_t data_out_left;
  char data_in[64];
  char data_in_size;

  struct usb_mac_statistics stats;
};

/* Return the statistics from this MAC */
const struct usb_mac_statistics *usbMacGetStatistics(struct USBMAC *mac);

/* Reset the statistics for this particular MAC */
int usbMacResetStatistics(struct USBMAC *mac);

/* Insert a new packet, received on the wire, into the USB MAC layer.
   Return value:
      <-4   A unknown error occurred
       -4   Unrecognized PID
       -3   Packet was too large
       -2   Packet was empty
       -1   Packet buffer overflowed and packet was dropped
        0   No error occurred, but there is a time-sensitive packet following
      > 0   No error occurred, and processing can continue
   If this function returns 0, then another packet will follow shortly, and
   the SoC must be able to quickly respond.  In these cases, it is inadvisable
   to switch threads.
 */
int usbMacInsertI(struct USBMAC *, uint8_t *incoming_packet, int packet_size);

/* Get the next available USB packet, or NULL if none available */
const struct usb_packet *usbMacGetPacket(struct USBMAC *mac);

/* Process all packets sitting in the queue */
int usbMacProcess(struct USBMAC *mac);

/* Send the specified USB packet */
int usbMacSendPacket(struct USBMAC *mac, const struct usb_packet *packet);

/* Initialize the given USB MAC */
void usbMacInit(struct USBMAC *mac, const char *usb_descriptor);

/* Set the given MAC to use the given PHY */
void usbMacSetPhy(struct USBMAC *mac, struct USBPHY *phy);

/* Get the default, system-wide USB MAC */
struct USBMAC *usbMacDefault(void);

/* Get the textual representation of the specified USB PID */
const char *usbPidToStr(uint8_t pid);

/* Get the PHY associated with this MAC */
struct USBPHY *usbMacPhy(struct USBMAC *mac);

#endif /* __USB_MAC_H__ */
