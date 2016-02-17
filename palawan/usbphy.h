#ifndef __USB_PHY_H__
#define __USB_PHY_H__

extern event_source_t usb_phy_data_available;
extern void usbStateTransitionI(void);
struct USBPHY;

struct usb_phy_statistics {
  int num_packets;
  int errors;
  int underflow;
  int overflow;
  int timeout;
  int in_read_head;
  int in_write_head;
  int in_buffer_size;
  int out_read_head;
  int out_write_head;
  int out_buffer_size;
};

void usbPhyGetStatistics(struct usb_phy_statistics *stats);
int usbPhyResetStatistics(void);

void usbInit(void);
int usbPhyRead(const struct USBPHY *phy, uint8_t *samples, uint32_t count);
int usbPhyWrite(const struct USBPHY *phy, const uint8_t *samples, uint32_t count);
int usbProcessIncoming(void);
int usbPhyQueue(const uint8_t *buffer, int buffer_size);

#endif /* __USB_PHY_H__ */
