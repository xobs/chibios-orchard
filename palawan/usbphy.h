#ifndef __USB_PHY_H__
#define __USB_PHY_H__

extern event_source_t usb_phy_data_available;
extern void usbStateTransitionI(void);
struct USBPHY;

struct usb_phy_statistics {
  int num_packets;
  int errors;
  int underflow;
  int read_head;
  int write_head;
  int buffer_size;
};

void usbPhyGetStatistics(struct usb_phy_statistics *stats);
int usbPhyResetStatistics(void);

void usbInit(void);
int usbPhyRead(struct USBPHY *phy, uint8_t *samples, uint32_t count);
int usbProcessIncoming(void);

#endif /* __USB_PHY_H__ */
