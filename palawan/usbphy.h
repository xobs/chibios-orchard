#ifndef __USB_PHY_H__
#define __USB_PHY_H__

extern event_source_t usb_phy_data_available;
extern void usbStateTransitionI(void);
struct USBPHY;

struct usb_phy_statistics {
  int num_packets;
  int errors;
  int overflow;
  int timeout;
  int no_end_of_sync;
  int no_end_of_frame;
  int out_read_head;
  int out_write_head;
  int out_buffer_size;
  uint32_t *timestamps;
  uint32_t timestamp_count;
};

void usbPhyGetStatistics(struct usb_phy_statistics *stats);
int usbPhyResetStatistics(void);

void usbInit(void);
int usbPhyRead(const struct USBPHY *phy, uint8_t samples[11], uint32_t scratch[3]);
int usbPhyWrite(const struct USBPHY *phy, const uint8_t samples[11], uint32_t count);
void usbPhyWriteTestPattern(const struct USBPHY *phy);
void usbPhyWriteTest(void);
int usbProcessIncoming(void);
int usbPhyQueue(const uint8_t *buffer, int buffer_size);
int usbCapture(uint8_t samples[11]);

#endif /* __USB_PHY_H__ */
