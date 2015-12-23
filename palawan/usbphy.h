#ifndef __USB_PHY_H__
#define __USB_PHY_H__

extern void usbStateTransitionI(void);
struct USBPHY;

int usbPhyRead(struct USBPHY *phy, uint8_t *samples, uint32_t count);

#endif /* __USB_PHY_H__ */
