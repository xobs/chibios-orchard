#ifndef __USB_PHY_H__
#define __USB_PHY_H__

extern void usbStateTransitionI(void);

void usbPhyRead(uint8_t *samples, uint32_t count);

#endif /* __USB_PHY_H__ */
