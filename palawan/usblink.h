#ifndef __USB_LINK_H__
#define __USB_LINK_H__

struct USBMAC;

/* Various Descriptor Types */
#define DT_DEVICE 0x01
#define DT_CONFIGURATION 0x02
#define DT_STRING 0x03
#define DT_INTERFACE 0x04
#define DT_ENDPOINT 0x05
#define DT_DEVICE_QUALIFIER 0x06
#define DT_OTHER_SPEED_CONFIGURATION 0x07
#define DT_INTERFACE_POWER 0x08

#define DT_HID 0x21
#define DT_HID_REPORT 0x22
#define DT_PID 0x23

struct usb_device_descriptor {
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint16_t bcdUSB;
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
} __attribute__((packed));

struct usb_configuration_descriptor {
  uint8_t  bLength;             /* Size of this descriptor, in bytes (9) */
  uint8_t  bDescriptorType;     /* DT_CONFIGURATION (2) */
  uint16_t wTotalLength;        /* Total length of this, plus sizeof(data) */
  uint8_t  bNumInterfaces;      /* Number of interfaces supported by config */
  uint8_t  bConfigurationValue; /* Value used by Set Configuration */
  uint8_t  iConfiguration;      /* index of string descriptor for config */
  uint8_t  bmAttributes;        /* Bitmap of attributes.  D7 must be 1. */
  uint8_t  bMaxPower;           /* Maximum power, in units of 2mA */
  uint8_t  data[];              /* Remaining descriptors */
} __attribute__((packed));

struct usb_string_descriptor {
  uint8_t bLength;          /* sizeof(usb_string_descriptor) + sizeof(data) */
  uint8_t bDescriptorType;  /* DT_STRING (3) */
  uint8_t data[];           /* UTF-16LE string data or lang data(for string 0 */
} __attribute__((packed));

struct usb_interface_descriptor {
  uint8_t bLength;            /* sizeof(usb_interface_descriptor) (9) */
  uint8_t bDescriptorType;    /* DT_INTERFACE (4) */
  uint8_t bInterfaceNumber;   /* Which interface this describes.  Usually 0. */
  uint8_t bAlternateSetting;  /* ??? */
  uint8_t bNumEndpoints;      /* Number of endpoints, minus 1 */
  uint8_t bInterfaceClass;    /* Class code */
  uint8_t bInterfaceSubclass; /* Class sub-code */
  uint8_t bInterfaceProtocol; /* Protocol code, assigned by USB */
  uint8_t iInterface;         /* Index of string for this interface */
} __attribute__((packed));

struct usb_endpoint_descriptor {
  uint8_t  bLength;           /* sizeof(usb_endpoint_descriptor) (7) */
  uint8_t  bDescriptorType;   /* DT_ENDPOINT (5) */
  uint8_t  bEndpointAddress;  /* High bit 1:IN, 0:OUT. Lower 4-bits are EP# */
  uint8_t  bmAttributes;      /* 0=control, 2=bulk, 3=interrupt */
  uint16_t wMaxPacketSize;    /* Max packet size for this EP */
  uint8_t  bInterval;         /* Polling rate (in 1ms units) */
} __attribute__((packed));

struct usb_hid_descriptor {
  uint8_t  bLength;           /* sizeof(usb_hid_descriptor) (9) */
  uint8_t  bDescriptorType;   /* DT_HID (0x21) */
  uint16_t bcdHID;            /* HID class version number, in BCD */
  uint8_t  bCountryCode;      /* Target country (usually 0) */
  uint8_t  bNumDescriptors;   /* Number of HID class descriptors (usually 1) */
  uint8_t  bReportDescriptorType;   /* Report descriptor type (usually 0x22) */
  uint16_t wReportDescriptorLength; /* Length of the HID/PID report descriptor */
} __attribute__((packed));

struct USBLink {
  struct USBMAC *mac;
  const struct usb_link_device_descriptor *device_descriptor;
  const struct usb_link_configuration_descriptor *configuration_descriptor;
  int phy_internal_is_ready;

  union {
    uint8_t data_in[8];
    struct usb_mac_setup_packet data_setup;
  };

  const void *data_out;
  int32_t data_out_left;

  /* Data that's ready to be sent */
  struct USBPHYInternalData phy_internal;

  char data_in_size;

  uint8_t data_buffer;  /* Whether we're sending DATA0 or DATA1 */
  uint8_t packet_type;  /* PACKET_SETUP, PACKET_IN, or PACKET_OUT */

  uint8_t address;      /* Our configured address */
  uint8_t config_num;   /* Currently-selected configuration */

  uint8_t tok_addr;     /* Last token's address */
  uint8_t tok_epnum;    /* Last token's endpoint */
};

#endif /* __USB_LINK_H__ */