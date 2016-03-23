#ifndef __USB_PHY_H__
#define __USB_PHY_H__

struct USBMAC;

struct usb_phy_statistics {
  int num_packets;
  int errors;
  int empty;
  int overflow;
  int timeout;
  int no_end_of_sync;
  int no_end_of_frame;
  int out_read_head;
  int out_write_head;
  int out_buffer_size;
//  uint32_t timestamps[512];
//  uint32_t timestamp_count;
//  uint32_t timestamp_mask;
};

#define BYTE_BUFFER_SIZE 0 + \
                          + 1             /* PID */                           \
                          + 8             /* Data */                          \
                          + 2             /* CRC16 */                         \
                          + 0

#ifndef MAX_SEND_QUEUES
#define MAX_SEND_QUEUES 16
#define MAX_SEND_QUEUES_MASK 0xf
#endif

union USBPHYInternalData {
  uint32_t scratch[7];    /* Scratch buffer, as used by usb reader/writer */
  struct {
    uint8_t data1[4];     /* Actual data of the first word */
    uint32_t bits1;       /* Number of bits in the first word */
    uint8_t data2[4];     /* Actual data of the second word */
    uint32_t bits2;       /* Number of bits in the third word */
    uint8_t data3[4];     /* Actual data of the third word */
    uint32_t bits3;       /* Number of bits in the third word */
    uint32_t first_word;  /* Which word to start with (1, 2, or 3) */
  };
};

struct USBPHY {

  struct USBMAC *mac;     /* Pointer to the associated MAC */
  int initialized;

  /* This union must be located 8 bytes from the start of the struct */
  union USBPHYInternalData internalData;

  /* USB D+ pin specification */
  volatile uint32_t *usbdpIAddr;
  volatile uint32_t *usbdpSAddr;
  volatile uint32_t *usbdpCAddr;
  volatile uint32_t *usbdpDAddr;
  uint32_t usbdpShift;

  /* USB D- pin specification */
  volatile uint32_t *usbdnIAddr;
  volatile uint32_t *usbdnSAddr;
  volatile uint32_t *usbdnCAddr;
  volatile uint32_t *usbdnDAddr;
  uint32_t usbdnShift;

  uint32_t usbdpMask;
  uint32_t usbdnMask;

  struct usb_phy_statistics stats;
  uint32_t write_head;
  uint32_t read_head;
  uint32_t queues[MAX_SEND_QUEUES][BYTE_BUFFER_SIZE];
  uint32_t queue_sizes[MAX_SEND_QUEUES];

  thread_reference_t thread;
  THD_WORKING_AREA(waThread, 128);

  event_source_t data_available;
} __attribute__((packed));

const struct usb_phy_statistics *usbPhyGetStatistics(struct USBPHY *phy);
int usbPhyResetStatistics(struct USBPHY *phy);

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac);
int usbPhyReadI(const struct USBPHY *phy, uint8_t samples[12]);
int usbPhyWriteI(const struct USBPHY *phy);
void usbPhyWriteTestPattern(const struct USBPHY *phy);
void usbPhyWriteTest(struct USBPHY *phy);
int usbProcessIncoming(struct USBPHY *phy);
int usbPhyQueue(struct USBPHY *phy, const uint8_t *buffer, int buffer_size);
int usbCapture(struct USBPHY *phy);
int usbPhyInitialized(struct USBPHY *phy);
int usbPhyWriteDirectI(struct USBPHY *phy, const uint8_t buffer[12], int size);

void usbPhyAttach(struct USBPHY *phy);
void usbPhyDetach(struct USBPHY *phy);

struct USBPHY *usbPhyDefaultPhy(void);
struct USBPHY *usbPhyTestPhy(void);

#endif /* __USB_PHY_H__ */
