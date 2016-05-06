#ifndef __USB_PHY_H__
#define __USB_PHY_H__

struct USBMAC;

struct USBPHYInternalData {
  uint32_t padding[2];
  union {
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
  uint32_t stack_pointer_temp;  /* Temporary storage for sp during call */
};

/* Make sure this struct is not in flash.  Reads from flash are non-
 * -deterministic, and can have a variable amount of delay.  Also, make
 * sure it's not declared "const", as the "spSave" field is written by
 * the reader code.
 */
struct USBPHY {

  struct USBMAC *mac;     /* Pointer to the associated MAC */
  int initialized;

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

  uint32_t spSave;  /* The stack pointer is stored here during reading */
  uint32_t bufSave; /* The output buffer is stored here during reading */

  const void *queued_data;
  uint32_t queued_size;

#if (CH_USE_RT == TRUE)
  thread_reference_t thread;
  THD_WORKING_AREA(waThread, 128);
  event_source_t data_available;
#endif

  uint32_t byte_queue[256][3];
  uint8_t byte_queue_head;
  uint8_t byte_queue_tail;
} __attribute__((packed));

int usbPhyResetStatistics(struct USBPHY *phy);

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac);
int usbPhyReadI(const struct USBPHY *phy, uint32_t samples[3]);
void usbPhyWriteI(const struct USBPHY *phy, const void *buffer, uint32_t count);
void usbPhyWriteTestPattern(const struct USBPHY *phy);
void usbPhyWriteTest(struct USBPHY *phy);
int usbProcessIncoming(struct USBPHY *phy);
int usbPhyQueue(struct USBPHY *phy, const uint8_t *buffer, int buffer_size);
int usbCapture(struct USBPHY *phy);
int usbPhyInitialized(struct USBPHY *phy);
int usbPhyWritePrepare(struct USBPHY *phy, const uint32_t buffer[3], int size);

void usbPhyAttach(struct USBPHY *phy);
void usbPhyDetach(struct USBPHY *phy);

struct USBPHY *usbPhyDefaultPhy(void);
struct USBPHY *usbPhyTestPhy(void);

#endif /* __USB_PHY_H__ */
