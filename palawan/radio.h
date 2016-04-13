#ifndef __ORCHARD_RADIO_H__
#define __ORCHARD_RADIO_H__

struct _KRadioDevice;
typedef struct _KRadioDevice KRadioDevice;

#define RADIO_NETWORK_MAX_LENGTH 8
#define RADIO_BROADCAST_ADDRESS 255

typedef struct _RadioPacket {
  uint8_t length;       /* Total length of packet, including length field */
  uint8_t dst;          /* Address of intended recipient */
  uint8_t src;          /* Address of device sending packet */
  uint8_t port;         /* Subsystem packet is intended for */
  uint8_t payload[0];   /* Actual contents of packet */
} RadioPacket;

enum radio_protocols {
  radio_prot_paging      = 1,
  radio_prot_dut_to_peer = 6,
  radio_prot_peer_to_dut = 7,
};

extern KRadioDevice KRADIO1;

/* Call exactly once at startup to initialize the radio object. */
void radioInit(KRadioDevice *radio, SPIDriver *spip);

/* Call after startup to start the radio. */
void radioStart(KRadioDevice *radio);

/* Call to stop the radio. */
void radioStop(KRadioDevice *radio);

/* Raw read of a radio register */
uint8_t radioReadReg(KRadioDevice *radio, uint8_t addr);

/* Raw write to a radio register */
void radioWriteReg(KRadioDevice *radio, uint8_t addr, uint8_t val);

/* Raw read from multiple radio registers */
int radioReadRegs(KRadioDevice *radio, uint8_t addr, void *bfr, int count);

/* Get the temperature (in C) of the radio */
int radioTemperature(KRadioDevice *radio);

/* Set our network ID.  Can be 0-8 bytes. */
void radioSetNetwork(KRadioDevice *radio, const uint8_t *id, uint8_t len);

/* Send a packet out the network */
void radioSend(KRadioDevice *radio, uint8_t dest, uint8_t prot,
                                    size_t len, const void *payload);

/* Set our 8-bit address */
void radioSetAddress(KRadioDevice *radio, uint8_t addr);

/* Return our 8-bit address */
uint8_t radioAddress(KRadioDevice *radio);

/* System callback for the radio packet_ready signal */
void radioInterrupt(EXTDriver *extp, expchannel_t channel);

#endif /* __ORCHARD_RADIO_H__ */
