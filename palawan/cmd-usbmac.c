#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "palawan-shell.h"
#include "usbphy.h"
#include "usbmac.h"

enum usb_setup_request {
  usb_setup_get_status = 0,
  usb_setup_clear_feature = 1,
  usb_setup_set_feature = 3,
  usb_setup_set_address = 5,
  usb_setup_get_descriptor = 6,
  usb_setup_set_descriptor = 7,
  usb_setup_get_configuration = 8,
  usb_setup_set_configuration = 9,
  usb_setup_get_interface = 10,
  usb_setup_set_interface = 11,
  usb_setup_synch_frame = 12,
};

static const char *event_names[] = {
  "(none)",
  "WR_EVENT_START",
  "WR_EVENT_END",
  "RD_EVENT_START",
  "RD_EVENT_END",
  "INSERT_EVENT_START",
  "INSERT_EVENT_END",
  "IRQ_EVENT_START",
  "IRQ_EVENT_END",
  "WRPREP_EVENT_START",
  "WRPREP_EVENT_END",
};


extern uint32_t stats_timestamps[256];
extern uint32_t *stats_timestamps_ptr;

void cmd_usbmac_print(BaseSequentialStream *chp, struct USBMAC *mac)
{
  const struct usb_mac_statistics *mac_stats;
  const struct usb_phy_statistics *phy_stats;
  const struct usb_packet *packet;
  struct USBPHY *phy = mac->phy;
  int count;
  unsigned int i;

  phy_stats = usbPhyGetStatistics(phy);
  mac_stats = usbMacGetStatistics(mac);

  count = 0;
  chprintf(chp, "USB MAC Events:\r\n");
  while ((packet = usbMacGetPacket(mac)) != NULL) {
    count++;
    chprintf(chp, "Packet %d:\r\n", count);
    chprintf(chp, "    %02x %s\r\n", packet->pid, usbPidToStr(packet->pid));
    chprintf(chp, "    %d bytes of data\r\n", packet->size);
    if (packet->size) {
      chprintf(chp, "    -->");
      for (i = 0; i < packet->size; i++)
        chprintf(chp, " %02x", packet->data[i]);
      chprintf(chp, "\r\n");
    }
  }

  chprintf(chp, "USB processed %d MAC packets\r\n", count);

  chprintf(chp, "USB PHY Events:\r\n");
  uint32_t *timestamp = stats_timestamps;
  count = 0;
  while (timestamp != stats_timestamps_ptr) {
    uint32_t event = *timestamp;

    chprintf(chp, "      %3d: %6d %02x  %s\r\n",
        count,
        event & 0xffff,
        (event >> 24) & 0xff,
        event_names[(event >> 16) & 0xff]);
    timestamp++;
    count++;
  }
  stats_timestamps_ptr = stats_timestamps;

  chprintf(chp, "USB MAC statistics:\r\n");
  chprintf(chp, "    MAC packets:          %d\r\n", mac_stats->num_packets);
  chprintf(chp, "    MAC general errors:   %d\r\n", mac_stats->errors);
  chprintf(chp, "    MAC empty packets:    %d\r\n", mac_stats->empty_packets);
  chprintf(chp, "    MAC big packets:      %d\r\n", mac_stats->big_packets);
  chprintf(chp, "    MAC underflow events: %d\r\n", mac_stats->underflow);
  chprintf(chp, "    MAC overflow events:  %d\r\n", mac_stats->overflow);
  chprintf(chp, "    MAC CRC5 errors:      %d\r\n", mac_stats->crc5_errors);
  chprintf(chp, "    MAC CRC16 errors:     %d\r\n", mac_stats->crc16_errors);
  chprintf(chp, "    MAC illegal PIDs:     %d\r\n", mac_stats->illegal_pids);
  chprintf(chp, "    Incoming read head:   %d\r\n", mac_stats->read_head);
  chprintf(chp, "    Incoming write head:  %d\r\n", mac_stats->write_head);
  chprintf(chp, "    Incoming buffer size: %d\r\n", mac_stats->buffer_size);

  chprintf(chp, "USB PHY statistics:\r\n");
  chprintf(chp, "    PHY packets:          %d\r\n", phy_stats->num_packets);
  chprintf(chp, "    PHY errors:           %d\r\n", phy_stats->errors);
  chprintf(chp, "    PHY overflow:         %d\r\n", phy_stats->overflow);
  chprintf(chp, "    PHY timeout:          %d\r\n", phy_stats->timeout);
  chprintf(chp, "    PHY no end-of-sync:   %d\r\n", phy_stats->no_end_of_sync);
  chprintf(chp, "    PHY no end-of-frame:  %d\r\n", phy_stats->no_end_of_frame);
  chprintf(chp, "    Outgoing read head:   %d\r\n", phy_stats->out_read_head);
  chprintf(chp, "    Outgoing write head:  %d\r\n", phy_stats->out_write_head);
  chprintf(chp, "    Outgoing buffer size: %d\r\n", phy_stats->out_buffer_size);

  usbPhyResetStatistics(phy);
  usbMacResetStatistics(mac);
}

static void cmd_usbmac(BaseSequentialStream *chp, int argc, char *argv[])
{

  (void)argc;
  (void)argv;

  cmd_usbmac_print(chp, usbMacDefault());
}

palawan_command("mac", cmd_usbmac);
