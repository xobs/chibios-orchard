#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "palawan-shell.h"
#include "usbphy.h"
#include "usbmac.h"

void cmd_usbmac(BaseSequentialStream *chp, int argc, char *argv[])
{
  struct usb_mac_statistics mac_stats;
  struct usb_phy_statistics phy_stats;
  const struct usb_packet *packet;
  int count;
  unsigned int i;

  (void)argc;
  (void)argv;

  usbPhyGetStatistics(&phy_stats);
  usbMacGetStatistics(&mac_stats);

  count = 0;
  chprintf(chp, "USB MAC Events:\r\n");
  while ((packet = usbMacGetPacket()) != NULL) {
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
  for (i = 0; i < phy_stats.timestamp_count; i++) {
    uint32_t event = phy_stats.timestamps[i];
    chprintf(chp, "      %3d: %c %d\r\n",
        i,
        event & 0x00010000 ? 'i' : 'o',
        event & 0xffff);
  }

  chprintf(chp, "USB MAC statistics:\r\n");
  chprintf(chp, "    MAC packets:          %d\r\n", mac_stats.num_packets);
  chprintf(chp, "    MAC general errors:   %d\r\n", mac_stats.errors);
  chprintf(chp, "    MAC underflow events: %d\r\n", mac_stats.underflow);
  chprintf(chp, "    MAC CRC5 errors:      %d\r\n", mac_stats.crc5_errors);
  chprintf(chp, "    MAC CRC16 errors:     %d\r\n", mac_stats.crc16_errors);
  chprintf(chp, "    MAC illegal PIDs:     %d\r\n", mac_stats.illegal_pids);
  chprintf(chp, "    Incoming read head:   %d\r\n", mac_stats.read_head);
  chprintf(chp, "    Incoming write head:  %d\r\n", mac_stats.write_head);
  chprintf(chp, "    Incoming buffer size: %d\r\n", mac_stats.buffer_size);

  chprintf(chp, "USB PHY statistics:\r\n");
  chprintf(chp, "    PHY packets:          %d\r\n", phy_stats.num_packets);
  chprintf(chp, "    PHY errors:           %d\r\n", phy_stats.errors);
  chprintf(chp, "    PHY overflow:         %d\r\n", phy_stats.overflow);
  chprintf(chp, "    PHY timeout:          %d\r\n", phy_stats.timeout);
  chprintf(chp, "    PHY no end-of-sync:   %d\r\n", phy_stats.no_end_of_sync);
  chprintf(chp, "    PHY no end-of-frame:  %d\r\n", phy_stats.no_end_of_frame);
  chprintf(chp, "    Outgoing read head:   %d\r\n", phy_stats.out_read_head);
  chprintf(chp, "    Outgoing write head:  %d\r\n", phy_stats.out_write_head);
  chprintf(chp, "    Outgoing buffer size: %d\r\n", phy_stats.out_buffer_size);

  usbPhyResetStatistics();
  usbMacResetStatistics();
}

palawan_command("usbmac", cmd_usbmac);
