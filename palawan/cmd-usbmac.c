#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "palawan-shell.h"
#include "usbphy.h"
#include "usbmac.h"

static const char *pid_to_str(uint8_t pid) {
  switch (pid) {
    /* Data packets */
    case 0xf0: return "MDATA";
    case 0xe1: return "DATA2";
    case 0xd2: return "DATA1";
    case 0xc3: return "DATA0";

    /* Token packets, with CRC-5 */
    case 0xb4: return "SETUP";
    case 0xa5: return "SOF";
    case 0x96: return "IN";
    case 0x87: return "OUT";

    /* One-byte packets, no CRC required */
    case 0x78: return "STALL";
    case 0x69: return "NYET";
    case 0x5a: return "NAK";
    case 0x4b: return "ACK";

    /* Special cases (also no CRC?) */
    case 0x3c: return "PRE";
    case 0x2d: return "PING";
    case 0x1e: return "SPLIT";
    case 0x0f: return "RESERVED";

    default:   return "UNKNOWN";
  }
}


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
  while ((packet = usbMacGetPacket()) != NULL) {
    count++;
    chprintf(chp, "Packet %d:\r\n", count);
    chprintf(chp, "    %02x %s\r\n", packet->pid, pid_to_str(packet->pid));
    chprintf(chp, "    %d bytes of data\r\n", packet->size);
    if (packet->size) {
      chprintf(chp, "   ");
      for (i = 0; i < packet->size; i++)
        chprintf(chp, " %02x", packet->data[i]);
      chprintf(chp, "\r\n");
    }
  }

  chprintf(chp, "USB processed %d packets\r\n", count);

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
  chprintf(chp, "    PHY underflow:        %d\r\n", phy_stats.underflow);
  chprintf(chp, "    PHY overflow:         %d\r\n", phy_stats.overflow);
  chprintf(chp, "    PHY timeout:          %d\r\n", phy_stats.timeout);
  chprintf(chp, "    PHY no end-of-sync:   %d\r\n", phy_stats.no_end_of_sync);
  chprintf(chp, "    PHY no end-of-frame:  %d\r\n", phy_stats.no_end_of_frame);
  chprintf(chp, "    Incoming read head:   %d\r\n", phy_stats.in_read_head);
  chprintf(chp, "    Incoming write head:  %d\r\n", phy_stats.in_write_head);
  chprintf(chp, "    Incoming buffer size: %d\r\n", phy_stats.in_buffer_size);
  chprintf(chp, "    Outgoing read head:   %d\r\n", phy_stats.out_read_head);
  chprintf(chp, "    Outgoing write head:  %d\r\n", phy_stats.out_write_head);
  chprintf(chp, "    Outgoing buffer size: %d\r\n", phy_stats.out_buffer_size);

  chprintf(chp, "USB PHY Events:\r\n");
  for (i = 0; i < phy_stats.timestamp_count; i++) {
    uint32_t event = phy_stats.timestamps[i];
    chprintf(chp, "      %3d: %c %d\r\n",
        i,
        event & 0x00010000 ? 'i' : 'o',
        event & 0xffff);
  }

  usbPhyResetStatistics();
  usbMacResetStatistics();
}

palawan_command("usbmac", cmd_usbmac);
