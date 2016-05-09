#include <stdint.h>
#include <string.h>
#include "chtypes.h"
#include "chstreams.h"
#include "chprintf.h"
#include "osal.h"

#include "usbphy.h"
#include "palawan-shell.h"
#include "shell.h"

static void cmd_usbattach(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  chprintf(chp, "Attaching USB device... ");
  usbPhyAttach(usbPhyDefaultPhy());
  chprintf(chp, "Done.\r\n");
}

palawan_command("attach", cmd_usbattach);

static void cmd_usbdetach(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  chprintf(chp, "Detaching USB device... ");
  usbPhyDetach(usbPhyDefaultPhy());
  chprintf(chp, "Done.\r\n");
}

palawan_command("detach", cmd_usbdetach);

#if defined(ENABLE_TEST_READ)
static struct USBPHY testReadPhy = {
  /* Pins J21 and J19 */
  /* PTD5 */
  .usbdpIAddr = &FGPIOD->PDIR,
  .usbdpSAddr = &FGPIOD->PSOR,
  .usbdpCAddr = &FGPIOD->PCOR,
  .usbdpDAddr = &FGPIOD->PDDR,
  .usbdpMask  = (1 << 5),
  .usbdpShift = 5,

  /* PTD6 */
  .usbdnIAddr = &FGPIOD->PDIR,
  .usbdnSAddr = &FGPIOD->PSOR,
  .usbdnCAddr = &FGPIOD->PCOR,
  .usbdnDAddr = &FGPIOD->PDDR,
  .usbdnMask  = (1 << 6),
  .usbdnShift = 6,
};

static void cmd_usbcap(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  struct USBPHY *phy = &testReadPhy;
  uint32_t sample_before, sample_after;
  uint8_t buffer[13];
  int count;
  int i;

  memset(buffer, 0, sizeof(buffer));
  chprintf(chp, "Capturing USB stream... ");
  *(phy->usbdnDAddr) &= ~phy->usbdnMask;
  *(phy->usbdpDAddr) &= ~phy->usbdpMask;

  osalSysLock();
  /* Wait for the line to flip */
  sample_after = sample_before = *(phy->usbdpIAddr) & phy->usbdpMask;
  while (sample_before == sample_after)
    sample_after = (*(phy->usbdpIAddr) & phy->usbdpMask);
  count = usbPhyReadI(phy, buffer);
  osalSysUnlock();

  if (count < 0) {
    chprintf(chp, "Error: %d\r\n", count);
    return;
  }

  chprintf(chp, "Read %d bytes:\r\n", count);
  chprintf(chp, "    [");

  for (i = 0; i < count; i++)
    chprintf(chp, "%s0x%02x", i==0?"":", ", buffer[i]);
  chprintf(chp, "]\r\n");
}
palawan_command("cap", cmd_usbcap);
#endif /* ENABLE_TEST_READ */
