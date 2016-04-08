#include <stdint.h>
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
