#ifndef __PALAWAN_SHELL_H__
#define __PALAWAN_SHELL_H__

#include "hal.h"
#include "chprintf.h"
#include "palawan.h"
#include "shell.h"

void palawanShellInit(void);
void palawanShellRestart(void);

#define palawan_command_start() \
({ \
  static char start[0] __attribute__((unused,  \
    aligned(4), section(".chibi_list_cmd_1")));        \
  (const ShellCommand *)&start;            \
})

#define palawan_command(_name, _func) \
  const ShellCommand _palawan_cmd_list_##_func \
  __attribute__((unused, aligned(4), section(".chibi_list_cmd_2_" _name))) = \
     { _name, _func }

#define palawan_command_end() \
  const ShellCommand _palawan_cmd_list_##_func \
  __attribute__((unused, aligned(4), section(".chibi_list_cmd_3_end"))) = \
     { NULL, NULL }

#endif /* __PALAWAN_SHELL_H__ */
