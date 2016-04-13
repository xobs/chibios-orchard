/* hid_gadget_test */

#include <string.h>

#include "osal.h"
#include "chtypes.h"
#include "chstreams.h"
#include "chprintf.h"

#include "usbmac.h"
#include "palawan-shell.h"
#include "shell.h"

#define BUF_LEN 512

struct options {
	const char    *opt;
	unsigned char val;
};

static struct options kmod[] = {
	{.opt = "--left-ctrl",		.val = 0x01},
	{.opt = "--right-ctrl",		.val = 0x10},
	{.opt = "--left-shift",		.val = 0x02},
	{.opt = "--right-shift",	.val = 0x20},
	{.opt = "--left-alt",		.val = 0x04},
	{.opt = "--right-alt",		.val = 0x40},
	{.opt = "--left-meta",		.val = 0x08},
	{.opt = "--right-meta",		.val = 0x80},
	{.opt = NULL}
};

static struct options kval[] = {
	{.opt = "--return",	.val = 0x28},
	{.opt = "--esc",	.val = 0x29},
	{.opt = "--bckspc",	.val = 0x2a},
	{.opt = "--tab",	.val = 0x2b},
	{.opt = "--spacebar",	.val = 0x2c},
	{.opt = "--caps-lock",	.val = 0x39},
	{.opt = "--f1",		.val = 0x3a},
	{.opt = "--f2",		.val = 0x3b},
	{.opt = "--f3",		.val = 0x3c},
	{.opt = "--f4",		.val = 0x3d},
	{.opt = "--f5",		.val = 0x3e},
	{.opt = "--f6",		.val = 0x3f},
	{.opt = "--f7",		.val = 0x40},
	{.opt = "--f8",		.val = 0x41},
	{.opt = "--f9",		.val = 0x42},
	{.opt = "--f10",	.val = 0x43},
	{.opt = "--f11",	.val = 0x44},
	{.opt = "--f12",	.val = 0x45},
	{.opt = "--insert",	.val = 0x49},
	{.opt = "--home",	.val = 0x4a},
	{.opt = "--pageup",	.val = 0x4b},
	{.opt = "--del",	.val = 0x4c},
	{.opt = "--end",	.val = 0x4d},
	{.opt = "--pagedown",	.val = 0x4e},
	{.opt = "--right",	.val = 0x4f},
	{.opt = "--left",	.val = 0x50},
	{.opt = "--down",	.val = 0x51},
	{.opt = "--kp-enter",	.val = 0x58},
	{.opt = "--up",		.val = 0x52},
	{.opt = "--num-lock",	.val = 0x53},
	{.opt = NULL}
};

static int keyboard_fill_report(char report[8],
                                int argc, char **argv,
                                int *hold) {
	int key = 0;
	int i = 0;
  int argnum;
  const char *tok;

  for (argnum = 0; argnum < argc; argnum++) {
    tok = argv[argnum];

		if (strcmp(tok, "--quit") == 0)
			return -1;

		if (strcmp(tok, "--hold") == 0) {
			*hold = 1;
			continue;
		}

		if (key < 6) {
			for (i = 0; kval[i].opt != NULL; i++)
				if (strcmp(tok, kval[i].opt) == 0) {
					report[2 + key++] = kval[i].val;
					break;
				}
			if (kval[i].opt != NULL)
				continue;
		}

		if (key < 6) {
			if (tok[0] >= 'a' && tok[0] <= 'z') {
				report[2 + key++] = (tok[0] - ('a' - 0x04));
				continue;
			}
			if (tok[0] >= 'A' && tok[0] <= 'Z') {
				//report[0] = report[0] | 0x02;
				report[2 + key++] = (tok[0] - ('A' - 0x04));
				continue;
			}
			if (tok[0] >= '1' && tok[0] <= '0') {
				if (tok[0] == '0')
					report[2 + key++] = (tok[0] - ('0' - 39));
				else
					report[2 + key++] = (tok[0] - ('0' - 29));
				continue;
			}

			if (tok[0] == '-') {
				report[2 + key++] = 45;
				continue;
			}
		}

		for (i = 0; kmod[i].opt != NULL; i++)
			if (strcmp(tok, kmod[i].opt) == 0) {
				report[0] = report[0] | kmod[i].val;
				break;
			}
		if (kmod[i].opt != NULL)
			continue;
	}
	return 8;
}

#if 0
static struct options mmod[] = {
	{.opt = "--b1", .val = 0x01},
	{.opt = "--b2", .val = 0x02},
	{.opt = "--b3", .val = 0x04},
	{.opt = NULL}
};

int mouse_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
	char *tok = strtok(buf, " ");
	int mvt = 0;
	int i = 0;
	for (; tok != NULL; tok = strtok(NULL, " ")) {

		if (strcmp(tok, "--quit") == 0)
			return -1;

		if (strcmp(tok, "--hold") == 0) {
			*hold = 1;
			continue;
		}

		for (i = 0; mmod[i].opt != NULL; i++)
			if (strcmp(tok, mmod[i].opt) == 0) {
				report[0] = report[0] | mmod[i].val;
				break;
			}
		if (mmod[i].opt != NULL)
			continue;

		if (!(tok[0] == '-' && tok[1] == '-') && mvt < 2) {
			errno = 0;
			report[1 + mvt++] = (char)strtol(tok, NULL, 0);
			if (errno != 0) {
				fprintf(stderr, "Bad value:'%s'\r\n", tok);
				report[1 + mvt--] = 0;
			}
			continue;
		}

		fprintf(stderr, "unknown option: %s\r\n", tok);
	}
	return 3;
}

static struct options jmod[] = {
	{.opt = "--b1",		.val = 0x10},
	{.opt = "--b2",		.val = 0x20},
	{.opt = "--b3",		.val = 0x40},
	{.opt = "--b4",		.val = 0x80},
	{.opt = "--hat1",	.val = 0x00},
	{.opt = "--hat2",	.val = 0x01},
	{.opt = "--hat3",	.val = 0x02},
	{.opt = "--hat4",	.val = 0x03},
	{.opt = "--hatneutral",	.val = 0x04},
	{.opt = NULL}
};

int joystick_fill_report(char report[8], char buf[BUF_LEN], int *hold)
{
	char *tok = strtok(buf, " ");
	int mvt = 0;
	int i = 0;

	*hold = 1;

	/* set default hat position: neutral */
	report[3] = 0x04;

	for (; tok != NULL; tok = strtok(NULL, " ")) {

		if (strcmp(tok, "--quit") == 0)
			return -1;

		for (i = 0; jmod[i].opt != NULL; i++)
			if (strcmp(tok, jmod[i].opt) == 0) {
				report[3] = (report[3] & 0xF0) | jmod[i].val;
				break;
			}
		if (jmod[i].opt != NULL)
			continue;

		if (!(tok[0] == '-' && tok[1] == '-') && mvt < 3) {
			errno = 0;
			report[mvt++] = (char)strtol(tok, NULL, 0);
			if (errno != 0) {
				fprintf(stderr, "Bad value:'%s'\r\n", tok);
				report[mvt--] = 0;
			}
			continue;
		}

		fprintf(stderr, "unknown option: %s\r\n", tok);
	}
	return 4;
}
#endif

static void print_options(BaseSequentialStream *chp) {
	int i = 0;

  chprintf(chp, "keyboard options:\r\n"
                "\t--hold\r\n");
  for (i = 0; kmod[i].opt != NULL; i++)
    chprintf(chp, "\t%s\r\n", kmod[i].opt);
  chprintf(chp, "\r\nkeyboard values:\r\n"
                "\t[a-z] or\r\n");
  for (i = 0; kval[i].opt != NULL; i++)
    chprintf(chp, "\t%-8s%s", kval[i].opt, i % 2 ? "\r\n" : "");
  chprintf(chp, "\r\n");
}


static void cmd_usbkbd(BaseSequentialStream *chp, int argc, char *argv[])
{
  uint32_t report_32[2] = {};
	char *report = (char *)report_32;
	int to_send = 8;
	int hold = 0;

  if (!argc) {
    print_options(chp);
    usbSendData(usbMacDefault(), 1, report_32, 8);
    return;
  }

  to_send = keyboard_fill_report(report, argc, argv, &hold);

  if (to_send <= 0)
    return;

  usbSendData(usbMacDefault(), 1, report_32, to_send);

  if (!hold) {
    memset(report_32, 0x0, sizeof(report_32));
    usbSendData(usbMacDefault(), 1, report_32, 8);
  }
}

palawan_command("kbd", cmd_usbkbd);
