#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "usbphy.h"
#include "palawan-shell.h"
#include "hex.h"
#include "string.h"

struct USBPHY {
  volatile uint32_t *usbdpIAddr;
  volatile uint32_t *usbdpSAddr;
  volatile uint32_t *usbdpCAddr;
  volatile uint32_t *usbdpDAddr;
  uint32_t usbdpMask;
  uint32_t usbdpShift;

  volatile uint32_t *usbdnIAddr;
  volatile uint32_t *usbdnSAddr;
  volatile uint32_t *usbdnCAddr;
  volatile uint32_t *usbdnDAddr;
  uint32_t usbdnMask;
  uint32_t usbdnShift;

  uint32_t ticks;
} __attribute__((packed, aligned(4)));

enum state {
  state_se0,
  state_k,
  state_j,
  state_se1,
};

//#define USB_PHY_LL_DEBUG
static struct USBPHY usbPhy = {
  /* PTB0 */
  .usbdpIAddr = &FGPIOB->PDIR,
  .usbdpSAddr = &FGPIOB->PSOR,
  .usbdpCAddr = &FGPIOB->PCOR,
  .usbdpDAddr = &FGPIOB->PDDR,
  .usbdpMask = (1 << 0),
  .usbdpShift = 0,

  /* PTA4 */
  .usbdnIAddr = &FGPIOA->PDIR,
  .usbdnSAddr = &FGPIOA->PSOR,
  .usbdnCAddr = &FGPIOA->PCOR,
  .usbdnDAddr = &FGPIOA->PDDR,
  .usbdnMask = (1 << 4),
  .usbdnShift = 4,
};

static uint8_t test_setup0_setup[] = {0x2D, 0x00, 0x10};
static uint8_t test_setup0_data[] = {0xC3, 0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00, 0xDD, 0x94};
static uint8_t test_pattern0[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t test_pattern1[] = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa, 0x99, 0x88, 0x77, 0x66, 0x55};
static uint8_t test_pattern2[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef, 0x00, 0xff, 0xaa};
static uint8_t test_pattern3[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
static uint8_t test_pattern4[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};
static uint8_t test_pattern5[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint8_t test_pattern6[] = {0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0x00};
static uint8_t test_pattern7[] = {0xff};
static uint8_t test_pattern8[] = {0xfe, 0x00, 0xf8};
static uint8_t test_pattern9[] = {0xfe, 0x00};

static uint32_t usbdn_buffer[140];
static uint32_t usbdp_buffer[140];
static uint8_t pair_buffer[140];

static int usb_test_to_buffer(struct USBPHY *usbPhy,
                              const uint8_t *input, uint32_t bytes) {
  unsigned int out_bits;
  unsigned int input_byte;
  int input_bit;
  unsigned int run_length;
  enum state last_state;
  enum state cur_state;

  usbPhy->usbdpIAddr = usbdp_buffer;
  usbPhy->usbdnIAddr = usbdn_buffer;

#define add_state(s) \
  do { \
    usbdp_buffer[out_bits] = (!!(s & 1)) << usbPhy->usbdpShift; \
    usbdn_buffer[out_bits] = (!!(s & 2)) << usbPhy->usbdnShift; \
    pair_buffer[out_bits] = s; \
    out_bits++; \
  } while(0)

  out_bits = 0;
  input_bit = 0;
  input_byte = 0;
  run_length = 0;

  /* Generate the start-of-frame pattern */
  add_state(state_k);
  add_state(state_j);
  add_state(state_k);
  add_state(state_j);
  add_state(state_k);
  add_state(state_j);
  add_state(state_k);
  add_state(state_k);

  last_state = cur_state = state_k;
  run_length = 2;

  for (input_byte = 0; input_byte < bytes; input_byte++) {
    for (input_bit = 7; input_bit >= 0; input_bit--) {

      /* Bit-stuffing.  More than 6 1s in a row get turned into a 0 */
      if (run_length >= 6) {
        if (cur_state == state_j)
          cur_state = state_k;
        else
          cur_state = state_j;
        add_state(cur_state);
        last_state = cur_state;
        run_length = 1;
      }

      if (input[input_byte] & (1 << input_bit)) {
        cur_state = last_state;
        run_length++;
      }
      else {
        run_length = 0;
        if (cur_state == state_j)
          cur_state = state_k;
        else
          cur_state = state_j;
      }
      add_state(cur_state);
      last_state = cur_state;
    }
  }

  /* Generate the end-of-frame pattern */
  add_state(state_se0);
  add_state(state_se0);
  return out_bits;
}

static void asmtest(BaseSequentialStream *chp) {
  register int a, b, c, d;
  int i;

  a = 0; b = 0; c = 1; d = 0;
  chprintf(chp, "Start: %08x %08x %08x\r\n", a, b, c);

  for (i = 0; i < 68; i++) {
    asm(
        "add %[d], %[d], %[d]\n\t"
        "adc %[c], %[c], %[c]\n\t"
        "adc %[b], %[b], %[b]\n\t"
        "adc %[a], %[a], %[a]\n\t"
        : [a] "+r" (a) , [b] "+r" (b), [c] "+r" (c), [d] "+r" (d)
       );
    chprintf(chp, "  %03d: %08x %08x %08x\r\n", i, a, b, c);
  }
}

static void cmd_usbphy(BaseSequentialStream *chp, int argc, char *argv[])
{

  int packet_bits;
  int ret;
  uint8_t buffer[11];
  uint32_t scratch[3];
  uint8_t *test_data;
  uint32_t test_size;

  if (argc > 0) {
    if (argv[0][0] == 'd') {
      test_data = test_setup0_data;
      test_size = sizeof(test_setup0_data);
    }
    else if (argv[0][0] == '0') {
      test_data = test_pattern0;
      test_size = sizeof(test_pattern0);
    }
    else if (argv[0][0] == '1') {
      test_data = test_pattern1;
      test_size = sizeof(test_pattern1);
    }
    else if (argv[0][0] == '2') {
      test_data = test_pattern2;
      test_size = sizeof(test_pattern2);
    }
    else if (argv[0][0] == '3') {
      test_data = test_pattern3;
      test_size = sizeof(test_pattern3);
    }
    else if (argv[0][0] == '4') {
      test_data = test_pattern4;
      test_size = sizeof(test_pattern4);
    }
    else if (argv[0][0] == '5') {
      test_data = test_pattern5;
      test_size = sizeof(test_pattern5);
    }
    else if (argv[0][0] == '6') {
      test_data = test_pattern6;
      test_size = sizeof(test_pattern6);
    }
    else if (argv[0][0] == '7') {
      test_data = test_pattern7;
      test_size = sizeof(test_pattern7);
    }
    else if (argv[0][0] == '8') {
      test_data = test_pattern8;
      test_size = sizeof(test_pattern8);
    }
    else if (argv[0][0] == '9') {
      test_data = test_pattern9;
      test_size = sizeof(test_pattern9);
    }
    else {
      test_data = test_setup0_setup;
      test_size = sizeof(test_setup0_setup);
    }
  }
  else {
    test_data = test_setup0_setup;
    test_size = sizeof(test_setup0_setup);
  }

  chprintf(chp, "Testing packet\r\n");
  asmtest(chp);

  packet_bits = usb_test_to_buffer(&usbPhy, test_data, test_size);
  chprintf(chp, "Packet to test is %d bytes, which expands to %d bits.\r\n",
                test_size, packet_bits);

  {
    int i;
    int count = packet_bits;
    static const char *states = "0KJ1";
    chprintf(chp, "    States: ");
    for (i = 0; i < count; i++)
      chprintf(chp, "%c", states[pair_buffer[i]]);
    chprintf(chp, "\r\n");

    int diff = pair_buffer[0];
    chprintf(chp, "    Diffs:  ");
    for (i = 1; i < count; i++) {
      chprintf(chp, " %d", (pair_buffer[i] - diff) & 0x3);
      diff = pair_buffer[i];
    }
    chprintf(chp, "\r\n");

    chprintf(chp, "    Decode: ");
    for (i = 1; i < count; i++)
      chprintf(chp, " %c", pair_buffer[i] == pair_buffer[i - 1] ? '1' : '0');
    chprintf(chp, "\r\n");
  }

  uint32_t start = (uint32_t)usbPhy.usbdpIAddr;
  memset(buffer, 0, sizeof(buffer));
  ret = usbPhyRead(&usbPhy, buffer, scratch);
  uint32_t end = (uint32_t)usbPhy.usbdpIAddr;

  chprintf(chp, "usbPhyRead() returned %d, read %d bits\r\n",
                ret, (end - start)/ 4);

  chprintf(chp, "Original packet:\r\n");
  print_hex(chp, test_data, test_size);

  chprintf(chp, "Read-back packet (result: %d):\r\n", ret);
  print_hex(chp, buffer, sizeof(buffer));
  print_hex(chp, scratch, sizeof(scratch));
}

palawan_command("usbphy", cmd_usbphy);
