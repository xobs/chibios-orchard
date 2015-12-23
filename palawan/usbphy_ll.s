.text

  /******************************************************
   * USB PHY low-level code
   * 
   * Exports the following functions:
   *
   *    int usbPhyRead(USBPhy *phy, uint8_t *buffer)
   *    void usbPhyWrite(USBPhy *phy, uint8_t buffer, uint32_t count)
   *
   * Interrupts are disabled during this code, since it is time-critical.
   * Additionally, timing is maintained by examining the SysTick timer,
   * so it is reconfigured during the course of running this code.
   *
   * A USB frame begins with the pattern KJKJKJKK.  Each bit takes about
   * 666 nS, which at a 48 MHz clock rate gives us 32 cycles to read
   * each bit.
   *
   * An interrupt takes 16 clock cycles, and it probably took at least 32
   * clock cycles to get here, meaning we've already lost the first KJ.
   * The first thing to do is measure how many ticks it takes for a
   * transition.  To do that, perform the following:
   *
   *    0) Load a large value into SysTick Current
   *    1) Wait for the line to transition
   *    2) Compare the SysTick current
   *    3) Divide by two, subtract residual cycles, and load that value
   *       into SysTick current
   *    4) Load the difference into SysTick reload
   *    5) Wait for SysTick to loop
   *
   * At this point, we should be synchronized with the middle of the
   * pulse.  SysTick will roll over once per clock cycle, in the middle,
   * when it's good to sample.
   *
   * Now, wait for the end-of-sync pulse "KK" state.
   *
   * Continue reading bits in until we get a double-SE0.  Actually, any
   * SE0 should be considered end-of-frame.
   *
   * Afterwards, restore the SysTick register.
   */

usbphy  .req r0
samples .req r1
count   .req r2
gpio    .req r3   // Offset to the GPIO block

delays  .req r7

/* Variables in use:
  usbphy   -- Pointer to a USBPHY struct
  samples  -- Pointer to the sampled values
  count    -- Maximum size of the sample array
  gpio     -- Offset to GPIO block
  usbdpo   -- Offset to register bank for USBDP GPIO
  usbdno   -- Offset to register bank for USBDN GPIO
  usbdps   -- Shift for USBDP GPIO
  usbdns   -- Shift for USBDN GPIO (plus 1)
  usbdpm   -- Mask for USBDP GPIO
  usbdnm   -- Mask for USBDN GPIO
  systick  -- Offset to SysTick register
  ticks    -- Number of ticks per frame
  tickcfg  -- SysTick config, mostly interested in whether it's rolled over
  usbdp    -- Sample of USBDP
  usbdn    -- Sample of USBDS


  // r0 is the USBPHY struct (see below)
  // r1 is the sample buffer
  // r2 is the number of [remaining] max samples
  // r3 is the FGPIO offset

/*
static struct USBPHY {
  uint32_t gpioBase;
  uint32_t udbdpOffset;
  uint32_t usbdpMask;
  uint32_t usbdnOffset;
  uint32_t usbdnMask;
  uint32_t ticks;
  uint32_t usbdpShift;
  uint32_t usbdnShift;
} __attribute__((__packed__));
*/
	
.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t *samples, int max_samples)*/:
	push {r4,r5,r6,r7}

  ldr gpio, [usbphy, #0]        // load the gpio register into r3

  mov r4, #1

  /* Wait for the line to shift */
  ldr r5, [gpio, #0x50]            // Sample USBDP
  and r5, r5, r4

wait_for_line_flip:
  ldr r6, [gpio, #0x50]            // Sample USBDP
  and r6, r6, r4
  cmp r6, r4
  beq wait_for_line_flip

  /* Wait until midway through the next pulse */
get_usb_bit:
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop

  /* Now we're lined up in the middle of the pulse */

  ldr r5, [gpio, #0x10]            // Sample USBDN
  ldr r4, [gpio, #0x50]            // Sample USBDP

  mov r6, #1                    // 1
  and r4, r4, r6                // 1

  mov r6, #2                    // 1
  asr r5, r5, #3                // 1
  and r5, r5, r6                // 1

  orr r4, r4, r5                // 1

  strb r4, [samples]            // 1
  add samples, samples, #1      // 1
  sub count, count, #1          // 1

  cmp count, #0                 // 1

  bne get_usb_bit               // 2

exit:	
	
	pop {r4,r5,r6,r7}
	bx lr

.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead

.balign 4
SysTick_BASE:
.word 0xE000E000

.end
	
