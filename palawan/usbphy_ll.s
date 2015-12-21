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
systick .req r3
gpio    .req r4   // Offset to the GPIO block

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
  // r3 is the SysTick offset
  // r4 is the FGPIO offset

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

  ldr systick, SysTick_BASE
  ldr r4, [systick, #0x10]       // Load SysTick_CTRL
  ldr r5, [systick, #0x14]       // Load SysTick_RELOAD
  ldr r6, [systick, #0x18]       // Load SysTick_CURRENT
  push {r4,r5,r6}                // Save these values

  /* Load a large value into SysTick */
  mov r4, #0x5                  // Set CLKSOURCE and ENABLE for SysTick_CTRL
	str r4, [systick, #0x10]      // Load the mask into SysTick_CTRL
  ldr r4, [usbphy, #20]         // Load the number of ticks into SysTick_RELOAD
	str r4, [systick, #0x14]      // Load the large value into SysTick_RELOAD
	str r4, [systick, #0x18]      // Load the large value into SysTick_CURRENT
  ldr r4, [systick, #0x10]      // Load SysTick_CTRL, which clears COUNTFLAG

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
#if 0
  /*~~~~~~~~~~ Now SysTick is prepped for measurement ~~~~~~~~~~~*/

  /* Wait for the line to change from a 1 to a 0 */
  ldr gpio, [usbphy, #0]        // load the gpio register
  ldr r6, [usbphy, #4]          // Load the USBDP offset into r6
  ldr r7, [usbphy, #8]          // Load the USBDP mask into r7

  /* Check to see if the pin is already 0, in which case wait for it to go 1 */
  ldr r5, [gpio, r6]            // Sample the pin
  tst r5, r7
  beq line_has_1                // If the line mask is set, then we're at a 1

  /* The line is 0, so wait for it to go 1. */
line_has_0:
  ldr r5, [gpio, r6]            // Sample the pin
  tst r5, r7
  bne line_has_0                // EQ means Z flag was set, meaning it's 0.

line_has_1:
  /* The line is now at 1.  Wait for it to transition to 0, so we can time it */
  ldr r5, [gpio, r6]            // Sample the pin
  tst r5, r7
  beq line_has_1                // NE means Z flag was clear, meaning it's 1.

  /* The line has just transitioned to 0 (within the last four cycles). */
  ldr r5, [usbphy, #20]         // Wait for half of SysTick
  asr r5, r5, #1
  sub r5, r5, #6                // Skip several cycles to compensate for
                                // the instructions required to load the
                                // half-SysTick value.
  str r5, [systick, #0x18]
#endif

  ldr r5, [systick, #0x10]
  /* Wait until midway through the next pulse */
get_usb_bit:
  mov r5, #1
  lsl r5, r5, #16
wait_for_sync_pulse:
  ldr r6, [systick, #0x10]
  tst r6, r5
  bne wait_for_sync_pulse

  /* Now we're lined up in the middle of the pulse */


  ldr gpio, [usbphy, #0]        // load the gpio register into r5

  ldr r6, [gpio, #0x50]            // Sample USBDP
  ldr r7, [gpio, #0x10]            // Sample USBDN

  mov r5, #1
  and r6, r6, r5

  mov r5, #2
  asr r7, r7, #3
  and r7, r7, r5
  orr r6, r6, r7

  /*
  ldr r5, [usbphy, #8]          // Load the USBDP mask into r6

  mov r3, #0
  tst r6, r5
  bne sample_dn
  mov r3, #1

sample_dn:
  ldr r5, [usbphy, #16]         // Load the USBDN mask into r7
  tst r5, r7
  bne skip_bit2
  mov r5, #2
  orr r3, r5, r3
skip_bit2:
*/

  strb r6, [samples]            // 2
  add samples, samples, #1      // 1
  sub count, count, #1          // 1

  cmp count, #0                 // 1

  bne get_usb_bit               // 3

exit:	
	
	pop {r4,r5,r6}                // Restore SysTick values
  ldr systick, SysTick_BASE
  str r4, [systick, #0x10]      // Restore SysTick_CTRL
  str r5, [systick, #0x14]      // Restore SysTick_RELOAD
  str r6, [systick, #0x18]      // Restore SysTick_CURRENT

	pop {r4,r5,r6,r7}
	bx lr

.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead

.balign 4
SysTick_BASE:
.word 0xE000E000

.end
	
