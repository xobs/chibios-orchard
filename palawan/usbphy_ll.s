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
reg     .req r3   // Offset to the GPIO block
mask    .req r4   // Mask within the GPIO block
shift   .req r4   // Shift within the GPIO block (shared with mask)
val1    .req r5
val2    .req r6
tmp     .req r7
lastbit .req r12

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
  uint32_t usbdpAddr;
  uint32_t usbdpMask;
  uint32_t usbdpShift;

  uint32_t usbdnAddr;
  uint32_t usbdnMask;
  uint32_t usbdnShift;

  uint32_t ticks;
} __attribute__((__packed__));
*/

.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t *samples, int max_samples)*/:
  push {samples,r4,r5,r6,r7}

//  ldr reg, [usbphy, #12]         // load the gpio register into r3
//  ldr mask, [usbphy, #16]        // Mask to get USB line bit from register
  ldr reg, [usbphy, #0]         // load the gpio register into r3
  ldr mask, [usbphy, #4]        // Mask to get USB line bit from register

  /* Wait for the line to shift */
  ldr val1, [reg]               // Sample USBDP
  and val1, val1, mask          // Mask off the interesting bit

  mov r7, #5                    // The loop is 9 cycles on a failure.  One
                                // pulse is 32 cycles.  Therefore, loop up
                                // to 5 times before giving up.

  /* Wait until midway through the next pulse */
wait_for_line_flip:
  ldr val2, [reg]               // Sample USBDP
  and val2, val2, mask          // Mask off the interesting bit

  sub r7, r7, #1
  cmp r7, #0
  beq timeout

  cmp val1, val2                // Wait for it to change
  beq wait_for_line_flip

  // There should be about 16 instructions between "ldr" above and "ldr" below.
  // Minus up to 8 instructions for the "beq" path
.rept 2
  nop
.endr

  /* Grab the next bit off the USB signals */
  mov tmp, #0
  mov lastbit, tmp              // Reset our last bit, to look for SE0
get_usb_bit:
.rept 4
  nop
.endr

  /* Now we're lined up in the middle of the pulse */
  ldr val1, [usbphy, #0]        // load USBDP register into temp reg
  ldr val2, [usbphy, #12]       // load USBDN register into temp reg
  ldr val1, [val1]              // Sample USBDP
  ldr val2, [val2]              // Sample USBDN

  ldr shift, [usbphy, #8]       // Load the USBDP shift value
  asr val1, val1, shift         // Shift the USBDP value down to bit 1
  mov mask, #1                  // Since it's bit 1, we'll mask by 1
  and val1, val1, mask          // Perform the mask by 0x1

  ldr shift, [usbphy, #20]      // Load the USBDN shift value
                                // We want to shift by one less, to move to
  sub shift, shift, #1          // bit 2, so subtract 1 from the shift.
  asr val2, val2, shift         // Perform the shift
  mov mask, #2                  // Now, mask the value by 2.
  and val2, val2, mask          // Perform the mask by 0x2.

  orr val1, val1, val2          // OR the two bits together.

  strb val1, [samples]          // Store the value in our sample buffer.

  add lastbit, lastbit, val1    // An end-of-frame is indicated by two
                                // frames of SE0.  If this is the case,
                                // then the result of adding these together
                                // will result in 0.
  beq exit                      // Exit if so.
  mov lastbit, val1

  add samples, samples, #1      // Move the sample buffer up by 1.
  sub count, count, #1          // Subtract 1 from the max sample number.

  cmp count, #0                 // See if there are any samples left.

  bne get_usb_bit               // If so, obtain another bit.

exit:
  mov count, samples
  pop {samples,r4,r5,r6,r7}
  sub r0, count, samples        // Report how many bits we received
  bx lr

timeout:
  pop {samples,r4,r5,r6,r7}
  mov r0, #0
  sub r0, r0, #1
  bx lr
.endfunc
.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead


/*
.align  2
.thumb_func
.global NMI_Handler
NMI_Handler:
  ldr r0, SIM_SCGC5
  ldr r0, [r0]
  ldr r1, SIM_SCGC5_default
  cmp r0, r1

  beq do_reset

  ldr r2, NMI_Func
  mov pc, r2

do_reset:
  ldr r2, Reset_Func
  mov pc, r2
.type NMI_Handler, %function
.size NMI_Handler, .-NMI_Handler

.balign 4
Reset_Func:
.word Reset_Handler
NMI_Func:
.word usbPhyISR

.balign 4
SIM_SCGC5:
.word 0x40048038
SIM_SCGC5_default:
.word 0x00000182
*/

.balign 4
SysTick_BASE:
.word 0xE000E000

.end
