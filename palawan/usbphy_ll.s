#if 0
.text

  /***************************************************************************
   * USB PHY low-level code
   * 
   * Exports the following functions:
   *
   *    int usbPhyRead(USBPhy *phy, uint8_t buffer[11])
   *    void usbPhyWrite(USBPhy *phy, uint8_t buffer[11], uint32_t count)
   *
   * Interrupts are disabled during this code, since it is time-critical.
   * Note that as a Kinetis "feature", jumps of more than 48 bytes can
   * cause random amounts of jitter.  Make sure you don't do that.
   *
   * Both functions take the following struct as their first parameter:
   *
   *  static struct USBPHY {
   *    /* USB D- line descriptor */
   *    uint32_t dpIAddr; /* GPIO "sample-whole-bank" address */
   *    uint32_t dpSAddr; /* GPIO "set-pin-level" address */
   *    uint32_t dpCAddr; /* GPIO "clear-pin-level" address */
   *    uint32_t dpDAddr; /* GPIO "pin-direction" address, where 1 = output */
   *    uint32_t dpMask;  /* Mask of GPIO pin in S/C/D/I addresses */
   *    uint32_t dpShift; /* Shift of GPIO pin in S/C/D/I addresses */
   *
   *    /* USB D+ line descriptor, as above */
   *    uint32_t dnIAddr;
   *    uint32_t dnSAddr;
   *    uint32_t dnCAddr;
   *    uint32_t dnDAddr;
   *    uint32_t dnMask;
   *    uint32_t dnShift;
   *
   *    /* Unused, but the number of CPU ticks/cycles for one USB bit */
   *    uint32_t ticks;
   * };
   */


/* usbphy offsets */
.equ dpIAddr,0x00
.equ dpSAddr,0x04
.equ dpCAddr,0x08
.equ dpDAddr,0x0c
.equ dpMask,0x10
.equ dpShift,0x14

.equ dnIAddr,0x18
.equ dnSAddr,0x1c
.equ dnCAddr,0x20
.equ dnDAddr,0x24
.equ dnMask,0x28
.equ dnShift,0x2c

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

} __attribute__((__packed__));
*/
  /*
   *
   * Each USB bit takes about 666 nS, which at a 48 MHz clock rate gives us
   * 32 cycles to read each bit.  This code runs on a Cortex M0+, where each
   * instruction is one cycle, except taken-branches are three cycles.
   *
   * A USB frame begins with the pattern KJKJ...KK.  A USB frame ends with
   * a double-SE0 state.  USB low-speed packets have an 8-bit sync period, and
   * a maximum of 11 bytes.  Thus, a USB low-speed packet looks like this:
   *
   *     KJKJKJKK | data | 00
   *
   * Our usbReadData() code will start by positioning ourselves in the
   * middle of a pulse.  We do that by samping the line, then waiting for
   * it to change, then waiting some number of cycles.
   *
   * Once we're positioned, we then start looking for the KK end-of-sync
   * indicator.  An interrupt takes 16 clock cycles, and it probably took
   * at least 32 clock cycles to get here, meaning we've already lost the
   * first KJ.  This is fine, as we just need to look for "KK" to indicate
   * the end of the sync period.
   *
   * Since a USB low-speed packet is at most 11 bytes, we can store this in
   * three 32-bit registers.  We chain three registers together in a shift-
   * chain by self-adding-with-carry on each of the three registers in
   * sequence to move the top bit from one into the bottom bit of the next.
   *
   * Add a 1 to the low register if the state is the same as the previous
   * state, and add a 0 to the low register if the state has changed.
   *
   * As a special case, when we get six consecutive bits in a row (i.e. six
   * ones or six zeroes), the host will "stuff" one bit and flip the state,
   * meaning we should ignore that 1-bit.  If this is the case, the shift
   * should not be processed.
   *
   * Continue reading bits in until we get a double-SE0.  Actually, any
   * SE0 should be considered end-of-frame.
   *
   * r0
   */
usbphy  .req r0   /* Pointer to the USBPHY struct, described above */
outptr  .req r1   /* Outgoing sample buffer */
reg     .req r2   /* Register to sample pin */
mask    .req r3   /* Mask to isolate required pin */
val     .req r4   /* Currently-sampled value */
lastval .req r5   /* What value was the last pin? */

sample1 .req r7   /* Most recent 32-bits */
sample2 .req r8   /* Next 32-bits */
sample3 .req r9   /* Remaining 24-bits */

.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t samples[11])*/:
  push {samples,r4,r5,r6,r7}

  /* Clear out the register shift-chain */
  mov r7, #0
  mov r8, #0
  mov r9, #0

  ldr reg, [usbphy, #dpIAddr]   // load the gpio register into r3
  ldr mask, [usbphy, #dpMask]   // Mask to get USB line bit from register

  /* Wait for the line to shift */
  ldr lastval, [reg]            // Sample USBDP
  and lastval, lastval, mask    // Mask off the interesting bit

  // The loop is 4 cycles on a failure.  One
  // pulse is 32 cycles.  Therefore, loop up
  // to 9 times before giving up.
.rept 11
  ldr val, [reg]                // Sample USBDP
  and val, val, mask            // Mask off the interesting bit
  cmp val, lastval              // Wait for it to change
  bne usb_phy_read_sync_wait
.endr
  b usb_phy_read_timeout

usb_phy_read_sync_wait:
  // There should be about 16 instructions between "ldr" above and "ldr" below.
  // Minus up to 8 instructions for the "beq" path
  b wait_13_cycles

  /* Grab the next bit off the USB signals */
  mov tmp, #4                   // Reset our last bit, to look for SE0
get_usb_bit:

  /* Now we're lined up in the middle of the pulse */
  ldr val1, [usbphy, #dpIAddr]  // load USBDP register into temp reg
  ldr val2, [usbphy, #dnIAddr]  // load USBDN register into temp reg
  ldr val1, [val1]              // Sample USBDP
  ldr val2, [val2]              // Sample USBDN

  ldr shift, [usbphy, #dpShift] // Load the USBDP shift value
  ror val1, val1, shift         // Shift the USBDP value down to bit 1
  mov mask, #1                  // Since it's bit 1, we'll mask by 1
  and val1, val1, mask          // Perform the mask by 0x1

  ldr shift, [usbphy, #dnShift] // Load the USBDN shift value
  ror val2, val2, shift         // Perform the shift
  mov mask, #1                  // Load 0x1, for the mask operation.
  and val2, val2, mask          // Perform the mask by 0x1.

                                // Move it up to bit 2, so we can or the
  lsl val2, val2, mask          // two values together (reusing the mask value)

  orr val1, val1, val2          // OR the two bits together.

  strb val1, [samples]          // Store the value in our sample buffer.

  //add mask, mask, usbphy // XXX
  add tmp, tmp, val1    // An end-of-frame is indicated by two
                                // frames of SE0.  If this is the case,
                                // then the result of adding these together
                                // will result in 0.
  cmp tmp, #0
  beq exit                      // Exit if so.
  mov tmp, val1

.rept 1 // Number of spare cycles to use
  nop
.endr

  add samples, samples, #1      // Move the sample buffer up by 1.
  sub count, count, #1          // Subtract 1 from the max sample number.

  cmp count, #0                 // See if there are any samples left.

  bne get_usb_bit               // If so, obtain another bit.

phy_overflow:                   // If not, we've overflowed the buffer
  pop {samples,r4,r5,r6,r7}
  mov r0, #0
  sub r0, r0, #2                // Return -2
  bx lr

exit:
  mov count, samples
  pop {samples,r4,r5,r6,r7}
  sub r0, count, samples        // Report how many bits we received
  sub r0, r0, #1                // Remove extra trailing SE0
  bx lr

timeout:
  pop {samples,r4,r5,r6,r7}
  mov r0, #0
  sub r0, r0, #1
  bx lr
.endfunc
.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead
.global usbPhyRead_size
.align 4
usbPhyRead_size: .long usbPhyRead, .-usbPhyRead

.global usbPhyRead_end
usbPhyRead_end: nop



/*
 * Registers:
 *   r0: USBPHY
 *   r1: sample buffer
 *   r2: number of samples to write
 *   r3: dp set/clear
 *   r4: dn set/clear
 *   r5: dp mask
 *   r6: dn mask
 *   r7: tmp
 */
.func usbPhyWrite
.global usbPhyWrite
/*int */usbPhyWrite/*(const USBPHY *phy, uint8_t *samples, int max_samples)*/:
  push {r4,r5,r6,r7}
  // First, set both lines to OUTPUT
  ldr reg, [usbphy, #dpDAddr]   // Get the direction address
  ldr val1, [reg]               // Get the direction value
  ldr val2, [usbphy, #dpMask]   // Get the mask value for ORing in
  orr val1, val1, val2          // Set the direciton mask
  ldr reg, [usbphy, #dpDAddr]   // Get the direction address
  str val1, [reg]               // Set the direction for Dp

  ldr reg, [usbphy, #dnDAddr]   // Get the direction address
  ldr val1, [reg]               // Get the direction value
  ldr val2, [usbphy, #dnMask]   // Get the mask value for ORing in
  orr val1, val1, val2          // Set the direciton mask
  ldr reg, [usbphy, #dnDAddr]   // Get the direction address
  str val1, [reg]               // Set the direction for Dp

  ldr r5, [usbphy, #dpMask]
  ldr r6, [usbphy, #dnMask]

usb_phy_write_top:

.rept 3
  nop                         // Padding to get to 1.5 MHz
.endr

  ldrb r7, [samples]          // Get the next sample to send

  mov r3, #3                  // Mask the sample so that it's in the
  and r7, r7, r3              // range of (0..3) so we don't jump out.

   // Each branch has an equal number of cycles and is an equal size.
   // Multiply the USB state (which is 0, 1, 2, or 3) by the size of
   // one cycle, to act as a jump table.
  mov r3, #(usb_phy_write_k - usb_phy_write_j)
  mul r7, r7, r3

  ldr r3, =usb_phy_write_se0  // Figure out the jump target, relative to the
  add r3, r7, r3              // start of the jump section.

  mov pc, r3                  // Jump into the table below.

  // Jump Table: Write SE0, K, J, or SE1 to the USB pins.
usb_phy_write_se0:
  ldr r3, [usbphy, #dpCAddr]
  ldr r4, [usbphy, #dnCAddr]
  b usb_phy_commit_values
usb_phy_write_j:
  ldr r3, [usbphy, #dpCAddr]
  ldr r4, [usbphy, #dnSAddr]
  b usb_phy_commit_values
usb_phy_write_k:
  ldr r3, [usbphy, #dpSAddr]
  ldr r4, [usbphy, #dnCAddr]
  b usb_phy_commit_values
usb_phy_write_se1:
  ldr r3, [usbphy, #dpSAddr]
  ldr r4, [usbphy, #dnSAddr]
  b usb_phy_commit_values

usb_phy_commit_values:
  str r5, [r3]                  // Write the computed states to the USB
  str r6, [r4]                  // pins, setting them or clearing as needed.

  add samples, samples, #1      // Move on to the next sample.
  sub count, count, #1          // Subtract 1 from the total sample number.
  cmp count, #0                 // See if there are any samples left to send.
  bne usb_phy_write_top         // If so, send another bit.

  // Now, set both lines to INPUT
  ldr reg, [usbphy, #dpDAddr]   // Get the direction address
  ldr val1, [reg]               // Get the direction value
  ldr val2, [usbphy, #dpMask]   // Get the mask value for ORing in
  bic val1, val1, val2          // Clear the direciton mask
  ldr reg, [usbphy, #dpDAddr]   // Get the direction address
  str val1, [reg]               // Set the direction for Dp

  ldr reg, [usbphy, #dnDAddr]   // Get the direction address
  ldr val1, [reg]               // Get the direction value
  ldr val2, [usbphy, #dnMask]   // Get the mask value for ORing in
  bic val1, val1, val2          // Clear the direciton mask
  ldr reg, [usbphy, #dnDAddr]   // Get the direction address
  str val1, [reg]               // Set the direction for Dp

  // Return the value
  mov count, samples
  pop {r4,r5,r6,r7}
  sub r0, count, samples        // Report how many bits we received
  bx lr
.endfunc
.type usbPhyWrite, %function
.size usbPhyWrite, .-usbPhyWrite
.global usbPhyWrite_size
.align 4
usbPhyWrite_size: .long usbPhyWrite, .-usbPhyWrite
.global usbPhyWrite_end
usbPhyWrite_end: nop

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
#endif


.func usbPhyWrite
.global usbPhyWrite
/*int */usbPhyWrite/*(const USBPHY *phy, uint8_t *samples, int max_samples)*/:
  bx lr
.type usbPhyWrite, %function
.size usbPhyWrite, .-usbPhyWrite
.endfunc

.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t *samples, int max_samples)*/:
  bx lr
.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead
.endfunc

.func usbPhyTime
.global usbPhyTime
/*int */usbPhyTime/*(volatile uint32_t *reg, uint32_t val)*/:
  str r1, [r0]
  str r1, [r0]
  str r1, [r0]
  str r1, [r0]
  str r1, [r0]
  str r1, [r0]
  bl wait_10_cycles

.rept 16
  str r1, [r0]
  bl wait_10_cycles
.endr
  str r1, [r0]
  bl wait_8_cycles
  b usbPhyTime /* 2 cycles */


wait_16_cycles: nop
wait_15_cycles: nop
wait_14_cycles: nop
wait_13_cycles: nop
wait_12_cycles: nop
wait_11_cycles: nop
wait_10_cycles: nop
wait_9_cycles:  nop
wait_8_cycles:  nop
wait_7_cycles:  nop
wait_6_cycles:  nop
wait_5_cycles:  mov pc, lr
wait_4_cycles:  nop
wait_3_cycles:  nop
wait_2_cycles:  nop
wait_1_cycles:  nop
.type usbPhyTime, %function
.size usbPhyTime, .-usbPhyTime
.endfunc
.global usbPhyTime_end
usbPhyTime_end: nop
.global usbPhyTime_size
.align 4
usbPhyTime_size: .long usbPhyTime, .-usbPhyTime
