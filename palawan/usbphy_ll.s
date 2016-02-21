.text

usbphy  .req r0   /* Pointer to the USBPHY struct, described above */
outptr  .req r1   /* Outgoing sample buffer (pushed to stack) */

one     .req r1   /* The value 1 */
reg     .req r2   /* Register to sample pin */
mash    .req r3   /* Mask/shift to isolate required pin */
val     .req r4   /* Currently-sampled value */

sample1 .req r5   /* Most recent 32-bits */
sample2 .req r6   /* Next 32-bits */
sample3 .req r7   /* Remaining 24-bits */

lastval .req r8   /* What value was the last pin? */
usbphybkp .req r9
counter .req r10
unstuff .req r11
tmp     .req r0

#if 0
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
#endif

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
wait_6_cycles:  mov pc, lr
/* Less than 6 cycles, call nop. */

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

/* Define this to have dpIAddr and dnIAddr point to uint32_t arrays rather
 * than GPIO banks.  This is used to test the code logic (but, obviously,
 * not the code timing.)
 */
#define USB_PHY_READ_TEST

.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t samples[11])*/:
  push {r4, r5, r6, r7, lr}
  push {outptr, r2}

  /* Clear out the register shift-chain */
  mov sample1, #0
  mov sample2, #0
  mov sample3, #0
  mov usbphybkp, usbphy
  mov counter, sample1
  mov one, #3
  mov unstuff, one                // Load 1 into unstuff reg, as the header
                                  // ends with the pattern KK, which starts
                                  // a run of two.
  mov one, #1

  ldr reg, [usbphy, #dpIAddr]     // load the gpio register into r3
  ldr mash, [usbphy, #dpMask]     // Mask to get USB line bit from register

  /* Wait for the line to flip */
  ldr val, [reg]              // Sample USBDP
  and val, val, mash      // Mask off the interesting bit
  mov lastval, val

#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [usbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]     // Move read test register up by 4
  ldr reg, [usbphy, #dpIAddr]
#endif /* defined(USB_PHY_READ_TEST) */

  // The loop is 4 cycles on a failure.  One
  // pulse is 32 cycles.  Therefore, loop up
  // to 9 times before giving up.
#if !defined(USB_PHY_READ_TEST)
.rept 11
  ldr val, [reg]                  // Sample USBDP
  and val, val, mash              // Mask off the interesting bit
  cmp val, lastval                // Wait for it to change
  bne usb_phy_read_sync_wait
.endr
  b usb_phy_read_timeout
#endif

usb_phy_read_sync_wait:
  // There should be about 16 instructions between "ldr" above and "ldr" below.
  // Minus up to 8 instructions for the "beq" path
#if !defined(USB_PHY_READ_TEST)
  bl usb_read_wait_13_cycles
#endif /* !defined(USB_PHY_READ_TEST) */

.rept 7
  ldr val, [reg]                  // Sample USBDP
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [usbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]     // Move read test register up by 4
  ldr reg, [usbphy, #dpIAddr]
#endif /* defined(USB_PHY_READ_TEST) */
  and val, val, mash              // Mask off the interesting bit
  cmp lastval, val
  beq start_reading_usb
  mov lastval, val
#if !defined(USB_PHY_READ_TEST)
  bl usb_read_wait_27_cycles
#endif /* defined(USB_PHY_READ_TEST) */
.endr
  b usb_phy_read_timeout

  /* We're synced to the middle of a pulse, and the clock sync / start-of-
   * -frame has been found.  Real packet data follows.
   */
start_reading_usb:

get_usb_bit:
  ldr reg, [usbphy, #dpIAddr]     // load the gpio register into r3
  ldr mash, [usbphy, #dpShift]    // load the gpio register into r3
  ldr val, [reg]                  // Sample USBDP
  mov one, #1
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]     // Move read test register up by 4
#endif /* defined(USB_PHY_READ_TEST) */
  ror val, val, mash              // Get it down to one bit
  mov one, #1
  and val, val, one               // And mask off everything else

  // Check for SE0
  ldr reg, [usbphy, #dnIAddr]
#if defined(USB_PHY_READ_TEST)
  ldr mash, [reg]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]   // Move read test register up by 4
  mov reg, mash
#else
  ldr reg, [reg]
#endif /* defined(USB_PHY_READ_TEST) */
  ldr mash, [usbphy, #dnShift]
  ror reg, reg, mash
  and reg, reg, one
  add reg, reg, val  // An end-of-frame is indicated by two
                                // frames of SE0.  If this is the case,
                                // then the result of adding these together
                                // will result in 0.
  cmp reg, #0
  beq exit                      // Exit if so.
  ldr mash, [usbphy, #dpShift]
  ldr reg, [usbphy, #dnIAddr]
  // ??

  mov tmp, lastval
  mov lastval, val
  eor val, tmp                   // Check to see if the state has flipped
  mvn val, val                    // Invert, as the bits come across flipped
  and val, val, one
  mov mash, unstuff
  lsl mash, mash, one
  orr mash, mash, val
  mov unstuff, mash
  lsr val, val, one              // Shift the state into the carry bit
  adc sample1, sample1, sample1  // Propagate the carry bit up
  adc sample2, sample2, sample2  // Propagate the carry bit up
  adc sample3, sample3, sample3  // Propagate the carry bit up
  // 10

  // Unstuff bits.  Six consecutive USB states will be followed by a dummy
  // state flip.  Ignore this.
  mov tmp, #0b111111              // Check to see if the next bit will be dummy
  mov mash, unstuff
  and mash, mash, tmp
  cmp tmp, mash
  beq usb_unstuff

  /* Restore values, increase counter */
  mov one, #1                     // Restore "one"
  mov usbphy, usbphybkp           // Restore "usbphy"
  add counter, counter, one


#if !defined(USB_PHY_READ_TEST)
  bl usb_read_wait_6_cycles
#endif /* defined(USB_PHY_READ_TEST) */
  b get_usb_bit
  // 2

usb_unstuff:
  mov one, #1                     // Restore "one"
  mov unstuff, one
  mov usbphy, usbphybkp           // Restore "usbphy"
  add counter, counter, one
// Just invert the value and ignore the next bit
#if defined(USB_PHY_READ_TEST)
  ldr reg, [usbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]   // Move read test register up by 4
  ldr reg, [usbphy, #dpIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]   // Move read test register up by 4
#endif /* defined(USB_PHY_READ_TEST) */
  mov reg, lastval
  mvn reg, reg
  and reg, reg, one
  mov lastval, reg
  // 18
#if !defined(USB_PHY_READ_TEST)
  bl usb_read_wait_12_cycles
#endif /* defined(USB_PHY_READ_TEST) */
  b get_usb_bit

exit:

  mov one, #1
.rept 8
  lsr val, val, one              // Shift the state into the carry bit
  adc sample1, sample1, sample1  // Propagate the carry bit up
  adc sample2, sample2, sample2  // Propagate the carry bit up
  adc sample3, sample3, sample3  // Propagate the carry bit up
.endr
  pop {outptr, r2}

  rev sample1, sample1
  rev sample2, sample2
  rev sample3, sample3
  str sample1, [r2, #8]
  str sample2, [r2, #4]
  str sample3, [r2, #0]

  /* Commit the number of bits read */
  mov r0, counter
  mov r3, #3
  asr r0, r0, r3

  mov r3, r0
  mov r4, #0
copy_loop_top:
  ldrb r5, [r2, r4]
  sub r3, r3, #1

  strb r5, [r1, r4]
  add r4, r4, #1

  cmp r4, r0
  bne copy_loop_top
final_exit:
  pop {r4,r5,r6,r7,pc}

usb_phy_read_timeout:
  pop {outptr, r2}
  mov r0, #0
  sub r0, r0, #1
  pop {r4,r5,r6,r7,pc}
.endfunc
.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead

usb_read_wait_28_cycles: nop
usb_read_wait_27_cycles: nop
usb_read_wait_26_cycles: nop
usb_read_wait_25_cycles: nop
usb_read_wait_24_cycles: nop
usb_read_wait_23_cycles: nop
usb_read_wait_22_cycles: nop
usb_read_wait_21_cycles: nop
usb_read_wait_20_cycles: nop
usb_read_wait_19_cycles: nop
usb_read_wait_18_cycles: nop
usb_read_wait_17_cycles: nop
usb_read_wait_16_cycles: nop
usb_read_wait_15_cycles: nop
usb_read_wait_14_cycles: nop
usb_read_wait_13_cycles: nop
usb_read_wait_12_cycles: nop
usb_read_wait_11_cycles: nop
usb_read_wait_10_cycles: nop
usb_read_wait_9_cycles:  nop
usb_read_wait_8_cycles:  nop
usb_read_wait_7_cycles:  nop
usb_read_wait_6_cycles:  nop
usb_read_wait_5_cycles:  mov pc, lr

.global usbPhyRead_size
.align 4
usbPhyRead_size: .long usbPhyRead, .-usbPhyRead



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
#if 0
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
#endif
  bx lr
.endfunc
.type usbPhyWrite, %function
.size usbPhyWrite, .-usbPhyWrite
.global usbPhyWrite_size
.align 4
usbPhyWrite_size: .long usbPhyWrite, .-usbPhyWrite
.end

