.section .text    /* Can also run out of .section .ramtext */

usbphy  .req sp   /* Pointer to the USBPHY struct, described above */
outptr  .req r1   /* Outgoing sample buffer (pushed to stack) */

one     .req r1   /* The value 1 */
reg     .req r2   /* Register to sample pin */
mash    .req r3   /* Mask/shift to isolate required pin */
val     .req r4   /* Currently-sampled value */

sample1 .req r5   /* Most recent 32-bits */
sample2 .req r6   /* Next 32-bits */
sample3 .req r7   /* Remaining 24-bits */

lastval .req r8   /* What value was the last pin? */
counter .req r10
unstuff .req r11

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

.equ spSave,0x30

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
   */

/* Define this to have dpIAddr and dnIAddr point to uint32_t arrays rather
 * than GPIO banks.  This is used to test the code logic (but, obviously,
 * not the code timing.)
 */
//#define USB_PHY_READ_TEST

.func usbPhyRead
.global usbPhyRead
/*int */usbPhyRead/*(const USBPHY *phy, uint8_t samples[11])*/:
  push {r4, r5, r6, r7, lr}
  push {outptr, r2}

  // We need an extra lo register for the unstuff pattern #0b111111, but
  // we're all out.  As a complete and total hack, re-use the stack pointer
  // to index into the USBPHY struct.
  // This means that we can't push anything onto the stack, nor can we call
  // any functions that do.
  mov r2, sp
  str r2, [r0, #spSave]
  mov usbphy, r0

  /* Clear out the register shift-chain */
  mov sample1, #0
  mov sample2, #0
  mov sample3, #0
  mov counter, sample1
  mov r0, #0b11
  mov unstuff, r0                 // Load 0b11 into unstuff reg, as the header
                                  // ends with the pattern KK, which starts
                                  // a run of two.
  mov r0, #0b111111               // Unstuff mask
  mov one, #1                     // Actually load the value '1' into the reg.

  ldr reg, [usbphy, #dpIAddr]     // Grab the address for the data input reg.
  ldr mash, [usbphy, #dpMask]     // Grab the mask for the bit.

  /* Wait for the line to flip */
  ldr val, [reg]                  // Sample D+, to watch for it flipping
  and val, val, mash              // Mask off the interesting bit
  mov lastval, val                // Save the bit for use in looking for sync

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

  // The loop is 5 cycles on a failure.  One
  // pulse is 32 cycles.  Therefore, loop up
  // to 8 times before giving up.
#if !defined(USB_PHY_READ_TEST)
.rept 8
  ldr val, [reg]                  // Sample USBDP
  and val, val, mash              // Mask off the interesting bit
  cmp val, lastval                // Wait for it to change
  bne usb_phy_read_sync_wait      // When it changes, go wait for sync pulse
.endr
  b usb_phy_read_timeout          // It never changed, so return "timeout".
#endif

usb_phy_read_sync_wait:
#if !defined(USB_PHY_READ_TEST)
  // Wait until we're in the middle of a pulse.  When we get here, the pulse
  // will have happened between 6 and 10 cycles ago.  Since the middle of a
  // pulse occurs at 16 cycles, delay 8 cycles to line us up.
//  bl usb_read_wait_8_cycles
#endif /* !defined(USB_PHY_READ_TEST) */

  // Wait for the end-of-header sync pulse, which is when the value
  // repeats itself.  This is the "KK" in the KJKJKJKK training sequence.
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
  b usb_phy_sync_timeout

  /* We're synced to the middle of a pulse, and the clock sync / start-of-
   * -frame has been found.  Real packet data follows.
   */
start_reading_usb:

  /* Adjust lastval so that it's in the correct position -- we skip doing
     this above since we're only interested in the value changing, not in
     what the value is.  However, we're now interested in what the value
     is, so we now deal with shifts instead of masks, and always mask by
     #1.
   */
  mov val, lastval
  ldr mash, [usbphy, #dpShift]
  ror val, val, mash
  and val, val, one
  mov lastval, val

usb_phy_read_get_usb_bit:
  ldr val, [usbphy, #dpIAddr]     // Get the address of the D+ input bank
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  mov reg, val
  ldr val, [val]                  // Actually sample D+
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [usbphy, #dnIAddr]
  ldr mash, [reg]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]   // Move read test register up by 4
  mov reg, mash
#else
  ldr reg, [usbphy, #dnIAddr]    // Get the address of the D- input bank
  ldr val, [val]                  // Actually sample D+
  ldr reg, [reg]                 // Also sample D-
#endif /* defined(USB_PHY_READ_TEST) */
  ldr mash, [usbphy, #dpShift]    // Get the mask of the D+ bit
  ror val, val, mash              // Rotate the value down to bit position 1.
  and val, val, one               // Mask off everything else.
  // 7 [5]

  // Check for SE0
#if 1
  ldr mash, [usbphy, #dnMask]
  and reg, reg, mash
  add reg, reg, val               // An end-of-frame is indicated by two
                                  // frames of SE0.  If this is the case,
                                  // then the result of adding these together
                                  // will result in 0.
  beq usb_phy_read_exit           // Exit if so.
#else
  mov mash, counter
  cmp mash, #88
  beq usb_phy_read_exit
#endif
  // 4 [3]

  mov mash, lastval
  mov lastval, val
  eor val, mash                   // Check to see if the state has changed.
  mvn val, val                    // Invert, as 1 ^ 1 or 0 ^ 0 should be 1.
  and val, val, one               // Mask off everything but the last bit.

  add unstuff, unstuff, unstuff   // Shift the "unstuff" value up by 1.
  add unstuff, unstuff, val       // "or" in the one-bit "val" to the bottom.

  lsr val, val, one               // Shift the state into the carry bit
  adc sample1, sample1, sample1   // Propagate the carry bit up
  adc sample2, sample2, sample2   // Propagate the carry bit up
  adc sample3, sample3, sample3   // Propagate the carry bit up
  // 11 [9]

  /* Restore values, increase counter */
  add counter, counter, one
  // 1

  // Figure out if we need to unstuff, or if we can just continue to the
  // next bit.
  // Six consecutive USB states will be followed by a dummy
  // state flip.  Ignore this.
  mov reg, unstuff
  and reg, reg, r0
  cmp reg, r0
  bne usb_phy_read_get_usb_bit
  // 4 [0]

usb_unstuff:
  nop                               // We get here when the current bit has one
                                    // more clock cycle left.  Add a nop just
                                    // to make the cycle-counting easier.
  /* --- New pulse starts here --- */
  // NOTE that we don't increment "counter" here.
  mov unstuff, one                  // We're skipping over one bit, which
                                    // results in a new run of one.
  // 1

#if defined(USB_PHY_READ_TEST)
  ldr reg, [usbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dnIAddr]       // Move read test register up by 4
  ldr reg, [usbphy, #dpIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [usbphy, #dpIAddr]       // Move read test register up by 4
#endif /* defined(USB_PHY_READ_TEST) */


  /* Invert the last value, since a false 0 was added to the stream */
  mov reg, lastval                  // Read the current last val into a lo reg.
  mvn reg, reg                      // Negate the value.
  and reg, reg, one                 // Mask it with 0b1
  mov lastval, reg                  // Save it back into a lo reg.
  // 4

#if !defined(USB_PHY_READ_TEST)
  bl usb_read_wait_23_cycles
#endif /* defined(USB_PHY_READ_TEST) */
  b usb_phy_read_get_usb_bit
  // 2

usb_phy_read_exit:
  ldr r2, [usbphy, #spSave]
  mov sp, r2
  pop {outptr, r2}

  /* Bits come across in the wrong endianess.  Fix that. */
  rev sample1, sample1
  rev sample2, sample2
  rev sample3, sample3
  str sample1, [r2, #8]
  str sample2, [r2, #4]
  str sample3, [r2, #0]

  retval .req r0
  dstptr .req r1
  srcptr .req r2
  dstoff .req r3
  srcoff .req r4
  endoff .req r5
  tmpval .req r6

  /* Count the number of bytes read (counter / 8) */
  mov retval, counter
  mov r3, #3
  asr retval, retval, r3              // Number of bytes read (return value)
  cmp retval, #11
  bgt usb_phy_read_overflow_exit

  cmp retval, #0
  beq usb_phy_read_no_data_exit

  mov srcoff, retval
  mov endoff, #12
#if 1 // XXX testing
  sub srcoff, endoff, srcoff
#else
  mov srcoff, #1
  mov retval, #11
#endif
  mov dstoff, #0

copy_loop_top:
  ldrb tmpval, [srcptr, srcoff]
  add srcoff, srcoff, #1

  strb tmpval, [dstptr, dstoff]
  add dstoff, dstoff, #1

  cmp srcoff, endoff
  bne copy_loop_top
final_exit:
  pop {r4,r5,r6,r7,pc}

  /* Read too many bits, return -2 */
usb_phy_read_overflow_exit:
  mov r1, #2
  mov r0, #0
  sub r0, r0, r1
  pop {r4,r5,r6,r7,pc}

usb_phy_read_no_data_exit:
  mov r0, #0
  pop {r4,r5,r6,r7,pc}

usb_phy_sync_timeout:
  ldr r2, [usbphy, #spSave]
  mov sp, r2
  pop {outptr, r2}
  mov r0, #0
  sub r0, r0, #3
  pop {r4,r5,r6,r7,pc}

usb_phy_read_timeout:
  ldr r2, [usbphy, #spSave]
  mov sp, r2
  pop {outptr, r2}
  mov r0, #0
  sub r0, r0, #1
  pop {r4,r5,r6,r7,pc}

usb_read_wait_30_cycles: nop
usb_read_wait_29_cycles: nop
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

.endfunc
.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead

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


.func usbPhyWriteTestPattern
.global usbPhyWriteTestPattern
usbPhyWriteTestPattern:
  push {lr}

  /* Set D+ to output */
  ldr reg, [usbphy, #dpDAddr]     // Get the direction address
  ldr val, [reg]                  // Get the direction value
  ldr mash, [usbphy, #dpMask]     // Get the mask value for ORing in
  orr val, val, mash              // Set the direciton mask
  str val, [reg]                  // Set the direction for Dp

  /* Set D- to output */
  ldr reg, [usbphy, #dnDAddr]     // Get the direction address
  ldr val, [reg]                  // Get the direction value
  ldr mash, [usbphy, #dnMask]     // Get the mask value for ORing in
  orr val, val, mash              // Set the direciton mask
  ldr reg, [usbphy, #dnDAddr]     // Get the direction address
  str val, [reg]                  // Set the direction for Dp

  ldr r1, [r0, #dpSAddr]
  ldr r2, [r0, #dpCAddr]
  ldr r3, [r0, #dpMask]

  ldr r4, [r0, #dnSAddr]
  ldr r5, [r0, #dnCAddr]
  ldr r6, [r0, #dnMask]

  // usb start-of-frame header //
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_write_state_k
  // end of header //

  // 0x00
  // 0x00
  // 0x00
  // 0x00
  // 0xd5
  // 0x6d
  // 0xdd
  // 0x80
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0

  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0

  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0

  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0

  bl usb_write_state_k // 1
  bl usb_write_state_k // 1
  bl usb_write_state_j // 0
  bl usb_write_state_j // 1
  bl usb_write_state_k // 0
  bl usb_write_state_k // 1
  bl usb_write_state_j // 0
  bl usb_write_state_j // 1

  bl usb_write_state_k // 0
  bl usb_write_state_k // 1
  bl usb_write_state_k // 1
  bl usb_write_state_j // 0
  bl usb_write_state_j // 1
  bl usb_write_state_j // 1
  bl usb_write_state_k // 0
  bl usb_write_state_k // 1

  bl usb_write_state_k // 1
  bl usb_write_state_k // 1
  bl usb_write_state_j // 0
  bl usb_write_state_j // 1
  bl usb_write_state_j // 1
  bl usb_write_state_j // 1
  bl usb_write_state_k // 0
  bl usb_write_state_k // 1

  bl usb_write_state_k // 1
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0
  bl usb_write_state_k // 0
  bl usb_write_state_j // 0

  // start of end-of-frame
  bl usb_write_state_se0
  bl usb_write_state_se0
  // end of end-of-frame

  pop {pc}

  /* These all specify one extra cycle.  If you add up the numbers, you'll
     find we should sleep for 26 cycles.  However, those names assume you
     get there via a "bl", which adds an extra cycle over "b" (3 cycles
     instead of 2.)  Therefore, branch to usb_read_wait_27_cycles instead
     of usb_read_wait_26_cycles to compensate.
   */
usb_write_state_j:
  str r3, [r1]    // D+ set
  str r6, [r5]    // D- clr
  b usb_read_wait_27_cycles

usb_write_state_k:
  str r3, [r2]    // D+ clr
  str r6, [r4]    // D- set
  b usb_read_wait_27_cycles

usb_write_state_se0:
  str r3, [r2]    // D+ clr
  str r6, [r5]    // D- clr
  b usb_read_wait_27_cycles

.endfunc
.type usbPhyWriteTestPattern, %function
.size usbPhyWriteTestPattern, .-usbPhyWriteTestPattern

.func asmtest
.global asmtest
asmtest:
  /*int asmtest(int a1, int a2)*/

  add r1, r1, r0
  beq asmtest_equal
  mov r0, #0
  bx lr

asmtest_equal:
  mov r0, #1
  bx lr

.endfunc
.type asmtest, %function
.size asmtest, .-asmtest
