.section .ramtext    /* Can also run out of .section .ramtext */
//.section .text    /* Can also run out of .section .ramtext */

rusbphy  .req sp   /* Pointer to the USBPHY struct, described above */
outptr  .req r1   /* Outgoing sample buffer (pushed to stack) */

one     .req r1   /* The value 1 */
reg     .req r2   /* Register to sample pin */
mash    .req r3   /* Mask/shift to isolate required pin */
val     .req r4   /* Currently-sampled value */

sample1 .req r5   /* Most recent 32-bits */
sample2 .req r6   /* Next 32-bits */
sample3 .req r7   /* Remaining 24-bits */

lastval .req r8   /* What value was the last pin? */
counter .req r9
unstuff .req r10

#if 0
  /***************************************************************************
   * USB PHY low-level code
   * 
   * Exports the following functions:
   *
   *    int usbPhyRead(USBPhy *phy, uint8_t buffer[11], uint32_t scratch[3])
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
   *    uint32_t dpShift; /* Shift of GPIO pin in S/C/D/I addresses */
   *
   *    /* USB D+ line descriptor, as above */
   *    uint32_t dnIAddr;
   *    uint32_t dnSAddr;
   *    uint32_t dnCAddr;
   *    uint32_t dnDAddr;
   *    uint32_t dnShift;
   *
   *    /* USB masks */
   *    uint32_t dpMask;  /* Mask of GPIO pin in S/C/D/I addresses */
   *    uint32_t dnMask;
   *
   *    /* Store the stack pointer here during running */
   *    uint32_t sp_temp;
   * };
   */
#endif

/* [r|w]usbphy offsets */
.equ dpIAddr,0x20
.equ dpSAddr,0x24
.equ dpCAddr,0x28
.equ dpDAddr,0x2c
.equ dpShift,0x30

.equ dnIAddr,0x34
.equ dnSAddr,0x38
.equ dnCAddr,0x3c
.equ dnDAddr,0x40
.equ dnShift,0x44

.equ dpMask,0x48
.equ dnMask,0x4c

.equ spSave,0x50

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
  mov rusbphy, r0

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

  ldr reg, [rusbphy, #dpIAddr]     // Grab the address for the data input reg.
  ldr mash, [rusbphy, #dpMask]     // Grab the mask for the bit.

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
  str reg, [rusbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [rusbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dnIAddr]     // Move read test register up by 4
  ldr reg, [rusbphy, #dpIAddr]
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
  str reg, [rusbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [rusbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dnIAddr]     // Move read test register up by 4
  ldr reg, [rusbphy, #dpIAddr]
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
  ldr mash, [rusbphy, #dpShift]
  ror val, val, mash
  and val, val, one
  mov lastval, val

usb_phy_read_get_usb_bit:
  ldr val, [rusbphy, #dpIAddr]     // Get the address of the D+ input bank
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  mov reg, val
  ldr val, [val]                  // Actually sample D+
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dpIAddr]     // Move read test register up by 4
  ldr reg, [rusbphy, #dnIAddr]
  ldr mash, [reg]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dnIAddr]   // Move read test register up by 4
  mov reg, mash
#else
  ldr reg, [rusbphy, #dnIAddr]    // Get the address of the D- input bank
  ldr val, [val]                  // Actually sample D+
  ldr reg, [reg]                 // Also sample D-
#endif /* defined(USB_PHY_READ_TEST) */
  ldr mash, [rusbphy, #dpShift]    // Get the mask of the D+ bit
  ror val, val, mash              // Rotate the value down to bit position 1.
  and val, val, one               // Mask off everything else.
  // 7 [5]

  // Check for SE0
#if 1
  ldr mash, [rusbphy, #dnMask]
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
  ldr reg, [rusbphy, #dnIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dnIAddr]       // Move read test register up by 4
  ldr reg, [rusbphy, #dpIAddr]
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  add reg, reg, one
  str reg, [rusbphy, #dpIAddr]       // Move read test register up by 4
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
  ldr r2, [rusbphy, #spSave]
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
  ldr r2, [rusbphy, #spSave]
  mov sp, r2
  pop {outptr, r2}
  mov r0, #0
  sub r0, r0, #3
  pop {r4,r5,r6,r7,pc}

usb_phy_read_timeout:
  ldr r2, [rusbphy, #spSave]
  mov sp, r2
  pop {outptr, r2}
  mov r0, #0
  sub r0, r0, #1
  pop {r4,r5,r6,r7,pc}

usb_read_wait_32_cycles: nop
usb_read_wait_31_cycles: nop
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



/*
 * Register (arguments):
 *   r0: USBPHY
 *   r1: sample buffer
 *   r2: number of samples to write
 */
wusbphy   .req r0   /* Pointer to USBPHY value */

wlastsym  .req r1   /* The last symbol (0 = j, 1 = k) */
wpkt      .req r2   /* Current packet */
wleft     .req r3   /* Number of bits left before we need to reload wpkt */

/* These are used when writing values out.  May be repurposed. */
wpaddr    .req r4   /* Write "address" for D+ line, during normal operation */
wnaddr    .req r5   /* Write "address" for D- line, during normal operation */
wtmp1     .req r4
wtmp2     .req r5

/* These must remain unchanged during the whole operation. */
wpmask    .req r6   /* Write mask for D+ line */
wnmask    .req r7   /* Write mask for D- line */

wstuff    .req r8   /* The last six bits, used for bit stuffing */
wpktnum   .req r9

/* Indexes off of the stack pointer */
.equ wBytesLeft,0x04
.equ wPkt1,0x08
.equ wPkt1Num,0x0c
.equ wPkt2,0x10
.equ wPkt2Num,0x14
.equ wPkt3,0x18
.equ wPkt3Num,0x1c
.equ wSpSize,0x20

.func usbPhyWrite
.global usbPhyWrite
/*int */usbPhyWrite/*(const USBPHY *phy, uint8_t *samples, int count)*/:
  push {r3,r4,r5,r6,r7,lr}

  /* TODO: Convert from uint8_t *samples to register values */
  cmp r2, #8
  ble usb_phy_write_less_than_8_bytes

  mov wtmp1, #32
  str wtmp1, [wusbphy, #wPkt3Num]
  str wtmp1, [wusbphy, #wPkt2Num]

  mov wtmp1, r2
  sub wtmp1, #8
  lsl wtmp1, #3
  str wtmp1, [wusbphy, #wPkt1Num]

  mov wtmp1, #3
  mov wpktnum, wtmp1
  b usb_phy_write_done_with_byte_check

usb_phy_write_less_than_8_bytes:
  cmp r2, #4
  ble usb_phy_write_less_than_4_bytes

  mov wtmp1, #0
  str wtmp1, [wusbphy, #wPkt3Num]

  mov wtmp1, #32
  str wtmp1, [wusbphy, #wPkt2Num]

  mov wtmp1, r2
  sub wtmp1, #4
  lsl wtmp1, #3
  str wtmp1, [wusbphy, #wPkt1Num]

  mov wtmp1, #2
  mov wpktnum, wtmp1
  b usb_phy_write_done_with_byte_check

usb_phy_write_less_than_4_bytes:

  mov wtmp1, #0
  str wtmp1, [wusbphy, #wPkt3Num]
  str wtmp1, [wusbphy, #wPkt2Num]

  mov wtmp1, r2
  lsl wtmp1, #3
  str wtmp1, [wusbphy, #wPkt1Num]

  mov wtmp1, #1
  mov wpktnum, wtmp1
  
usb_phy_write_done_with_byte_check:

  // Load the bytes into the pkt caches
  mov wtmp2, wusbphy
  add wtmp2, #wPkt1
  ldrb wtmp1, [r1, #0]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #1]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #2]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #3]
  strb wtmp1, [wtmp2]

  mov wtmp2, wusbphy
  add wtmp2, #wPkt2
  ldrb wtmp1, [r1, #4]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #5]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #6]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #7]
  strb wtmp1, [wtmp2]

  mov wtmp2, wusbphy
  add wtmp2, #wPkt3
  ldrb wtmp1, [r1, #8]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #9]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #10]
  strb wtmp1, [wtmp2]
  add wtmp2, #1
  ldrb wtmp1, [r1, #11]
  strb wtmp1, [wtmp2]

  ldr wtmp1, [wusbphy, #wPkt1]
  mvn wtmp1, wtmp1                // Invert, because it makes math work
  rev wtmp1, wtmp1
  str wtmp1, [wusbphy, #wPkt1]
  ldr wtmp1, [r1, #4]
  mvn wtmp1, wtmp1                // Invert, because it makes math work
  rev wtmp1, wtmp1
  str wtmp1, [wusbphy, #wPkt2]
  ldr wtmp1, [r1, #8]
  mvn wtmp1, wtmp1                // Invert, because it makes math work
  rev wtmp1, wtmp1
  str wtmp1, [wusbphy, #wPkt3]

usb_phy_write_get_first_packet:
  // Read the first packet
  mov wtmp1, wpktnum
  lsl wtmp1, #3
  add wtmp1, wusbphy
  ldr wpkt, [wtmp1]
  add wtmp1, #4
  ldr wleft, [wtmp1]

  // First, set both lines to OUTPUT
  ldr wtmp1, [wusbphy, #dpDAddr]  // Get the direction address
  ldr wtmp2, [wtmp1]              // Get the direction value
  ldr wpmask, [wusbphy, #dpMask]  // Get the mask value for ORing in
  orr wtmp2, wtmp2, wpmask        // Set the direciton mask
  ldr wtmp1, [wusbphy, #dpDAddr]  // Get the direction address
  str wtmp2, [wtmp1]              // Set the direction for Dp

  ldr wtmp1, [wusbphy, #dnDAddr]  // Get the direction address
  ldr wtmp2, [wtmp1]              // Get the direction value
  ldr wnmask, [wusbphy, #dnMask]  // Get the mask value for ORing in
  orr wtmp2, wtmp2, wnmask        // Set the direciton mask
  ldr wtmp1, [wusbphy, #dnDAddr]  // Get the direction address
  str wtmp2, [wtmp1]              // Set the direction for Dp

  mov wlastsym, #1                // Last symbols were "KK" from the header
  mov wtmp1, #0b111100            // So load a run of 2 into the stuff value.
  mov wstuff, wtmp1               //

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

usb_phy_write_top:

  mov wtmp1, #0                     // Clear wthisbit, so we can add later.
  add wpkt, wpkt, wpkt              // Shift the top bit into the carry bit.
  adc wtmp1, wtmp1                  // Pull the new bit out from the carry bit.

  add wstuff, wstuff, wstuff        // Shift up the stuff bit, to allow for
  add wstuff, wstuff, wtmp1         // adding the new bit in.

  add wlastsym, wlastsym, wtmp1     // Add the new bit to the last symbol

  mov wtmp1, #0b1
  tst wlastsym, wtmp1
  // 8

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write_j
usb_phy_write_k:
  ldr wpaddr, [wusbphy, #dpSAddr]    // D+ set
  ldr wnaddr, [wusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_out
  
usb_phy_write_j:
  ldr wpaddr, [wusbphy, #dpCAddr]    // D+ clr
  ldr wnaddr, [wusbphy, #dnSAddr]    // D- set
  nop

usb_phy_write_out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  // 7 (either branch taken)

  // 15 cycles total

  mov wtmp1, #0b111111
  mov wtmp2, wstuff
  and wtmp2, wtmp1
//  cmp wtmp2, wtmp1
  beq usb_phy_write_stuff_bit
  // 4

usb_phy_write_done_stuffing_bit:
  sub wleft, wleft, #1
  bne usb_phy_write_continue_word
  // 2

usb_phy_write_finished_word:
  mov wtmp1, wpktnum
  sub wtmp1, #1
  mov wpktnum, wtmp1
  beq usb_write_eof

usb_phy_write_calculate_next_pkt:
  lsl wtmp1, #3
  ldr wpkt, [wusbphy, wtmp1]
  add wtmp1, #4
  ldr wleft, [wusbphy, wtmp1]
  // 8

  b usb_phy_write_top

usb_phy_write_continue_word:
  .rept 6
  nop
  .endr

  b usb_phy_write_top
  // 2


usb_phy_write_stuff_bit:
// 5 vs 8 cycles
// The real path is 3 cycles longer
// There are 2 + 2 + 7 skipped cycles here, too.
  bl usb_read_wait_13_cycles
  mov wtmp1, #0b111110              // Clear out the bit-stuff counter
  mov wstuff, wtmp1
  // 2

  add wlastsym, wlastsym, #1        // Invert the last symbol.

  mov wtmp1, #0b1                   // See if we need to send j or k
  tst wlastsym, wtmp1
  // 3

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write_stuff_j
usb_phy_write_stuff_k:
  ldr wpaddr, [wusbphy, #dpSAddr]    // D+ set
  ldr wnaddr, [wusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_stuff_out
  
usb_phy_write_stuff_j:
  ldr wpaddr, [wusbphy, #dpCAddr]    // D+ clr
  ldr wnaddr, [wusbphy, #dnSAddr]    // D- set
  nop

usb_phy_write_stuff_out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  // 7 (either branch taken)

  .rept 2
  nop
  .endr
  b usb_phy_write_done_stuffing_bit



usb_write_eof:
  bl usb_write_state_se0
  bl usb_write_state_se0

  // --- Done Transmitting --- //

  // Now, set both lines to INPUT
  ldr wtmp1, [wusbphy, #dpDAddr]   // Get the direction address
  ldr wtmp2, [wtmp1]               // Get the direction value
  ldr wpmask, [wusbphy, #dpMask]   // Get the mask value for ORing in
  bic wtmp2, wtmp2, wpmask          // Clear the direciton mask
  ldr wtmp1, [wusbphy, #dpDAddr]   // Get the direction address
  str wtmp2, [wtmp1]               // Set the direction for Dp

  ldr wtmp1, [wusbphy, #dnDAddr]   // Get the direction address
  ldr wtmp2, [wtmp1]               // Get the direction value
  ldr wnmask, [wusbphy, #dnMask]   // Get the mask value for ORing in
  bic wtmp2, wtmp2, wnmask          // Clear the direciton mask
  ldr wtmp1, [wusbphy, #dnDAddr]   // Get the direction address
  str wtmp2, [wtmp1]               // Set the direction for Dp

  pop {r3,r4,r5,r6,r7,pc}


  // Useful functions
usb_write_state_se0:
  ldr wpaddr, [wusbphy, #dpCAddr]    // D+ clr
  ldr wnaddr, [wusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_out_func
usb_write_state_j:
  ldr wpaddr, [wusbphy, #dpSAddr]    // D+ set
  ldr wnaddr, [wusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_out_func
usb_write_state_k:
  ldr wpaddr, [wusbphy, #dpCAddr]    // D+ clr
  ldr wnaddr, [wusbphy, #dnSAddr]    // D- set
  nop
usb_phy_write_out_func:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  b usb_read_wait_23_cycles

.endfunc
.type usbPhyWrite, %function
.size usbPhyWrite, .-usbPhyWrite


tusbphy .req r0

tpaddr  .req r1   /* Write "set-" or "clear-bit" address for D+ line */
tpmask  .req r2   /* Write mask for D+ line */

tnaddr  .req r3   /* Write "set-" or "clear-bit" address for D- line */
tnmask  .req r4   /* Write mask for D- line */

.func usbPhyWriteTestPattern
.global usbPhyWriteTestPattern
usbPhyWriteTestPattern:
  push {lr}

  /* Set D+ to output */
  ldr reg, [tusbphy, #dpDAddr]     // Get the direction address
  ldr val, [reg]                  // Get the direction value
  ldr tpmask, [tusbphy, #dpMask]     // Get the mask value for ORing in
  orr val, val, tpmask              // Set the direciton mask
  str val, [reg]                  // Set the direction for Dp

  /* Set D- to output */
  ldr reg, [tusbphy, #dnDAddr]     // Get the direction address
  ldr val, [reg]                  // Get the direction value
  ldr tnmask, [tusbphy, #dnMask]     // Get the mask value for ORing in
  orr val, val, tnmask              // Set the direciton mask
  ldr reg, [tusbphy, #dnDAddr]     // Get the direction address
  str val, [reg]                  // Set the direction for Dp

  ldr tpmask, [r0, #dpMask]
  ldr tnmask, [r0, #dnMask]

  // usb start-of-frame header //
  bl usb_write_test_state_k
  bl usb_write_test_state_j
  bl usb_write_test_state_k
  bl usb_write_test_state_j
  bl usb_write_test_state_k
  bl usb_write_test_state_j
  bl usb_write_test_state_k
  bl usb_write_test_state_k
  // end of header //

  bl usb_write_test_state_k // 1
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_k // 1

  bl usb_write_test_state_k // 1
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0

  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_j // 0

  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0

  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_k // 1

  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0

  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0

  bl usb_write_test_state_j // 0
  bl usb_write_test_state_j // 1
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0

  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0

  bl usb_write_test_state_j // 1
  bl usb_write_test_state_j // 1
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_j // 1

  bl usb_write_test_state_j // 1
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_j // 1
  bl usb_write_test_state_k // 0
  bl usb_write_test_state_k // 1
  bl usb_write_test_state_j // 0
  bl usb_write_test_state_k // 0

  // start of end-of-frame
  bl usb_write_test_state_se0
  bl usb_write_test_state_se0
  // end of end-of-frame

  pop {pc}

  /* These all specify one extra cycle.  If you add up the numbers, you'll
     find we should sleep for 26 cycles.  However, those names assume you
     get there via a "bl", which adds an extra cycle over "b" (3 cycles
     instead of 2.)  Therefore, branch to usb_read_wait_27_cycles instead
     of usb_read_wait_26_cycles to compensate.
   */
usb_write_test_state_se0:
  ldr tpaddr, [tusbphy, #dpCAddr]    // D+ clr
  ldr tnaddr, [tusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_test_out
usb_write_test_state_j:
  ldr tpaddr, [tusbphy, #dpSAddr]    // D+ set
  ldr tnaddr, [tusbphy, #dnCAddr]    // D- clr
  b usb_phy_write_test_out
usb_write_test_state_k:
  ldr tpaddr, [tusbphy, #dpCAddr]    // D+ clr
  ldr tnaddr, [tusbphy, #dnSAddr]    // D- set
  nop
usb_phy_write_test_out:
  str tpmask, [wpaddr]
  str tnmask, [wnaddr]
  b usb_read_wait_23_cycles

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
