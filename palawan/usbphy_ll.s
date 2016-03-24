.section .ramtext    /* Can also run out of .section .ramtext */
//.section .text    /* Can also run out of .section .ramtext */

#if 0
  /***************************************************************************
   * USB PHY low-level code
   * 
   * Exports the following functions:
   *
   *    int usbPhyReadI(USBPhy *phy, uint8_t buffer[11], uint32_t scratch[3])
   *    void usbPhyWriteI(USBPhy *phy, uint8_t buffer[11], uint32_t count)
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
.equ dpIAddr,0x08
.equ dpSAddr,0x0c
.equ dpCAddr,0x10
.equ dpDAddr,0x14
.equ dpShift,0x18

.equ dnIAddr,0x1c
.equ dnSAddr,0x20
.equ dnCAddr,0x24
.equ dnDAddr,0x28
.equ dnShift,0x2c

.equ dpMask,0x30
.equ dnMask,0x34

.equ spSave,0x38

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

rusbphy  .req sp  /* Pointer to the USBPHY struct, described above */
rusmask  .req r0  /* Unstuff Mask (constant 0b111111) */
routptr  .req r1  /* Outgoing sample buffer (pushed to stack) */

rone     .req r1  /* The value 1 */
rreg     .req r2  /* Register to sample pin */
rmash    .req r3  /* Mask/shift to isolate required pin */
rval     .req r4  /* Currently-sampled value */

rsample1 .req r5  /* Most recent 32-bits */
rsample2 .req r6  /* Next 32-bits */
rsample3 .req r7  /* Remaining 24-bits */

rlastval .req r8  /* What value was the last pin? */
rcounter .req r9
runstuff .req r10

rdpiaddr   .req r11
rdniaddr   .req r12


.func usbPhyReadI
.global usbPhyReadI
/*int */usbPhyReadI/*(const USBPHY *phy, uint8_t samples[12])*/:
  push {r2,r3,r4,r5,r6,r7,lr}
  push {routptr}

  // We need an extra lo register for the unstuff pattern #0b111111, but
  // we're all out.  As a complete and total hack, re-use the stack pointer
  // to index into the USBPHY struct.
  // This means that we can't push anything onto the stack, nor can we call
  // any functions that do.
  mov r2, sp
  str r2, [r0, #spSave]
  mov rusbphy, r0

  /* Clear out the register shift-chain */
  mov rsample1, #0
  mov rsample2, #0
  mov rsample3, #0
  mov rcounter, rsample1
  mov r0, #0b11
  mov runstuff, r0                  // Load 0b11 into unstuff reg, as the header
                                    // ends with the pattern KK, which starts
                                    // a run of two.
  mov rusmask, #0b111111            // Unstuff mask
  mov rone, #1                      // Actually load the value '1' into the reg.

  ldr rreg, [rusbphy, #dpIAddr]     // Grab the address for the data input reg.
  ldr rmash, [rusbphy, #dpMask]     // Grab the mask for the bit.

  /* Wait for the line to flip */
  ldr rval, [rreg]                  // Sample D+, to watch for it flipping
  and rval, rval, rmash             // Mask off the interesting bit
  mov rlastval, rval                // Save the bit for use in looking for sync

#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  add rreg, #4
  str rreg, [rusbphy, #dpIAddr]     // Move read test register up by 4
  ldr rreg, [rusbphy, #dnIAddr]
  add rreg, #4
  str rreg, [rusbphy, #dnIAddr]     // Move read test register up by 4
  ldr rreg, [rusbphy, #dpIAddr]
#endif /* defined(USB_PHY_READ_TEST) */

  // The loop is 4 cycles on a failure.  One
  // pulse is 32 cycles.  Therefore, loop up
  // to 8 times before giving up.
#if !defined(USB_PHY_READ_TEST)
.rept 9
  ldr rval, [rreg]                  // Sample USBDP
  and rval, rval, rmash             // Mask off the interesting bit
  cmp rval, rlastval                // Wait for it to change
  bne usb_phy_read_sync_wait        // When it changes, go wait for sync pulse
.endr
  b usb_phy_read_timeout            // It never changed, so return "timeout".
#endif

usb_phy_read_sync_wait:
#if !defined(USB_PHY_READ_TEST)
  // Wait until we're in the middle of a pulse.  When we get here, the pulse
  // will have happened between 6 and 10 cycles ago.  Since the middle of a
  // pulse occurs at 16 cycles, delay 8 cycles to line us up.
  bl usb_phy_wait_13_cycles
#endif /* !defined(USB_PHY_READ_TEST) */

  // Wait for the end-of-header sync pulse, which is when the value
  // repeats itself.  This is the "KK" in the KJKJKJKK training sequence.
.rept 7
  ldr rval, [rreg]                    // Sample USBDP
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  add rreg, #4
  str rreg, [rusbphy, #dpIAddr]       // Move read test register up by 4
  ldr rreg, [rusbphy, #dnIAddr]
  add rreg, #4
  str rreg, [rusbphy, #dnIAddr]       // Move read test register up by 4
  ldr rreg, [rusbphy, #dpIAddr]
#endif /* defined(USB_PHY_READ_TEST) */
  and rval, rval, rmash               // Mask off the interesting bit
  cmp rlastval, rval
  beq start_reading_usb
  mov rlastval, rval
#if !defined(USB_PHY_READ_TEST)
  bl usb_phy_wait_27_cycles
#endif /* defined(USB_PHY_READ_TEST) */
.endr
  b usb_phy_sync_timeout

  /* We're synced to the middle of a pulse, and the clock sync / start-of-
   * -frame has been found.  Real packet data follows.
   */
start_reading_usb:

  /* Adjust rlastval so that it's in the correct position -- we skip doing
     this above since we're only interested in the value changing, not in
     what the value is.  However, we're now interested in what the value
     is, so we now deal with shifts instead of masks, and always mask by
     #1.
   */
  mov rval, rlastval
  ldr rmash, [rusbphy, #dpShift]
  ror rval, rval, rmash
  and rval, rval, rone
  mov rlastval, rval

  ldr rreg, [rusbphy, #dpIAddr]       // Cache the address of the D+ input bank
  mov rdpiaddr, rreg                  // to save one cycle.
  ldr rreg, [rusbphy, #dnIAddr]       // Cache the address of the D- input bank
  mov rdniaddr, rreg                  // to save another cycle.

usb_phy_read_get_usb_bit:
  mov rval, rdpiaddr                  // Get the address of the D+ input bank.
#if defined(USB_PHY_READ_TEST)
  /* Advance to the next sample */
  mov rreg, rval
  ldr rval, [rval]                    // Actually sample D+
  add rreg, #4
  str rreg, [rusbphy, #dpIAddr]       // Move read test register up by 4
  ldr rreg, [rusbphy, #dnIAddr]
  ldr rmash, [rreg]
  add rreg, #4
  str rreg, [rusbphy, #dnIAddr]       // Move read test register up by 4
  mov rreg, rmash
#else
  mov rreg, rdniaddr                  // Get the address of the D+ input bank.
  ldr rval, [rval]                    // Actually sample D+
  ldr rreg, [rreg]                    // Also sample D-
#endif /* defined(USB_PHY_READ_TEST) */
  ldr rmash, [rusbphy, #dpShift]      // Get the mask of the D+ bit
  ror rval, rval, rmash               // Rotate the value down to bit 1.
  and rval, rval, rone                // Mask off everything else.
  // 8

  // Check for SE0
#if 1
  ldr rmash, [rusbphy, #dnMask]
  and rreg, rreg, rmash
  add rreg, rreg, rval                // An end-of-frame is indicated by two
                                      // frames of SE0.  If this is the case,
                                      // then the result of adding these
                                      // together will result in 0.
  beq usb_phy_read_exit               // Exit if so.
#else
  mov rmash, rcounter
  cmp rmash, #88
  beq usb_phy_read_exit
#endif
  // 5

  mov rmash, rlastval
  mov rlastval, rval
  eor rval, rmash                     // Check to see if the state has changed.
  mvn rval, rval                      // Invert, as 1 ^ 1 or 0 ^ 0 should be 1.
  and rval, rval, rone                // Mask off everything but the last bit.

  add runstuff, runstuff, runstuff    // Shift the "unstuff" value up by 1.
  add runstuff, runstuff, rval        // "or" in the one-bit rval to the bottom.

  lsr rval, rval, rone                // Shift the state into the carry bit
  adc rsample1, rsample1, rsample1    // Propagate the carry bit up
  adc rsample2, rsample2, rsample2    // Propagate the carry bit up
  adc rsample3, rsample3, rsample3    // Propagate the carry bit up
  // 11 [9]

  /* Restore values, increase rcounter */
  add rcounter, rcounter, rone
  // 1

  nop
  nop

  // Figure out if we need to unstuff, or if we can just continue to the
  // next bit.
  // Six consecutive USB states will be followed by a dummy
  // state flip.  Ignore this.
  mov rreg, runstuff
  and rreg, rreg, rusmask
  cmp rreg, rusmask
  bne usb_phy_read_get_usb_bit
  // 5 [0]

usb_unstuff:
  nop                                 // We get here when the current bit has
                                      // one more clock cycle left.  Add a nop
                                      // just to make the cycle-counting easier.
  /* --- New pulse starts here --- */
  // NOTE that we don't increment "rcounter" here.
  mov runstuff, rone                  // We're skipping over one bit, which
                                      // results in a new run of one.
  // 1

#if defined(USB_PHY_READ_TEST)
  ldr rreg, [rusbphy, #dnIAddr]
  add rreg, #4
  str rreg, [rusbphy, #dnIAddr]       // Move read test register up by 4
  ldr rreg, [rusbphy, #dpIAddr]
  add rreg, #4
  str rreg, [rusbphy, #dpIAddr]       // Move read test register up by 4
#endif /* defined(USB_PHY_READ_TEST) */


  /* Invert the last value, since a false 0 was added to the stream */
  mov rreg, rlastval                  // Read the current last val into a lo reg.
  mvn rreg, rreg                      // Negate the value.
  and rreg, rreg, rone                // Mask it with 0b1
  mov rlastval, rreg                  // Save it back into a lo reg.
  // 4

#if !defined(USB_PHY_READ_TEST)
  bl usb_phy_wait_23_cycles
#endif /* defined(USB_PHY_READ_TEST) */
  b usb_phy_read_get_usb_bit
  // 2

usb_phy_read_exit:
  ldr r2, [rusbphy, #spSave]
  mov sp, r2
  pop {routptr}

#if 0
  // Allocate 12 bytes on the stack.
  sub r2, #12

  // Store samples on the stack, where we can swap values and copy to dest.
  str rsample1, [r2, #0]
  str rsample2, [r2, #4]
  str rsample3, [r2, #8]
#endif

  retval .req r0
  srcptr .req r2
  dstoff .req r3
  srcoff .req r4
  endoff .req r5
  tmpval .req r6
  table  .req r7

  /* Count the number of bytes read (rcounter / 8) */
  mov retval, rcounter
  mov r3, #3
  asr retval, #3                  // Return number of bytes (not bits) read.
  cmp retval, #11                 // Error out if we read more than 11 bytes.
  bgt usb_phy_read_overflow_exit

  cmp retval, #0
  beq usb_phy_read_no_data_exit

#if 0
  mov srcoff, #0
  mov dstoff, retval
  sub dstoff, #1
  ldr table, =bit_reverse_table_256

copy_loop_top:
  ldrb tmpval, [srcptr, srcoff]
  add srcoff, #1

  // Reverse bit order using the bit_reverse_table_256
  add tmpval, table
  ldrb tmpval, [tmpval]

  strb tmpval, [routptr, dstoff]
  sub dstoff, dstoff, #1

  cmp srcoff, retval
  bne copy_loop_top
#else
  str rsample1, [routptr, #0]
  str rsample2, [routptr, #4]
  str rsample3, [routptr, #8]
#endif
final_exit:
  pop {r2,r3,r4,r5,r6,r7,pc}

  /* Read too many bits, return -2 */
usb_phy_read_overflow_exit:
  mov r1, #2
  mov r0, #0
  sub r0, r0, r1
  pop {r2,r3,r4,r5,r6,r7,pc}

usb_phy_read_no_data_exit:
  mov r0, #0
  pop {r2,r3,r4,r5,r6,r7,pc}

usb_phy_sync_timeout:
  ldr r2, [rusbphy, #spSave]
  mov sp, r2
  pop {routptr}
  mov r0, #0
  sub r0, r0, #3
  pop {r2,r3,r4,r5,r6,r7,pc}

usb_phy_read_timeout:
  ldr r2, [rusbphy, #spSave]
  mov sp, r2
  pop {routptr}
  mov r0, #0
  sub r0, r0, #1
  pop {r2,r3,r4,r5,r6,r7,pc}

.endfunc
.type usbPhyReadI, %function
.size usbPhyReadI, .-usbPhyReadI



usb_phy_wait_32_cycles: nop
usb_phy_wait_31_cycles: nop
usb_phy_wait_30_cycles: nop
usb_phy_wait_29_cycles: nop
usb_phy_wait_28_cycles: nop
usb_phy_wait_27_cycles: nop
usb_phy_wait_26_cycles: nop
usb_phy_wait_25_cycles: nop
usb_phy_wait_24_cycles: nop
usb_phy_wait_23_cycles: nop
usb_phy_wait_22_cycles: nop
usb_phy_wait_21_cycles: nop
usb_phy_wait_20_cycles: nop
usb_phy_wait_19_cycles: nop
usb_phy_wait_18_cycles: nop
usb_phy_wait_17_cycles: nop
usb_phy_wait_16_cycles: nop
usb_phy_wait_15_cycles: nop
usb_phy_wait_14_cycles: nop
usb_phy_wait_13_cycles: nop
usb_phy_wait_12_cycles: nop
usb_phy_wait_11_cycles: nop
usb_phy_wait_10_cycles: nop
usb_phy_wait_9_cycles:  nop
usb_phy_wait_8_cycles:  nop
usb_phy_wait_7_cycles:  nop
usb_phy_wait_6_cycles:  nop
usb_phy_wait_5_cycles:  mov pc, lr





/*
 * usbPhyWriteI
 * Register (arguments):
 *   r0: USBPHY
 *   r1: internal buffer data, prepared already
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
wnextpkt  .req r10
wdpsetreg .req r11
wdpclrreg .req r12
wdnclrreg .req sp

/* Indexes off of internal data */
.equ wDnSAddr,0x00      /* D- Set Addr */
.equ wDnCAddr,0x04      /* D- Clear Addr */
.equ wPkt1,0x08
.equ wPkt1Num,0x0c
.equ wPkt2,0x10
.equ wPkt2Num,0x14
.equ wPkt3,0x18
.equ wPkt3Num,0x1c
.equ wFirstPkt,0x20
.equ wSpSave,0x24

.func usbPhyWriteI
.global usbPhyWriteI
/*int */usbPhyWriteI/*(const USBPHY *phy, USBPHYInternal *internal)*/:
  push {r2,r3,r4,r5,r6,r7,lr}
  push {r0}                         // Save usbphy so we can restore pins later

  ldr wtmp1, [r0, #dnSAddr]         // Grab the D- address for our temp reg
  str wtmp1, [r1, #wDnSAddr]        // Store it in USBPHYInternal

  ldr wtmp1, [r0, #dnCAddr]         // Grab the D- address for our temp reg
  str wtmp1, [r1, #wDnCAddr]        // Store it in USBPHYInternal

  ldr wtmp1, [r1, #wFirstPkt]       // Load the first packet out of internal
  mov wpktnum, wtmp1                // Save it in wpktnum for later.

usb_phy_write_get_first_packet:
  // Read the first packet
  lsl wtmp1, #3                     // Multiply the packet number by 3.
  add wtmp1, r1                     // Add it to the start of the usbphy struct.
  ldr wpkt, [wtmp1, #0]             // Load the value, which is the packet.
  ldr wleft, [wtmp1, #4]            // Load the number of bits left.

  /* Load D+ set and clear registers early on */
  ldr wtmp1, [wusbphy, #dpSAddr]    // Registers are faster than RAM, and we
  mov wdpsetreg, wtmp1              // only have two free registers, so pre-
  ldr wtmp1, [wusbphy, #dpCAddr]    // cache the D+ addresses to save one
  mov wdpclrreg, wtmp1              // clock cycle.

  /* Load D+ and D- masks, used for direction and value setting */
  ldr wpmask, [wusbphy, #dpMask]    // USB D+ mask
  ldr wnmask, [wusbphy, #dnMask]    // USB D- mask

  /* Pre-set the lines to J-state to prevent glitching */
#if 1
  mov wpaddr, wdpsetreg             // D+ set
  ldr wnaddr, [r1, #wDnCAddr]       // D- clr
#else
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [r1, #wDnSAddr]       // D- set
#endif
  str wpmask, [wpaddr]              // Write D+ value
  str wnmask, [wnaddr]              // Write D- value

  /* Set D+ line to OUTPUT */
  ldr wtmp1, [wusbphy, #dpDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  orr wtmp2, wtmp2, wpmask          // Set the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D+

  /* Set D- line to OUTPUT */
  ldr wtmp1, [wusbphy, #dnDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  orr wtmp2, wtmp2, wnmask          // Set the direciton mask
  str wtmp2, [wtmp1]                // Set the direction for D-

  mov wusbphy, r1                   // Use internal data
  mov wlastsym, #1                  // Last symbols were "KK" from the header,
  mov wtmp1, #0b111100              // so load a run of 2 into the
  mov wstuff, wtmp1                 // stuff value.

  // Save sp, since we reuse it for D- clr address.
  mov wtmp1, sp
  str wtmp1, [wusbphy, #wSpSave]

  ldr wtmp1, [wusbphy, #wDnCAddr]   // Cache D- clr as well.
  mov wdnclrreg, wtmp1

  // usb start-of-frame header //
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_write_state_j
  bl usb_write_state_k
  bl usb_phy_wait_26_cycles         // Hold k state for one more cycle.  Take
                                    // up the slack that would normally
                                    // follow this.
  // end of header //

usb_phy_write_top:

  mov wtmp1, #0                     // Clear wthisbit, so we can add later.
  lsr wpkt, #1                      // Shift the bottom bit into the carry bit.
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
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg
  b usb_phy_write_out
  
usb_phy_write_j:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [wusbphy, #wDnSAddr]  // D- set

usb_phy_write_out:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  // 7 (either branch taken)

  // 15 cycles total

  mov wtmp1, #0b111111              // See if we just wrote "111111"
  mov wtmp2, wstuff                 // Compare it with the wstuff value
  and wtmp2, wtmp1                  // AND the two together.  If they're the
  beq usb_phy_write_stuff_bit       // same, then stuff one bit.
  // 4

usb_phy_write_done_stuffing_bit:
  sub wleft, wleft, #1              // See how many bits we have left to write.
  bne usb_phy_write_continue_word   // If nonzero, write another bit.
  // 2

usb_phy_write_finished_word:
  mov wtmp1, wpktnum                // Move wpktnum into a lo reg
  sub wtmp1, #1                     // Subtract one from the output
  beq usb_write_eof                 // Exit if it's now 0.
  mov wpktnum, wtmp1                // ...and store it back in the hi reg.

usb_phy_write_calculate_next_pkt:
  mov wpkt, wnextpkt                // Load the prefetched next packet
  lsl wtmp1, #3                     // Convert pktnum into 32-bit offset
  add wtmp1, #4                     // The "bits left" is stored 4 bytes above.
  ldr wleft, [wusbphy, wtmp1]       // Load the value from the phy data.
  // 8

  b usb_phy_write_top

  // Pre-fetch the next packet.  We have some spare cycles here, so load
  // the next packet into wnextpkt.
usb_phy_write_continue_word:
usb_phy_write_prefetch_next_word:
  mov wtmp1, wpktnum                // Load the current pktnum (offset)
  sub wtmp1, #1                     // The next packet will be 1 less.
  lsl wtmp1, #3                     // Convert pktnum to 32-bit word offsets.
  ldr wtmp2, [wusbphy, wtmp1]       // Load the packet into wtmp2.
  mov wnextpkt, wtmp2               // Move the packet into the hi wnextpkg reg.
  .rept 2
  nop
  .endr

  b usb_phy_write_top
  // 2

usb_phy_write_stuff_bit:
// 5 vs 8 cycles
// The real path is 3 cycles longer
// There are 2 + 2 + 7 skipped cycles here, too.
  bl usb_phy_wait_13_cycles
  mov wtmp1, #0b111110              // Clear out the bit-stuff rcounter
  mov wstuff, wtmp1
  // 2

  add wlastsym, wlastsym, #1        // Invert the last symbol.

  mov wtmp1, #0b1                   // See if we need to send j or k
  tst wlastsym, wtmp1
  // 3

  /* Write the desired state out (each branch is balanced) */
  bne usb_phy_write_stuff_j
usb_phy_write_stuff_k:
  mov wpaddr, wdpsetreg             // D+ set
  nop
  ldr wnaddr, [wusbphy, #wDnCAddr]   // D- clr
  b usb_phy_write_stuff_out
  
usb_phy_write_stuff_j:
  mov wpaddr, wdpclrreg             // D+ clr
  nop
  ldr wnaddr, [wusbphy, #wDnSAddr]   // D- set
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
  bl usb_write_state_j

  // --- Done Transmitting --- //

  // Restore sp, since we're done with it
  ldr wtmp1, [wusbphy, #wSpSave]
  mov sp, wtmp1

  pop {r0}                          // Restore wusbphy

  // Now, set both lines back to INPUT
  ldr wtmp1, [wusbphy, #dpDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  ldr wpmask, [wusbphy, #dpMask]    // Get the mask value for ORing in
  bic wtmp2, wtmp2, wpmask          // Clear the direciton mask
  ldr wtmp1, [wusbphy, #dpDAddr]    // Get the direction address
  str wtmp2, [wtmp1]                // Set the direction for Dp

  ldr wtmp1, [wusbphy, #dnDAddr]    // Get the direction address
  ldr wtmp2, [wtmp1]                // Get the direction value
  ldr wnmask, [wusbphy, #dnMask]    // Get the mask value for ORing in
  bic wtmp2, wtmp2, wnmask          // Clear the direciton mask
  ldr wtmp1, [wusbphy, #dnDAddr]    // Get the direction address
  str wtmp2, [wtmp1]                // Set the direction for Dp

  mov r0, #0                        // Return 0
  pop {r2,r3,r4,r5,r6,r7,pc}        // Return to caller

  // Useful functions
usb_write_state_se0:
  mov wpaddr, wdpclrreg             // D+ clr
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write_out_func
usb_write_state_j:
  mov wpaddr, wdpsetreg             // D+ set
  mov wnaddr, wdnclrreg             // D- clr
  b usb_phy_write_out_func
usb_write_state_k:
  mov wpaddr, wdpclrreg             // D+ clr
  ldr wnaddr, [wusbphy, #wDnSAddr]   // D- set
  nop
usb_phy_write_out_func:
  str wpmask, [wpaddr]
  str wnmask, [wnaddr]
  b usb_phy_wait_24_cycles

.endfunc
.type usbPhyWriteI, %function
.size usbPhyWriteI, .-usbPhyWriteI

