.text

  /***
    WS2812B Low-Level code
      WS2812B timing requirements:
      To transmit a "0": 400ns high, 850ns low  +/- 150ns  total = 1250ns
	19 cycles H / 41 cycles L
      To transmit a "1": 800ns high, 450ns low  +/- 150ns  total = 1250ns
	38 cycles H / 22 cycles L
      Clock balance to within +/-150ns = 7 cycles

      Reset code is 50,000ns of low

      Each instruction takes 20.8ns, 1250/20.8 = 60 instructions per cycle total
      
      Data transmit to chain is {G[7:0],R[7:0],B[7:0]}

      Data passed in shall be an array of bytes, GRB ordered, of length N specified as a parameter

   ***/

samples .req r0
count   .req r1

curval  .req r2  // Current GPIO value we're working on
workval .req r3  // Current GPIO value we're working on

dnmask  .req r4  // mask to examine D-
dpmask  .req r5  // mask to examine D+

gpio    .req r6
delays  .req r7

	// r2 is GPIO dest location
	// r0 is "0" bit number code
	// r1 is "1" bit number code
	// r3 is loop counter for total number of pixels
	// r4 is the current pixel value
	// r5 is the test value for the top bit of current pixel
	// r6 is the loop counter for bit number in a pixel
	// r7 is current pointer to the pixel array
	
.func usbPhyRead
.global usbPhyRead
usbPhyRead/*(uint8_t *samples, uint32_t count)*/:
	// r0  uint8_t *samples
	// r1  uint32_t	count
	push {r4,r5,r6,r7}

	ldr gpio, FGPIOREG
	ldr dnmask, DNMASK
	ldr dpmask, DPMASK

  mov delays, #0
wait_for_zero:
  ldr curval, [gpio, #0x50]           // 2
  and curval, curval, dpmask    // 1
  cmp curval, #0
  bne wait_for_zero

wait_for_one:
  ldr curval, [gpio, #0x50]           // 2
  and curval, curval, dpmask    // 1
  cmp curval, #0
  add delays, #4
  beq wait_for_zero

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


	/////////////////////// /* 32 cycles (1.5 Mbit/s @ 48 MHz ) total */
get_usb_bit:
  ldr workval, [gpio, #0x10]           // 2
  ldr curval, [gpio, #0x50]            // 2

  and curval, curval, dpmask    // 1

  lsr workval, workval, #3      // 1
  and workval, workval, dnmask  // 1

  orr curval, curval, workval   // 1

  strb curval, [samples]        // 2

  /* 16 delays(?) */

  mov workval, delays           // 1
delay_loop:
  sub workval, #1               // 1
  cmp workval, #0               // 1
  bne delay_loop                // 3

  add samples, samples, #1      // 1
  sub count, count, #1          // 1

  cmp count, #0                 // 1

  bne get_usb_bit               // 3

exit:	
	
	pop {r4,r5,r6,r7}
	bx lr

.type usbPhyRead, %function
.size usbPhyRead, .-usbPhyRead
	
.balign 4
DNMASK:
.word 0x00002
DPMASK: /* Read from FGPIOB_PDIR */
.word 0x00001
FGPIOREG:	 /* FGPIOB_PDIR */
.word 0xF8000000

.end
	
