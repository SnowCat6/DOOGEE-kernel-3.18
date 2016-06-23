@;;; arm-none-eabi-gcc  -nostdlib ./fake_bootslave.S -o fake_bootslave.axf -Ttext=0

	.text
	.org 0
	.global _bootslave
	.global _start
	.func _start
_bootslave:
_start:	
	adr 	r0, br_base 		@SRAMROM controller literal
	ldr 	r0, [r0] 		@SRAMROM controller base
	ldr 	r1, [r0, #4] 
	tst 	r1, #(1<<31)  		@SRAMROM power-down mode ? 
	beq 	_sramrom 
	
_jump: 
	ldr 	r2, [r0]		@jump_addr
	cmp 	r2, #0
	bxne 	 r2			@@ fixme, 
	                                @@ 'ne' is a workaround
					@@ to avoid FPGA always-SRAMROM-power-down mode.

	
_sramrom:  
	adr    	r0, boot_addr  			@boot addr
	ldr	r0, [r0]
	mrc 	p15, 0x0, r1, c0, c0, 0x5 	@MPIDR 
	ubfx 	r1, r1, #0, #15	 		@cluster-0, cpu-0?
	tst	r1, #0
	bne 	_sboot 
	
	and    	r2,r1,#0x3  			@cpu id  
	ands   	r3,r1,#0xf00 			@cluster id 
	add 	r3,r2,r3,lsr #0x6  		@index  
	mov 	r4, r3 	
	moveq  	r3, #0x4  			@ fix magic number array 
	
	ldr  	r1, [r0, r3,lsl #0x2] 		@ magic num 
	adr 	r2, magic_key	
	ldr	r2, [r2, r4, lsl #0x2] 
	cmp	r1, r2 
	beq	_sboot 
	
	dsb 
	wfi 
	nop 
	b 	_sramrom 

_sboot: 
	ldr    r2,[r0] 
	bx     r2 
	
	nop 
		
boot_addr:
	.long 0x10202034, 0,  0 
br_base:
	.long 0x10001800	@bootrom enable 
magic_key:
	.long 0, 0x534C4131, 0x4C415332, 0x41534C33, 0x12345678, 0x87654321, 0, 0
