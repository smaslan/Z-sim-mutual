;compiler: avrasm
;Simple bootloader that should fit in 512 Bytes.
;(c) Stanislav Maslan, 2020, s.maslan@seznam.cz
;The code is distributed under MIT license, https://opensource.org/licenses/MIT. 

	.include "m88def.inc"	;ATmega168 defines
	.include "macros.asm"	;macros
		
	.equ	XTAL=1000000	;XTAL frequency [Hz]
	.equ	baud=9600	;initial uart baudrate

	.dseg
page_buff:	.byte	PAGESIZE	;page buffer


	.cseg
	.equ	MYBOOTSTART=SECONDBOOTSTART
	.org	MYBOOTSTART	;bootsector reset
	rjmp	main
	.org	MYBOOTSTART+INT_VECTORS_SIZE
	reti		;safety gap in case of random ints

	;welcome string
str_welcome:	.db	"Schmutzig Bootloader Ready!",0
	;bootloader passcode
str_passcode:	.db	"17IND10",0
	
main:	;stop WDT and ISRs just in case
	cli
	adiw	R30,0	;nop,nop
	clr	R0
	sts	mcusr,R0
	sts	wdtcsr,R0
	
	outiw	sp,RAMEND	;reset stack

	;reset ports to inputs
	out	ddrb,R0
	out	ddrc,R0
	out	ddrd,R0

	;init USART
	stsi	ucsr0a,(1<<u2x0)
	stsi	ucsr0b,(1<<rxen0)|(1<<txen0)
	stsi	ucsr0c,(3<<ucsz00)
	stsiw	ubrr0l,(XTAL/(8*baud)-1)
	
	;wait a moment
	ldi	R16,50	;long when we just booted
	rcall	Delay10ms
	
	;flush RX buffer
	lds	R16,udr0
	lds	R16,udr0
		
	ldi	R16,'B'
	rcall	UsTxChar	;send bootloader ready mark
	
	;send welcome string
	ldiw	R30,str_welcome*2
boot_welc_str:	lpm	R16,Z+
	rcall	UsTxChar
	tst	R16
	brne	boot_welc_str
	
	
	;wait up to 1s for response
	ldi	R20,10
boot_wait:	ldi	R16,100
	rcall	Delay1ms
	rjss	ucsr0a,rxc0,boot_start
	;no response yet
	dbnz	R20,boot_wait
	;timeout
app_start:	jmp	0x0000	;start application

	;start bootloader		
boot_start:	lds	R16,udr0
	cpi	R16,'R'
	brne	app_start	;leave if incorrect response
	
	ldi	R16,'A'
	rcall	UsTxChar	;send ack mark
	
	;receive passcode
	ldiw	R30,str_passcode*2	;passcode string
	ldi	R18,'A'	;status: so far ok
boot_pass_loop:	rcall	UsRxChar	;get usart char
	lpm	R17,Z+	;get passcode char
	cpse	R16,R17
	ldi	R18,'F'	;status: failed
	cpi	R16,0
	breq	boot_pass_a	;string done, passed
	cpi	R17,0	;string not done
	brne	boot_pass_loop	;passcode not done	
	;passcode compared
boot_pass_a:	mov	R16,R18
	rcall	UsTxChar	;return passcode status
	
	cpi	R16,'A'
	brne	app_start	;leave if not ack		
	
	ldi	R16,PAGESIZE
	rcall	UsTxChar	;send page size
	
	ldiw	R16,MYBOOTSTART
	rcall	UsTxWord	;send available FLASH size [B]

	ldiw	R30,0x0000	;FLASH start

	
boot_next_page:	rcall	UsRxChar
	cpi	R16,'D'
	breq	app_start	;all done - start app
	
	;receive page
	ldiw	R28,page_buff
	ldi	R24,PAGESIZE*2
boot_page_rx:	rcall	UsRxChar
	st	Y+,R16
	dbnz	R24,boot_page_rx	
		
	ldi	R16,(1<<selfprgen)|(1<<pgers)
	rcall	ExecSPM	;erase page
	
	ldi	R16,(1<<selfprgen)|(1<<rwwsre)
	rcall	ExecSPM	;re-enable RWW section	

	;write page from buffer
	ldiw	R28,page_buff	;page temp buffer
	ldi	R24,PAGESIZE
boot_data_loop:	ldw	R0,Y+
	ldi	R16,(1<<selfprgen)
	rcall	ExecSPM	;write word into buffer
	adiw	R30,2	;flash next word	
	dbnz	R24,boot_data_loop	;next word?
	subiw	R30,PAGESIZE*2	;pointer back to start of page
	 	
	ldi	R16,(1<<selfprgen)|(1<<pgwrt)
	rcall	ExecSPM	;page write

	ldi	R16,(1<<selfprgen)|(1<<rwwsre)
	rcall	ExecSPM	;re-enable RWW section
	
	;page verification
	ldi	R26,'P'	;no error
	ldiw	R28,page_buff
	ldi	R24,PAGESIZE*2	;page size in bytes
boot_verif_loop:	lpm	R0,Z+	;FLASH application dword
	ld	R1,Y+	;RAM application dword
	cpse	R0,R1	;compare
	ldi	R26,'F'	;set error flag	
	dec	R24
	brne	boot_verif_loop
	
	;verify that RWW section is safe to read
boot_wait_pge_rdy:	lds	R16,spmcsr
	sbrs	R16,rwwsb
	rjmp	boot_page_written
	ldi	R16,(1<<selfprgen)|(1<<rwwsre)
	rcall	ExecSPM
	rjmp	boot_wait_pge_rdy
boot_page_written:		
	mov	R16,R26	;page write status
	rcall	UsTxChar	;send page data request
	
	rjmp	boot_next_page	;next page


;------------------------------------------------------------------------------------------
;	execute SPM instruction
;------------------------------------------------------------------------------------------
ExecSPM:	in	R17,spmcsr
	sbrc	R17,selfprgen
	rjmp	ExecSPM	;wait for complete last spm operation
	out	spmcsr,R16	;new command
	spm		;execute
	ret

;************************************************************************************************
;	Other routines
;************************************************************************************************
;------------------------------------------------------------------------------------------------
;	R16*1ms delay (0=~256ms)
;------------------------------------------------------------------------------------------------
Delay1ms:	;~1ms
	ldi	R17,XTAL/((100*3)*1000)
	;100*3 cycles
Delay1msL2:	ldi	R18,100
Delay1msL1:	dbnz	R18,Delay1msL1
	dbnz	R17,Delay1msL2
	dbnz	R16,Delay1ms	
	ret
	
;------------------------------------------------------------------------------------------------
;	R16*10ms delay (0=~2.56s)
;------------------------------------------------------------------------------------------------
Delay10ms:	mov	R15,R16
	ldi	R16,10
	rcall	Delay1ms	;10ms delay
	dbnz	R15,PC-1-1-1
	ret
	
;************************************************************************************************
;	USART routines
;************************************************************************************************
;------------------------------------------------------------------------------------------------
;	Rx R16 char
;------------------------------------------------------------------------------------------------
UsRxChar:	lds	R16,ucsr0a
	sbrs	R16,rxc0
	rjmp	UsRxChar
	lds	R16,udr0
	ret

;------------------------------------------------------------------------------------------------
;	Tx R16 char
;------------------------------------------------------------------------------------------------
UsTxChar:	lds	R17,ucsr0a
	sbrs	R17,udre0
	rjmp	UsTxChar
	sts	udr0,R16
	ret

;------------------------------------------------------------------------------------------------
;	Tx R16 word
;------------------------------------------------------------------------------------------------
UsTxWord:	push	R17
	rcall	UsTxChar
	pop	R16
	rjmp	UsTxChar
	