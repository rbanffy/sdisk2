/*------------------------------------------------------

	DISK II Emulator Farmware (1 of 2) for ATMEGA328P
	
	2009.11.16 by Koichi Nishida

------------------------------------------------------*/

/*
This is a part of the firmware for DISK II emulator by Nishida Radio.

Copyright (C) 2009 Koichi NISHIDA
email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
if the crystal on your SDISK II is 25 MHz, 
I recommend you to replace it with 27 MHz,
or ask Nishida Radio.
if you don't want to replace it, change the following to
.equ CRYSTAL, 25
*/

.equ CRYSTAL, 27

.equ PINB, 0x03
.equ DDRB, 0x04
.equ PORTB, 0x05
.equ PINC, 0x06
.equ DDRC, 0x07
.equ PORTC, 0x08
.equ PIND, 0x09
.equ DDRD, 0x0a
.equ PORTD, 0x0b
.equ SREG, 0x3f
.equ TCNT0, 0x26

.global __vector_1
.global __vector_16
.global wait5

.global readPulse
.global bitByte
.global sector
.global prepare
.global writeData
.global writeBack
.global writePtr

.func wait5
wait5:
	ldi r18,24
wait51:
	nop
	dec r18
	brne wait51
	sbiw r24,1
	brne wait5
	ret
.endfunc	
	
.func wait1
wait1:
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
.if CRYSTAL==25
	nop		; 1
.endif
	ret		; 4
.endfunc

.func __vector_16
__vector_16:
	push	r26
	in		r26, SREG
	push	r26
	push	r27
	push	r18
	lds		r26,readPulse
	lds		r18,protect
	or		r26,r18
	out 	PORTC,r26
.if CRYSTAL==27	
	ldi		r26,170		; 1
.else
	ldi		r26,180 	; 1
.endif
	out		TCNT0,r26	; 1
	ldi		r18,0		; 1
	rcall	wait1		; 11
	lds		r26,protect	; 2
	out 	PORTC,r26
	lds		r27,prepare
	and		r27,r27
	breq 	NOT_PREPARE
	sts		readPulse,r18
	pop		r18
	pop		r27
	pop		r26
	out		SREG,r26	
	pop		r26
	reti
NOT_PREPARE:
	ldi		r26,0b00110000	; 1
	out		PORTD,r26		; 1
	in		r26,PIND		; 1
	andi	r26,1			; 1
	lsl		r26				; 1
	mov		r18,r26			; 1
	ldi		r26,0b00010000	; 1
	out		PORTD,r26		; 1
	lds		r26,bitbyte
	lds		r27,(bitbyte+1)
	adiw	r26,1
	sts		bitbyte,r26
	sts		(bitbyte+1),r27
	cpi		r26,((402*8)%256)
	brne	LBL1
	cpi		r27,((402*8)/256)
	brne	LBL1
	; set prepare flag
	ldi		r26,1
	sts		prepare,r26
	; discard 112 byte (including CRC 2 byte)
	push	r28
	ldi		r28,112
DSC_LP2:
	ldi		r26,8
DSC_LP1:
	ldi		r27,0b00110000	; 1
	out		PORTD,r27		; 1
	ldi		r27,0b00010000	; 1
	out		PORTD,r27	
	dec		r26
	brne	DSC_LP1
	dec		r28
	brne	DSC_LP2
	pop		r28
LBL1:
	sts		readPulse,r18
	pop		r18
	pop		r27
	pop		r26
	out		SREG,r26	
	pop		r26
	reti
.endfunc

.func __vector_1
__vector_1:
	push	r18			; 1
	in		r18, SREG	; 1
	push	r18			; 1
	sbic	PINC,0
	rjmp	NOT_ENABLE
	push	r19			; 2
	lds		r19,magState; 2
WLP8:
	; wait start bit 1
	in		r18,PINC	; 1
	andi	r18,4		; 1
	eor		r18,r19		; 1
	breq	WLP8		; 2/1
	in		r18,PINC	; 1
	andi	r18,4		; 1
	sts		magState,r18; 2
	ldi		r18, 8		; 1
WLP9:
	dec		r18			; 1
	brne	WLP9		; 2
	nop					; 1
	push	r20			; 2
	push	r21			; 2
	push	r22			; 2
	push	r23			; 2
	push	r24			; 2
	push	r30			; 2
	push	r31			; 2
	ldi		r22,0		; 1 start storing
	lds		r30,(writePtr)
	lds		r31,(writePtr+1)
	ldi		r19,lo8(349) ;1
	ldi		r20,hi8(349) ;1 
	rjmp	ENTR		; 2
WLP2:
	lds		r21,magState; 2
WLP6:
	; wait start bit 1
	in		r23,PINC	; 1
	andi	r23,4		; 1
	eor		r23,r21		; 1
	breq	WLP6		; 2/1
	in		r23,PINC	; 1
	andi	r23,4		; 1
	sts		magState,r23; 2
	ldi		r23, 14		; 1
WLP7:
	dec		r23			; 1
	brne	WLP7		; 2
ENTR:
	ldi		r18,7		; 1
	ldi		r24,1		; 1
WLP1:
	in		r23,PIND	; 1
	andi	r23,4		; 1
	brne	WRITE_END	; 1	
	nop					; 1	
.if CRYSTAL==27	
	ldi		r23, 30		; 1
.else
	nop
	ldi		r23,27		; 1
.endif
WLP3:
	dec		r23			; 1
	brne	WLP3		; 2
WLP5:
	in		r23,PINC	; 1
	andi	r23,4		; 1
	lds		r21,magState; 2
	sts		magState,r23; 2
	eor		r23,r21		; 1
	lsr		r23			; 1
	lsr		r23			; 1
	lsl		r24			; 1
	or		r24,r23		; 1
	dec		r18			; 1
	brne	WLP1		; 2/1
	cpi		r24,0xD5	; 1
	brne	NOT_START	; 2/1
	ldi		r22,1		; 1
NOT_START:
	cpi		r22,0		; 1
	breq	WLP2		; 1
	st		Z+,r24		; 2
	subi	r19,1		; 1
	sbci	r20,0		; 1
	brne	WLP2		; 2/1
WRITE_END:

	push	r25
	push	r26
	push	r27
	call	writeBack
	pop		r27
	pop		r26
	pop		r25
	pop		r31
	pop		r30
	pop		r24
	pop		r23
	pop		r22
	pop		r21
	pop		r20
	pop		r19
NOT_ENABLE:
	pop		r18
	out		SREG,r18	
	pop		r18
	reti
.endfunc

