;====================================================================================================
;
;    Filename:      NerfCannonLights.asm
;    Date:          4/4/2015
;    File Version:  1.0d1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    NerfCannonLights receives a pulse from the gun controller and 
;      flashes the lights down the barrel.
;
;
;    History:
;
; 1.0d1   4/4/2015	First code. Copied from StepperTest.
;
;====================================================================================================
; Options
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink and the barrel lights cycle once.
;
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2) Latch address A2 (output)
;   Pin 2 (RA3/AN3) Latch data (active high output)
;   Pin 3 (RA4/AN4) System LED (Active Low Output)
;   Pin 4 (RA5/MCLR*) N.C.
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) N.C.
;   Pin 7 (RB1/AN11/SDA1) I2C Data
;   Pin 8 (RB2/AN10/RX) N.C.
;   Pin 9 (RB3/CCP1) N.C.
;
;   Pin 10 (RB4/AN8/SLC1) I2C Clock
;   Pin 11 (RB5/AN7)  LED0..7 Chip Select (Active Low Output)
;   Pin 12 (RB6/AN5/CCP2) LED8..15 Chip Select (Active Low Output)
;   Pin 13 (RB7/AN6) LED16..23 Chip Select (Active Low Output)
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) N.C.
;   Pin 16 (RA7) N.C.
;   Pin 17 (RA0) Latch address A0 (output)
;   Pin 18 (RA1) Latch address A1 (output)
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=0	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_OFF & _LVP_OFF
;
; Write protection off
; 4x PLL disabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=0
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'11100000'
#Define	LED_A0	LATA,0	;Output
#Define	LED_A1	LATA,1	;Output
#Define	LED_A2	LATA,2	;Output
#Define	LED_D	LATA,3	;Output
#Define	SystemLED	LATA,4	;Output, 0=LED ON
#Define	RA5_In	PORTA,5	;unused
#Define	RA6_In	PORTA,6	;unused
#Define	RA7_In	PORTA,7	;unused
;
LED_AddrDataMask	EQU	0xF0
;
PortAValue	EQU	b'00010000'
;
;    Port B bits
PortBDDRBits	EQU	b'00011111'	;LEDs Out Others In
;
#Define	RB0_In	PORTB,0	;unused
#Define	RB1_In	PORTB,1	;I2C Data
#Define	RB2_In	PORTB,2	;unused
#Define	RB3_In	PORTB,2	;unused
#Define	RB4_In	PORTB,4	;I2C Clock
#Define	LED0_E	LATB,5	;LED0..7 Chip Select (Active Low Output)
#Define	LED8_E	LATB,6	;LED8..15 Chip Select (Active Low Output)
#Define	LED16_E	LATB,7	;LED16..23 Chip Select (Active Low Output)
;
PortBValue	EQU	b'11100000'
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
TMR0Val	EQU	0xB2	;0xB2=100Hz, 0.000128S/Count
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
kWDTime	EQU	d'200'	;2 seconds
;
T1CON_Val	EQU	b'00000001'	;PreScale=1,Fosc/4,Timer ON
TMR1L_Val	EQU	0x3C	; -2500 = 2.5 mS, 400 steps/sec
TMR1H_Val	EQU	0xF6
;
;TMR1L_Val	EQU	0x1E	; -1250 = 1.25 mS, 800 steps/sec
;TMR1H_Val	EQU	0xFB
;
;TMR1L_Val	EQU	0x8F	; -625 = 0.625 mS, 1600 steps/sec
;TMR1H_Val	EQU	0xFD
;
TXSTA_Value	EQU	b'00100000'	;8 bit, TX enabled, Async, low speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
; 8MHz clock low speed (BRGH=0,BRG16=1)
Baud_300	EQU	d'1666'	;0.299, -0.02%
Baud_1200	EQU	d'416'	;1.199, -0.08%
Baud_2400	EQU	d'207'	;2.404, +0.16%
Baud_9600	EQU	d'51'	;9.615, +0.16%
BaudRate	EQU	Baud_9600
;
;
DebounceTime	EQU	d'10'
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
Bank0_Vars	udata	0x20 
;
ISR_Temp	RES	1	;scratch mem
LED_Time	RES	1
lastc	RES	1	;part of tickcount timmer
tickcount	RES	1	;Timer tick count
;
StatLED_Time	RES	1
Stat_Count	RES	1
;
TXByte	RES	1	;Next byte to send
RXByte	RES	1	;Last byte received
WorkingRXByte	RES	1
RS232Flags	RES	1
#Define	DataSentFlag	RS232Flags,0
#Define	DataReceivedFlag	RS232Flags,1
;
;
;
EEAddrTemp	RES	1	;EEProm address to read or write
EEDataTemp	RES	1	;Data to be writen to EEProm
;
;
Timer1Lo	RES	1	;1st 16 bit timer
Timer1Hi	RES	1	; one second RX timeiout
;
Timer2Lo	RES	1	;2nd 16 bit timer
Timer2Hi	RES	1	;
;
Timer3Lo	RES	1	;3rd 16 bit timer
Timer3Hi	RES	1	;GP wait timer
;
Timer4Lo	RES	1	;4th 16 bit timer
Timer4Hi	RES	1	; debounce timer
;
;
SysFlags	RES	1
;
#Define	FirstRAMParam	MinSpdLo
#Define	LastRAMParam	SysFlags
;
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
; I2C Stuff is here
#define	I2C_ADDRESS	0x30	; Slave address
#define	RX_ELEMENTS	.32	; number of allowable array elements, in this case 32
;
Bank2_Vars	udata	0x120   
I2C_ARRAY_TX	res	RX_ELEMENTS	; array to transmit to master
I2C_ARRAY_RX 	res	RX_ELEMENTS 	; array to receive from master
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
#Define	INDEX_I2C	Param70	;I2C Data Pointer
#Define	GFlags	Param71
#Define	I2C_TXLocked	Param71,0	; Set/cleared by ISR, data is being sent
#Define	I2C_RXLocked	Param71,1	; Set/cleared by ISR, data is being received
#Define	I2C_NewRXData	Param71,2	; Set by ISR, The new data is here!
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
;	ORG	0x2000
;	DE	'1','.','0','0'
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
	cblock	0x0000
;
nvMinSpdLo;	RES	1	;0x1E; -1250 = 1.25 mS, 60RPM
nvMinSpdHi;	RES	1	;0xFB
nvMaxSpdLo;	RES	1	;0x8F; -625 = 0.625 mS, 120RPM
nvMaxSpdHi;	RES	1	;0xFD
;
nvSysFlags;	RES	1
	endc
;
#Define	nvFirstParamByte	nvMinSpdLo
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
;
	ORG	0x000	; processor reset vector
	CLRF	STATUS
	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	BSR	; bank0
;
;
	btfss	INTCON,T0IF
	goto	SystemBlink_end
;
	movlw	TMR0Val	;256x39+16 cycles (10,000uS)
	addwf	TMR0,F	; reload TMR0 with -40
	bcf	INTCON,T0IF	; reset interupt flag bit
;
;Decrement timers until they are zero
; 
	call	DecTimer1	;if timer 1 is not zero decrement
; 
	call	DecTimer2
; GP, Wait loop timer
	call	DecTimer3
; 
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
	BSF	SystemLED
	MOVF	tickcount,F
	SKPNZ
	GOTO	SystemBlinkDone
	DECF	tickcount,F
	SKPNZ
	CALL	ToggleSysLED
	GOTO	SystemBlink_end
;
SystemBlinkDone	MOVF	LED_Time,F
	SKPZ
	CAll	ToggleSysLED
;
SystemBlink_end	
;
;-----------------------------------------------------------------
; I2C Com
IRQ_4	btfss 	PIR1,SSP1IF 	; Is this a SSP interrupt?
	goto 	IRQ_4_End 	; if not, bus collision int occurred
	movlb	0x04	;banksel SSP1STAT						
	btfsc	SSP1STAT,R_NOT_W	; is it a master read:
	goto	I2C_READ	; if so go here
	goto	I2C_WRITE	; if not, go here
I2C_READ_Return
I2C_WRITE_Return	movlb	0x00
	bcf 	PIR1,SSP1IF	; clear the SSP interrupt flag
IRQ_4_End
;-----------------------------------------------------------------
; I2C Bus Collision
IRQ_5	btfss	PIR2,BCL1IF
	goto	IRQ_5_End
	movlb	0x04	;banksel SSPBUF						
	clrf	SSP1BUF	; clear the SSP buffer
	movlb	0x00	;banksel PIR2
	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag	
	movlb	0x04	;banksel SSPCON1
	bsf	SSP1CON1,CKP	; release clock stretch
	movlb	0x00
;
IRQ_5_End
;-----------------------------------------------------------------
;
	retfie		; return from interrupt
;
;
;==============================================================================================
;==============================================================================================
; This routine runs every 1/2 second. Unless an error is active then every 1/10th
;
ToggleSysLED	MOVF	LED_Time,W
	MOVWF	tickcount
	BCF	SystemLED
	RETURN
;
;==============================================================================================
;
	include	F1847_Common.inc
;
;==============================================================================================
;
start	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLB	0x01	; bank 1
	MOVLW	b'01110000'	; 8 MHz
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON 	
;	
	MOVLB	0x03	; bank 3
	CLRF	ANSELB	;Digital I/O
;
; setup timer 1 for 1uS/count
;
	MOVLB	0x00	; bank 0
	bcf	T1CON,TMR1CS0	; Fosc/4 = 2Mhz
	bcf	T1CON,TMR1CS1
	bsf	T1CON,T1CKPS0	; prescale /2
	bcf	T1CON,T1CKPS1
	bsf	T1CON,NOT_T1SYNC	;not sync'ed
	bsf	T1CON,TMR1ON	;always on
	bcf	T1GCON,TMR1GE
;
;
	MOVLB	0x00	;Bank 0
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
; setup serial I/O
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	BaudRate
	MOVWF	SPBRG
	MOVLB	0x00	; bank 0
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
;
	CLRWDT
; clear memory to zero
	CALL	ClearRam
;
	MOVLW	LEDErrorTime
	MOVWF	LED_Time
;
	MOVLW	0x01
	MOVWF	tickcount
	MOVWF	Stat_Count
;
	CALL	INITIALIZE_I2C
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,T0IE	; enable TMR0 interupt
	bsf	INTCON,GIE	; enable interupts
;
;=========================================================================================
;=========================================================================================
;  Main Loop
;
;=========================================================================================
MainLoop	CLRWDT
;
	MOVLB	0x00
	BTFSS	RB1_In
	CALL	DoLightShow
;
	goto	MainLoop
;
;=========================================================================================
;=========================================================================================
; Entry: none
; Exit: none
; RAM used: Param75..Param7B
;
DoLightShow	MOVLW	0x07
	MOVWF	Param79
	CLRF	Param7A
	CLRF	Param7B
;
DoLightShow_L1	MOVF	Param79,W
	MOVWF	Param75
	MOVF	Param7A,W
	MOVWF	Param76
	MOVF	Param7B,W
	MOVWF	Param77
	CALL	LightLEDs
;
	CALL	Light_Delay_10mS
;
	MOVF	Param79,W
	IORWF	Param7A,W
	IORWF	Param7B,W
	BTFSC	_Z
	return
;
	BCF	_C
	RLF	Param79,F
	RLF	Param7A,F
	RLF	Param7B,F
	GOTO	DoLightShow_L1
;
;=========================================================================================
; 10mS Delay
; 8MHz = 500uS/instruction
;
Light_Delay_10mS	MOVLW	.10
	MOVWF	Param78
;
Delay_W_mS	CLRF	Param77
Light_Delay_L1	NOP	
	NOP
	NOP
	NOP
	NOP
	DECFSZ	Param77,F
	GOTO	Light_Delay_L1	;4uS/Loop
	DECFSZ	Param78,F
	GOTO	Light_Delay_L1	;1025uS/Loop
;
;=========================================================================================
; Entry: Param75, Param76, Param77 24 bits to send to LEDs 0..23
; Exit: none
; RAM used: Param78
;
LightLEDs:
	MOVLB	0x02	;LATA/LATB
	CLRF	Param78	;for Param78=0 to 23
LightLEDs_L1	MOVLW	LED_AddrDataMask
	ANDWF	LATA,F
	RRF	Param77,F
	RRF	Param76,F
	RRF	Param75,F
	BTFSC	_C
	BSF	LED_D
	MOVLW	0x18
	ANDWF	Param78,W
	BTFSC	_Z
	GOTO	LightLED_0
	BTFSC	Param78,4
	GOTO	LightLED_16

LightLED_8	BCF	LED8_E
	NOP
	BSF	LED8_E
	GOTO	LightLED_Next
;
LightLED_16	BCF	LED16_E
	NOP
	BSF	LED16_E
	GOTO	LightLED_Next
;
LightLED_0	BCF	LED0_E
	NOP
	BSF	LED0_E
; Next
LightLED_Next	INCF	Param78,F
	MOVLW	.24
	SUBWF	Param78,W
	BTFSS	_Z
	GOTO	LightLEDs_L1
	return
;
;=========================================================================================
	include	I2C_SLAVE.inc
;=========================================================================================
;
;
;
;
;
	END
;
