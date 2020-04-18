//
//
//
//		8888888b.  888b     d888 Y88b   d88P  .d8888b.   .d8888b.   .d8888b.  888b     d888 
//		888  "Y88b 8888b   d8888  Y88b d88P  d88P  Y88b d88P  Y88b d88P  Y88b 8888b   d8888 
//		888    888 88888b.d88888   Y88o88P          888 Y88b.      888    888 88888b.d88888 
//		888    888 888Y88888P888    Y888P         .d88P  "Y888b.   888        888Y88888P888 
//		888    888 888 Y888P 888    d888b     .od888P"      "Y88b. 888  88888 888 Y888P 888 
//		888    888 888  Y8P  888   d88888b   d88P"            "888 888    888 888  Y8P  888 
//		888  .d88P 888   "   888  d88P Y88b  888"       Y88b  d88P Y88b  d88P 888   "   888 
//		8888888P"  888       888 d88P   Y88b 888888888   "Y8888P"   "Y8888P88 888       888 
//
//
//***********************************************************************************************                                                                                
//                                                                                    
//								   DMX512 to Dual SGM256 Converter                                                                                   
//										   Version: 1.0
//									  Build date: 26.10.2019
//										  Author: Gabs'e
//										File: DMX2SGM.asm
//								   Target MCU: Microchip ATtiny1634
//										Licence: CC BY-NC-SA 
//
//***********************************************************************************************
//
//	Attiny Pinout:
//					ADDRESS SW = 0-7= PORTA
//					SGM1	= UART0	= PB0
//					DMX		= UART1 = PB1
//					SGM2	= USI	= PB2
//					ADDRESS SW = 8	= PB3
//					LED		=		  PC2
//
//	Fuse Settings: 
//						EXTENDED:	0xF5
//						HIGH:		0xD4
//						LOW:		0xCE
//					
//
//	Changelog:		
//				V1.0..........First Release
//
//
//***********************************************************************************************

.equ F_CPU = 12000000					// CPU Clock


//***************************Register Assignment****************************

.def SGM1DATA		=	R0				// SGM 1 data buffer to store current byte
.def SGM2DATA		=	R1				// SGM 2 data buffer to store current byte
.def DMXDATA		=	R2				// DMX data buffer to store current byte
.def DMXADDRESSL	=	R3				// DIP DMX address LSB
.def DMXADDRESSH	=	R4				// DIP DMX address MSB
.def DMXADDRESSCL	=	R5				// DIP DMX address compare LSB

.def SGM1BIT		=	R16				// SGM 1 bit pair
.def SGM2BIT		=	R17				// SGM 2 bit pair
.def SGM1OUTREG		=	R18				// Buffer for SGM 1 data output
.def SGM2OUTREG		=	R19				// Buffer for SGM 2 data output
.def temp			=	R20				// Temp register for setup and interrupts
.def tempm			=	R21				// Temp register for main loop 
.def DMXSTATUS		=	R22				// Register for DMX Status
.def DMXADDRESSCH	=	R23				// DIP DMX address compare MSB
.def DMXBYTEL		=	R24				// Last DMX Byte number LSB
.def DMXBYTEH		=	R25				// Last DMX Byte number MSB
// SGM1ADDRESS		=	X => R26 & R27	// SGM 1 data RAM read address
// SGM2ADDRESS		=	Y => R28 & R29	// SGM 2 data RAM read address
// DMXADRESS		=	Z => R30 & R31	// DMX data RAM write address


//*****************************DMXSTATUS Flags******************************

.equ DSNB			=	0				// DMX Skip Next Byte
.equ DRSB			=	1				// DMX Read Start Byte
.equ LSWS			=	2				// LED Switch State


//******************************Program Start*******************************

.org 0x0000
	rjmp main									// Jump to MAIN on program start

.org OC1Aaddr
	rjmp TC1CMA									// Jump to TC1CMA on OC1Aaddr interrupt

.org UDRE0addr
	rjmp USART0_OVERFLOW						// Jump to USART0_OVERFLOW on UDRE0addr interrupt

.org URXC1addr
	rjmp USART1_OVERFLOW						// Jump to USART1_OVERFLOW on URXC1addr interrupt

.org USI_OVFaddr
	rjmp USI_OVERFLOW							// Jump to USI_OVERFLOW on USI_OVFaddr interrupt


//*******************************Clear Stack*********************************

main:

	CLR R0										// Clear stack register 0
	CLR R1										// Clear stack register 1
	CLR R2										// Clear stack register 2
	CLR R3										// Clear stack register 3
	CLR R4										// Clear stack register 4
	CLR R5										// Clear stack register 5
	CLR R6										// Clear stack register 6
	CLR R7										// Clear stack register 7
	CLR R8										// Clear stack register 8
	CLR R9										// Clear stack register 9
	CLR R10										// Clear stack register 10
	CLR R11										// Clear stack register 11
	CLR R12										// Clear stack register 12
	CLR R13										// Clear stack register 13
	CLR R14										// Clear stack register 14
	CLR R15										// Clear stack register 15
	CLR R16										// Clear stack register 16
	CLR R17										// Clear stack register 17
	CLR R18										// Clear stack register 18
	CLR R19										// Clear stack register 19
	CLR R20										// Clear stack register 20
	CLR R21										// Clear stack register 21
	CLR R22										// Clear stack register 22
	CLR R23										// Clear stack register 23
	CLR R24										// Clear stack register 24
	CLR R25										// Clear stack register 25
	CLR R26										// Clear stack register 26
	CLR R27										// Clear stack register 27
	CLR R28										// Clear stack register 28
	CLR R29										// Clear stack register 29
	CLR R30										// Clear stack register 30
	CLR R31										// Clear stack register 31


//********************************Clear RAM*********************************

	CLR temp									// Reset temp
	LDI R30, 0x00								// Set Z pointer LSB RAM clear start adress
	LDI R31, 0x02								// Set Z pointer MSB RAM clear start adress
clr_ram:
	ST Z+, temp									// Copy temp to RAM and increment Z pointer
	CPI R31, 0x04								// Set Z pointer RAM clear stop adress
	BRNE clr_ram								// Jump to clr_ram if RAM adress MSB is below 0x04
	

//***********************Definitions for SGM1_USART0************************

	LDI R26, 0x00								// Set X pointer LSB to 0
	LDI R27, 0x02								// Set X pointer MSB to 2
	LDI SGM1BIT,0x00							// Reset SGM1BIT
	LDI temp, 1<<UMSEL00 | 1<<UMSEL01			// Set USART mode to MSPIM
	OUT UCSR0C, temp							// Set USART mode to MSPIM
	LDI temp, 1<<UDRIE0 | 1<<TXEN0				// Enable USART 0 transmitt & TX complete interrupt
	OUT UCSR0B, temp							// Enable USART 0 transmitt & TX complete interrupt	
	LDI temp, HIGH ((F_CPU/(250000*2))-1)		// Set USART 0 baud register MSB to (Clock/(250kHz*2))-1 MSB
	OUT UBRR0H, temp							// Set USART 0 baud register MSB to (Clock/(250kHz*2))-1 MSB
	LDI temp, LOW ((F_CPU/(250000*2))-1)		// Set USART 0 baud register LSB to (Clock/(250kHz*2))-1 LSB
	OUT UBRR0L, temp							// Set USART 0 baud register LSB to (Clock/(250kHz*2))-1 LSB


//************************Definitions for DMX_USART1************************

	LDI temp, 1<<RXCIE1 | 1<<RXEN1				// Enable USART 1 recive & RX interrupt
	STS UCSR1B, temp							// Enable USART 1 recive & RX interrupt
	LDI temp, 1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10	// Set USART 1 to 8 databits & 2 stopbits
	STS UCSR1C, temp							// Set USART 1 to 8 databits & 2 stopbits
	LDI temp, HIGH ((F_CPU/(250000*16))-1)		// Set USART 1 baud register MSB to (Clock/(250kHz*16))-1 MSB
	STS UBRR1H, temp							// Set USART 1 baud register MSB to (Clock/(250kHz*16))-1 MSB
	LDI temp, LOW ((F_CPU/(250000*16))-1)		// Set USART 1 baud register LSB to (Clock/(250kHz*16))-1 LSB
	STS UBRR1L, temp							// Set USART 1 baud register LSB to (Clock/(250kHz*16))-1 LSB
	SBI PUEB, PB1								// Enable DMX Pullup


//*************************Definitions for SGM2_USI*************************
	
	LDI R28, 0x00								// Set Y pointer LSB to 0
	LDI R29, 0x03								// Set Y pointer MSB to 3
	LDI SGM2BIT, 0x00							// Reset SGM2BIT
	LDI temp, 1<<WGM01							// Set waveform generation of Timer/Counter0 to Clear on Compare
	OUT TCCR0A, temp							// Set waveform generation of Timer/Counter0 to Clear on Compare
	LDI temp, 1<<CS00							// Set Timer/Counter0 prescaler to Clock
	OUT TCCR0B, temp							// Set Timer/Counter0 prescaler to Clock
	LDI temp, ((F_CPU/250000)-1) 				// Set Compare register to Count to (Clock/250KHz)-1
	OUT OCR0A,temp								// Set Compare register to Count to (Clock/250KHz)-1 
	LDI temp, 1<<USIOIE | 1<<USIWM0 | 1<<USICS0	// Set USI to 3Wire mode, clock to TC0 Overflow & Interrupt on USI overflow
	OUT USICR, temp								// Set USI to 3Wire mode, clock to TC0 Overflow & Interrupt on USI overflow
	LDI temp, 1<<USIOIF |	10					// Reset USI Overflow Flag & Set USI Counter to 10 to shift interrupts a part
	OUT USISR, temp								// Reset USI Overflow Flag & Set USI Counter to 10 to shift SGM interrupts a part 
	SBI DDRB, PB2								// Set SGM2 pin to Output

//****************************Definitions for LED***************************
 
	LDI temp, 1<<OCIE1A							// Enable Timer/Counter1 Compare Match A interrupt
	OUT TIMSK, temp								// Enable Timer/Counter1 Compare Match A interrupt	
	LDI temp, HIGH (((F_CPU/1024)/33)-1)		// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)
	STS OCR1AH,temp								// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)		
	LDI temp, LOW (((F_CPU/1024)/33)-1)			// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	STS OCR1AL,temp								// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	CLR temp									// Reset Timer/Counter1
	STS TCNT1L, temp							// Reset Timer/Counter1
	STS TCNT1H, temp							// Reset Timer/Counter1
	STS TCCR1A, temp							// Reset Timer/Counter1 Control Register A
	LDI temp, 1<<WGM12 | 1<<CS10 | 1<<CS12		// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	STS TCCR1B, temp							// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	SBI DDRC, PC2								// Set LED pin to output
	SBI PORTC, PC2								// Turn LED ON



//***************************General Definitions****************************

	LDI temp, 0x00								// Set PortA to input 
	OUT DDRA, temp								// Set PortA to input 
	LDI temp, 0xFF								// Enable PortA pullup
	OUT PUEA, temp								// Enable PortA pullup
	CBI DDRB, PB3								// Set Pin PB3 to input
	SBI PUEB, PB3								// Enable Pin PB3 Pullup
	LDI temp, 1<<WDE							// Enable Watchdog Timer @ 16ms
	OUT WDTCSR, temp							// Enable Watchdog Timer @ 16ms
	LDI tempm, 0xFF								// Load temp with value for XOR
	IN DMXADDRESSL, PINB						// Read DMX address bit 8 from DIP switch
	EOR DMXADDRESSL, tempm						// Invert register
	BST DMXADDRESSL, PB3						// Copy DMX address PB3 to T
	BLD DMXADDRESSH, 0							// Copy T to DMXADDRESSH bit 0
	IN DMXADDRESSL, PINA						// Read DMX address bit 0-7 from DIP switch
	EOR DMXADDRESSL, tempm						// Invert register
	SEI											// Enable global interrupts


//********************************main loop*********************************

loop:

	IN DMXADDRESSCH, PINB						// Read DMX address bit 8 from DIP switch
	EOR DMXADDRESSCH, tempm						// Invert register
	ANDI DMXADDRESSCH, 1<<PB3					// Mask out PB3
	LSR DMXADDRESSCH							// Shift DMXADDRESSCH right by 1 possition
	LSR DMXADDRESSCH							// Shift DMXADDRESSCH right by 1 possition
	LSR DMXADDRESSCH							// Shift DMXADDRESSCH right by 1 possition
	IN DMXADDRESSCL, PINA						// Read DMX address bit 0-7 from DIP switch
	EOR DMXADDRESSCL, tempm						// Invert register
	CPSE DMXADDRESSCL, DMXADDRESSL				// Compare new address LSB with set adress
		RJMP loop									// Jump back to loop if address LSB has changed, forcing a watchdog reset
	CPSE DMXADDRESSCH, DMXADDRESSH				// Compare new address MSB with set adress
		RJMP loop									// Jump back to loop if address MSB has changed, forcing a watchdog reset
	WDR											// Reset Watchdog Timer
	RJMP loop									// Jump back to loop
	

//**************************SGM1 Output Interrupt***************************

USART0_OVERFLOW:
	
	LDI SGM1OUTREG, 0x88						// Load startbit to SGM1OUTREG
	CPI SGM1BIT, 0x00							// Check SGM1BIT
		BRNE SGM1DETEND								// If SGM1BIT not 0 jump to SGM1DETEND
	DEC R26										// Decrement X LSB by 1
	LD SGM1DATA, X								// Store RAM adress X to SGM1DATA
	LDI SGM1BIT, 0x04							// Set SGM1BIT

	SGM1DETEND:
		CPI SGM1BIT, 0x01						// Check SGM1BIT
			BRNE SGM1SEND							// If SGM1BIT not 1 jump to SGM1SEND
		CPI R26, 0x00							// Check X LSB
			BRNE SGM1SEND							// If X LSB not 0 jump to SGM1SEND
		LDI SGM1OUTREG, 0x8A					//Load start and sync bit to SGMOUTREG

	SGM1SEND:
		BST SGM1DATA, 7							// Copy SGM2DATA bit 7 to T
		BLD SGM1OUTREG, 6						// Copy T to SGMOUTREG bit 1
		BST SGM1DATA, 6							// Copy SGM2DATA bit 6 to T
		BLD SGM1OUTREG, 2						// Copy T to SGMOUTREG bit 5
		DEC SGM1BIT								// Decrement SGM1BIT
		LSL SGM1DATA							// Shift SGM2DATA left by 1 possition
		LSL SGM1DATA							// Shift SGM2DATA left by 1 possition
		OUT UDR0, SGM1OUTREG					// Output from SGMOUTREG to USART data register 
		RETI									// Exit interrupt


//***************************DMX Input Interrupt****************************

USART1_OVERFLOW:

	LDS temp, UCSR1A							// Load data from UCSR1A to temp register
	LDS	DMXDATA, UDR1							// Load data from USART data register to DMXDATA register
	ST Z, DMXDATA								// Copy DMXDATA to RAM and increment Z pointer
	SBRC temp, DOR1								// Check dataoverrun flag
		RJMP SKIPNEXT								// Jump to SKIPNEXT if dataoverrunn occours
	SBRC temp, FE1								// Chek frameerror flag
		RJMP FRAMEERROR								// Jump to FRAMEERROR if frameerror occours (Marks new DMX package) 
	SBRC DMXSTATUS,	DSNB						// Check DMX Status for Skip next
		RJMP SKIPNEXT							// Jump to SKIPNEXT
	SBRC DMXSTATUS,	DRSB							// Check DMX Status for Startbyte
		RJMP	STARTBYTE							// Jump to STARTBYTE
	CP DMXBYTEL, DMXADDRESSL					// Check DMX address
	CPC DMXBYTEH, DMXADDRESSH					// Check DMX address
		BRLO ADDRESSLOW								// Jump to ADDRESSLOW if DMX byte is below DMX address
	ST Z+, DMXDATA								// Copy DMXDATA to RAM and increment Z pointer

	ADDRESSLOW:
		ADIW DMXBYTEL,1							// Increase DMXBYTE by 1
		LDI DMXSTATUS, 1<<LSWS					// Set DMXSTATUS to switch LED state
		RETI									// Exit Interrupt

	SKIPNEXT:

		LDI DMXSTATUS, 1<<DSNB					// Set DMXSTATUS to skip til next frameerror
		CBR temp, 1<<DOR1							// Clear dataoverrunn flag (Can not be done directly due to ATTiny1634 register address mapping)
		STS UCSR1A,temp							// Set UCSR1A to temp
		RETI									// Exit interrupt

	FRAMEERROR:
		LDI R30, 0x00							// Reset DMXADRESS LSB
		LDI R31, 0x02							// Reset DMXADRESS MSB
		LDI DMXBYTEL, 0x00						// Reset DMXBYTE LSB
		LDI DMXBYTEH, 0x00						// Reset DMXBYTE MSB
		CBR temp, 1<<FE1						// Clear frameerror flag (Can not be done directly due to ATTiny1634 register address mapping)
		STS UCSR1A,temp							// Set UCSR1A to temp
		LDI DMXSTATUS, 1<<DRSB					// Set DMXSTATUS to check startbyte
		RETI									// Exit Interrupt

	STARTBYTE:					
		CPSE DMXDATA, DMXBYTEH					// Check startbit
			RJMP SKIPNEXT							// Jump to SKIPNEXT if startbit is not 0
		LDI DMXSTATUS, 1<<LSWS					// Set DMXSTATUS to Switch LED state
		RETI									// Exit interrupt
		

//**************************SGM2 Output Interrupt***************************

USI_OVERFLOW:

	LDI SGM2OUTREG, 0xFF						// Set USIDR output high
	OUT USIDR, SGM2OUTREG						// Set USIDR output high
	LDI SGM2OUTREG, 0x88						// Load startbits to SGMOUTREG
	CPI SGM2BIT, 0x00							// Check SGM2BIT
		BRNE SGM2DETEND								// If SGM2BIT not 0 jump to SGM2DETEND
	DEC R28										// Decrement Y LSB by 1
	LD SGM2DATA, Y								// Store RAM adress Y to SGM2DATA
	LDI SGM2BIT, 0x04							// Set SGM2BIT

	SGM2DETEND:
		CPI SGM2BIT, 0x01						// Check SGM2BIT
			BRNE SGM2SEND							// If SGM2BIT not 1 jump to SGM2SEND
		CPI R28, 0x00							// Check Y LSB
			BRNE SGM2SEND							// If Y LSB not 0 jump to SGM2SEND
		LDI SGM2OUTREG, 0x8A					//Load start and sync bit to SGMOUTREG

	SGM2SEND:
		BST SGM2DATA, 7							// Copy SGM2DATA bit 7 to T
		BLD SGM2OUTREG,6						// Copy T to SGMOUTREG bit 6
		BST SGM2DATA, 6							// Copy SGM2DATA bit 6 to T
		BLD SGM2OUTREG,2						// Copy T to SGMOUTREG bit 2
		DEC SGM2BIT								// Decrement SGMBIT
		LSL SGM2DATA							// Shift SGM2DATA left by 1 possition
		LSL SGM2DATA							// Shift SGM2DATA left by 1 possition
		OUT USIDR, SGM2OUTREG					// Output from SGMOUTREG to USIDR 
		LDI temp, 1<<USIOIF | 8					// Reset USI Overflow Flag & Set USI Counter to 8
		OUT USISR,temp							// Reset USI Overflow Flag & Set USI Counter to 8
		RETI									// Exit interrupt


//****************Timer/Counter1 Compare Match A Interrupt *****************

TC1CMA:

	SBRS DMXSTATUS,	LSWS						// Check DMX Status for LED switch
		RJMP	LEDON								// Jump to LEDON if LSWS is cleared
	SBIS PINC,	PC2								// Check if LED is OFF
		RJMP	LEDON								// Jump to LEDON if LED is OFF
	CBI PORTC, PC2								// Turn LED OFF
	CBR DMXSTATUS, 1<<LSWS						// Clear LSWS
	RETI										// Exit interrupt

	LEDON:
		SBI PORTC, PC2							// Turn LED ON
		CBR DMXSTATUS, 1<<LSWS					// Clear LSWS
		RETI									// Exit interrupt


//******************************Programm End *******************************