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
//										   Version: 2.0
//									  Build date: 26.10.2019
//										  Author: Gabs'e
//										File: DMX2SGM.asm
//								   Target MCU: Microchip ATtiny1634
//										Licence: CC BY-NC-SA 
//								SGM Protocol Description: https://forum.dmxcontrol-projects.org/core/index.php?attachment/8711-sgm256-de-pdf/
//
//***********************************************************************************************
//
//	Attiny Pinout:
//					ADDRESS SW = 0-7= PORTA
//					SGM1	= UART0	= PB0
//					DMX		= UART1 = PB1
//					SGM2	= USI	= PB2
//					ADDRESS SW = 8	= PB3
//					SGM CLOCK	= PC0&PC1
//					LED		=		  PC2
//
//	Fuse Settings: 
//						EXTENDED:	0xF5
//						HIGH:		0xD4
//						LOW:		0xCF
//					
//
//	Changelog:		
//				V1.0..........First Release
//				V2.0..........Code cleanup, Fixing a few bugs, General code optimations
//
//
//***********************************************************************************************

.equ F_CPU = 12000000							// CPU Clock

//********************************Registers*********************************

.def SGM1DATA		=	R0						// SGM 1 data buffer to store current byte
.def SGM2DATA		=	R1						// SGM 2 data buffer to store current byte
.def DMX_ADDRESS_L	=	R2						// Register for DIP DMX address LSB
.def DMX_ADDRESS_H	=	R3						// Register for DIP DMX address MSB
.def DMX_ADDRESS_L_C=	R4						// Register for DIP DMX address LSB compare value
.def DMX_ADDRESS_H_C=	R5						// Register for DIP DMX address MSB compare value
.def DMX_BYTE_H		=	R6						// Register for last DMX byte number MSB
.def DMX_BYTE_L		=	R7						// Register for last DMX byte number LSB
.def DMX_DATA		=	R8						// Register for current DMX byte
.def S_SREG			=	R9						// Register to restore the SREG after Interrupts
.def S_SREG_D		=	R10						// Register to restore the SREG after DMX Interrupt

.def FLAGS			=	R16						// Register for general status flags
.def SGMOUTREG		=	R17						// Buffer for SGM data output
.def SGM1BIT		=	R18						// SGM 1 bit pair
.def SGM2BIT		=	R19						// SGM 2 bit pair
.def TEMP_I			=	R20						// Register for temporary values during interrupts
.def TEMP			=	R21						// Register for temporary values in the main loop
// SGM1_ADDRESS		=	X => R26 & R27			// Address pointer for SGM 1 interrupt
// SGM2_ADDRESS		=	Y => R28 & R29			// Address pointer for SGM 2 interrupt
// DMX_ADDRESS		=	Z => R30 & R31			// Address pointer for DMX interrupt


//******************************General FLAGS*******************************

.equ DSNB			=	0						// DMX Skip Next Byte
.equ DRSB			=	1						// DMX Read Start Byte
.equ LEDT			=	2						// LED Toggle


//*****************************Interrupt Vectors*****************************

.org 0x0000
	RJMP MAIN									// Jump to MAIN on program start

.org OC1Aaddr
	RJMP TC1CMA									// Jump to TC1CMA on OC1Aaddr interrupt

.org UDRE0addr
	RJMP SGM1_INTERRUPT							// Jump to SGM1_INTERRUPT on UDRE0addr interrupt

.org URXC1addr
	RJMP DMX_INTERRUPT							// Jump to DMX_INTERRUPT on URXC1addr interrupt

.org USI_OVFaddr
	RJMP SGM2_INTERRUPT							// Jump to SGM2_INTERRUPT on USI_OVFaddr interrupt


//*****************************Init Controller******************************
main:

	// Clear work registers
	CLR R0										// Clear register 0
	CLR R1										// Clear register 1
	CLR R2										// Clear register 2
	CLR R3										// Clear register 3
	CLR R4										// Clear register 4
	CLR R5										// Clear register 5
	CLR R6										// Clear register 6
	CLR R7										// Clear register 7
	CLR R8										// Clear register 8
	CLR R9										// Clear register 9
	CLR R10										// Clear register 10
	CLR R11										// Clear register 11
	CLR R12										// Clear register 12
	CLR R13										// Clear register 13
	CLR R14										// Clear register 14
	CLR R15										// Clear register 15
	CLR R16										// Clear register 16
	CLR R17										// Clear register 17
	CLR R18										// Clear register 18
	CLR R19										// Clear register 19
	CLR R20										// Clear register 20
	CLR R21										// Clear register 21
	CLR R22										// Clear register 22
	CLR R23										// Clear register 23
	CLR R24										// Clear register 24
	CLR R25										// Clear register 25
	CLR R26										// Clear register 26
	CLR R27										// Clear register 27
	CLR R28										// Clear register 28
	CLR R29										// Clear register 29
	CLR R30										// Clear register 30
	CLR R31										// Clear register 31

	// Init Stack Pointer
	LDI	TEMP,	LOW(RAMEND)						// Init StackPointer to end of RAM
	OUT	SPL,	TEMP							// Init StackPointer to end of RAM
	LDI	TEMP,	HIGH(RAMEND)					// Init StackPointer to end of RAM
	OUT	SPH,	TEMP							// Init StackPointer to end of RAM

	// Init RAM
	CLR TEMP									// Clear TEMP
	LDI ZL,	LOW(SRAM_START)						// Set Z pointer LSB to Start of RAM
	LDI ZH,	HIGH(SRAM_START)					// Set Z pointer MSB to Start of RAM

clr_ram:
	ST Z+,	TEMP								// Copy TEMP to RAM and increment Z pointer
	CPI ZL,	LOW(RAMEND+1)						// Check if End of RAM reached
		BRNE clr_ram								// Jump to clr_ram if End of RAM was not reached
	CPI ZH,	HIGH(RAMEND+1)						// Check if End of RAM reachedd
		BRNE clr_ram								// Jump to clr_ram if End of RAM was not reached
	

//**************************Init USART0 for SGM1****************************

	LDI XL, 0x00								// Set X pointer LSB to 0
	LDI XH, 0x01								// Set X pointer MSB to 1
	CLR SGM1BIT									// Reset SGM1BIT
	LDI TEMP, 1<<UMSEL00 | 1<<UMSEL01			// Set USART mode to MSPIM
	OUT UCSR0C, TEMP							// Set USART mode to MSPIM
	LDI TEMP, 1<<UDRIE0 | 1<<TXEN0				// Enable USART 0 transmitt & TX complete interrupt
	OUT UCSR0B, TEMP							// Enable USART 0 transmitt & TX complete interrupt	
	LDI TEMP, HIGH ((F_CPU/(250000*2))-1)		// Set USART 0 baud register MSB to 250kHz
	OUT UBRR0H, TEMP							// Set USART 0 baud register MSB to 250kHz
	LDI TEMP, LOW ((F_CPU/(250000*2))-1)		// Set USART 0 baud register LSB to 250kHz
	OUT UBRR0L, TEMP							// Set USART 0 baud register LSB to 250kHz
	SBI DDRC,	 PC0							// Enable clock output
	
	
	
//*****************************Init USI for SGM2****************************
	
	
	LDI YL, 0x00								// Set Y pointer LSB to 0
	LDI YH, 0x02								// Set Y pointer MSB to 2
	CLR SGM2BIT									// Reset SGM2BIT
	LDI TEMP,	1<<USIOIE | 1<<USIWM0 | 1<<USICS1// Set USI to 3Wire mode, clock from PC1 & interrupt on USI overflow
	OUT USICR,	TEMP							// Set USI to 3Wire mode, clock from PC1 & interrupt on USI overflow
	CLR TEMP									// Set USI output to 0
	OUT USIDR, TEMP								// Set USI output to 0
	LDI TEMP, 1<<USIOIF |	10					// Reset USI Overflow Flag & Set USI Counter to 10 to shift SGM interrupts a part 
	OUT USISR, TEMP								// Reset USI Overflow Flag & Set USI Counter to 10 to shift SGM interrupts a part 
	SBI DDRB, PB2								// Set SGM2 pin to Output


//****************************Init USART1 for DMX***************************

	LDI TEMP,	1<<RXCIE1 | 1<<RXEN1			// Enable USART 1 recive & RX interrupt
	STS UCSR1B, TEMP							// Enable USART 1 recive & RX interrupt
	LDI TEMP,	1<<USBS1 | 1<<UCSZ11 | 1<<UCSZ10// Set USART 1 to 8 databits & 2 stopbits
	STS UCSR1C,	TEMP							// Set USART 1 to 8 databits & 2 stopbits
	LDI TEMP,	HIGH ((F_CPU/(250000*16))-1)	// Set USART 1 baud register MSB to 250kHz
	STS UBRR1H, TEMP							// Set USART 1 baud register MSB to 250kHz
	LDI TEMP,	LOW ((F_CPU/(250000*16))-1)		// Set USART 1 baud register LSB to 250kHz
	STS UBRR1L, TEMP							// Set USART 1 baud register LSB to 250kHz
	SBI PUEB,	PB1								// Enable DMX pullup


//******************************Init Timer for LED**************************

	LDI TEMP,	1<<OCIE1A						// Enable Timer/Counter1 Compare Match A interrupt
	OUT TIMSK,	TEMP							// Enable Timer/Counter1 Compare Match A interrupt	
	LDI TEMP,	HIGH (((F_CPU/1024)/33)-1)		// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)
	STS OCR1AH,	TEMP							// Set Timer/Counter1 Compare Register A MSB to about 30ms (33Hz)		
	LDI TEMP,	LOW (((F_CPU/1024)/33)-1)		// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	STS OCR1AL,	TEMP							// Set Timer/Counter1 Compare Register A LSB to about 30ms (33Hz)
	CLR TEMP									// Reset Timer/Counter1
	STS TCNT1L, TEMP							// Reset Timer/Counter1
	STS TCNT1H, TEMP							// Reset Timer/Counter1
	STS TCCR1A, TEMP							// Reset Timer/Counter1 Control Register A
	LDI TEMP,	1<<WGM12 | 1<<CS10 | 1<<CS12	// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	STS TCCR1B, TEMP							// Set Timer/Counter1 Control Register B to Clear Timer on Compare A & Clock/1024
	SBI DDRC,	PC2								// Set LED pin to output
	SBI PORTC,	PC2								// Turn LED ON


//******************************General Init********************************
	
	LDI TEMP,	1<<WDE							// Enable Watchdog Timer @ 16ms
	OUT WDTCSR,	TEMP							// Enable Watchdog Timer @ 16ms
	LDI TEMP,	0x00							// Set PortA to input 
	OUT DDRA,	TEMP							// Set PortA to input 
	LDI TEMP,	0xFF							// Enable PortA pullup
	OUT PUEA,	TEMP							// Enable PortA pullup
	CBI DDRB,	PB3								// Set pin PB3 to input
	SBI PUEB,	PB3								// Enable pin PB3 pullup
	SER TEMP									// Load TEMP with value for XOR
	IN DMX_ADDRESS_L,	PINB					// Read DMX address bit 8 from DIP switch
	EOR DMX_ADDRESS_L,	TEMP					// Invert register
	BST DMX_ADDRESS_L,	PB3						// Copy DMX address PB3 to T
	BLD DMX_ADDRESS_H,	0						// Copy T to DMX_ADDRESS_H bit 0
	IN DMX_ADDRESS_L,	PINA					// Read DMX address bit 0-7 from DIP switch
	EOR DMX_ADDRESS_L,	TEMP					// Invert register
	LDI SGMOUTREG, 0x88							// Load startbit to SGMOUTREG
	SEI											// Enable global interrupts


//********************************main loop*********************************
loop:

		// Check if DIP switch address has changed, and force a device reset by not reseting the watchdog if address has changed.
		IN DMX_ADDRESS_L_C,	PINB				// Read DMX address bit 8 from DIP switch
		EOR DMX_ADDRESS_L_C,	TEMP			// Invert register
		BST DMX_ADDRESS_L_C,	PB3				// Copy DMX address PB3 to T
		BLD DMX_ADDRESS_H_C,	0				// Copy T to DMX_ADDRESS_H_C bit 0
		IN DMX_ADDRESS_L_C,	PINA				// Read DMX address bit 0-7 from DIP switch
		EOR DMX_ADDRESS_L_C,	TEMP			// Invert register
		CPSE DMX_ADDRESS_L_C,	DMX_ADDRESS_L	// Compare new address LSB with set address
			RJMP loop								// Jump back to loop if address LSB has changed, forcing a device reset
		CPSE DMX_ADDRESS_H_C,	DMX_ADDRESS_H	// Compare new address MSB with set address
			RJMP loop								// Jump back to loop if address MSB has changed, forcing a device reset
		WDR										// Reset watchdog timer
		RJMP loop								// Jump back to loop


//****************Timer/Counter1 Compare Match A Interrupt*****************
TC1CMA:
	
		// Check if DMX has updated since the last Interrupt, and flash the LED if update has ocoured. Turn the LED ON otherwise.	
		IN S_SREG,	SREG						// Save SREG
		SBI PINC,	PC2							// Toggle LED
		SBRS FLAGS,	LEDT						// Check DMX status for LED toggle
			SBI PORTC,	PC2							// Turn LED ON
		CBR FLAGS,	1<<LEDT						// Clear LEDT flag
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt


//***************************DMX Input Interrupt****************************
DMX_INTERRUPT:

		// Get Data from the UART, check if special DMX condition has ocoured. Save the recived DMX Data to RAM starting from 0x0100 with the first DMX channel selected by the DIP Switch. Toggle the LED if valid DMX Frames are recived.
		LDS TEMP_I,	UCSR1A						// Load data from UCSR1A to TEMP_I register
		LDS	DMX_DATA,	UDR1					// Load data from USART data register to DMX_DATA register
		IN S_SREG_D,	SREG					// Save SREG
		SEI										// Allow interrupts with higher priority to occour
		SBRC TEMP_I,	DOR1					// Check dataoverrun flag
			RJMP SKIPNEXT							// Jump to SKIPNEXT if dataoverrunn occoured (drop current DMX frame, as dataoverruns should never happen)
		SBRC TEMP_I,	FE1						// Chek frameerror flag
			RJMP FRAMEERROR							// Jump to FRAMEERROR if frameerror occoured (marks the beginning of a new DMX frame) 
		SBRS FLAGS,	DSNB						// Check DMX status for skip next flag
			RJMP SKIPNEXT							// Jump to SKIPNEXT
		SBRC FLAGS,	DRSB						// Check DMX status for startbyte
			RJMP STARTBYTE							// Jump to STARTBYTE
		SBR FLAGS,	1<<LEDT						// Set flag to toggle the LED
		CP DMX_BYTE_L,	DMX_ADDRESS_L			// Check DMX address MSB
		CPC DMX_BYTE_H,	DMX_ADDRESS_H			// Check DMX address LSB
			BRLO ADDRESSLOW							// Jump to ADDRESSLOW if DMX byte is below DMX address 
		SBRC DMX_BYTE_H,	1					// Check if all 512 bytes have been recived
			RJMP SKIPNEXT							// Jump to SKIPNEXT if all 512 bytes have been recived. (should never happen to recive more than 512 bytes)
		ST Z+,	DMX_DATA						// Copy DMX_DATA to RAM and increment Z pointer

	ADDRESSLOW:
		// Increment the DMX_BYTE
		INC DMX_BYTE_L							// Increment DMXBYTE L by 1
			BRNE NINCDMXH							// Jump to NINCDMXH when DMX_BYTE_L hasn't overflown
		INC DMX_BYTE_H							// Increment DMXBYTE H by 1
	NINCDMXH:
		OUT SREG,	S_SREG_D					// Restore SREG
		RETI									// Exit Interrupt

	SKIPNEXT:
		// Drop recived data until next DMX Frame starts
		CBR FLAGS,	1<<DSNB						// Clear FLAGSATUS to skip til next frameerror
		OUT SREG,	S_SREG_D					// Restore SREG
		RETI									// Exit interrupt

	FRAMEERROR:
		// Reset X Pointer & DMX_BYTE counter. Set DMX status to check the startbyte on the next interrupt
		LDI ZL,	0x00							// Reset DMXADRESS LSB
		LDI ZH, 0x01							// Reset DMXADRESS MSB
		CLR DMX_BYTE_L							// Reset DMXBYTE LSB
		CLR DMX_BYTE_H							// Reset DMXBYTE MSB
		SBR FLAGS,	1<<DRSB	| 1<<DSNB			// Set DMX Skip Next Byte & DMX Read Start Byte flag to check startbyte on next interrupt
		OUT SREG,	S_SREG_D					// Restore SREG
		RETI									// Exit Interrupt

	STARTBYTE:
		// Check if recived startbit is valid (zero). Otherwise drop the incoming DMX Frame.
		CPSE DMX_DATA,	DMX_BYTE_H				// Check startbit
			RJMP SKIPNEXT							// Jump to SKIPNEXT if startbit is not 0
		CBR FLAGS,	1<<DRSB						// Clear DMX Read Start Byte flag
		OUT SREG,	S_SREG_D					// Restore SREG
		RETI									// Exit interrupt
		
			
//**************************SGM1 Output Interrupt***************************
SGM1_INTERRUPT:

		// Check for a ongoing SGM transmition. If no ongoing transmistion is present get new data from RAM
		IN S_SREG,	SREG						// Save SREG
		CPI SGM1BIT,	0x00					// Check SGM1BIT
			BRNE SGM1DETEND							// If SGM1BIT not 0 jump to SGM1DETEND
		DEC XL									// Decrement X LSB by 1
		LD SGM1DATA,	X						// Get data from RAM @  X
		LDI SGM1BIT,	0x04					// Set SGM1BIT

	SGM1DETEND:
		// Check for SGM end of frame and set sync bit
		CPI SGM1BIT,	0x01					// Check SGM1BIT
			BRNE SGM1SEND							// If SGM1BIT not 1 jump to SGM1SEND
		CPI XL,	0x00							// Check X LSB
			BRNE SGM1SEND							// If X LSB not 0 jump to SGM1SEND
		LDI SGMOUTREG,	0x8A					//Load start and sync bit to SGMOUTREG

	SGM1SEND:
		// Prepare and output SGM data
		BST SGM1DATA,	7						// Copy SGM2DATA bit 7 to T
		BLD SGMOUTREG,	6						// Copy T to SGMOUTREG bit 1
		BST SGM1DATA,	6						// Copy SGM2DATA bit 6 to T
		BLD SGMOUTREG,	2						// Copy T to SGMOUTREG bit 5
		DEC SGM1BIT								// Decrement SGM1BIT
		LSL SGM1DATA							// Shift SGM2DATA left by 1 possition
		LSL SGM1DATA							// Shift SGM2DATA left by 1 possition
		OUT UDR0,	SGMOUTREG					// Output from SGMOUTREG to USART data register
		OUT SREG,	S_SREG						// Restore SREG
		LDI SGMOUTREG,	0x88					// Load startbit to SGMOUTREG for next Interrupt
		RETI									// Exit interrupt


//**************************SGM2 Output Interrupt***************************
SGM2_INTERRUPT:

		// Check for a ongoing SGM transmition. If no ongoing transmistion is present get new data from RAM
		OUT USIDR,	SGMOUTREG					// Preload startbyte to USIDR
		IN S_SREG,	SREG						// Save SREG
		CPI SGM2BIT,	0x00					// Check SGM2BIT
			BRNE SGM2DETEND							// If SGM2BIT not 0 jump to SGM2DETEND
		DEC YL									// Decrement Y LSB by 1
		LD SGM2DATA,	Y						// Get data from RAM @ Y
		LDI SGM2BIT,	0x04					// Set SGM2BIT

	SGM2DETEND:
		// Check for SGM end of frame and set sync bit
		CPI SGM2BIT,	0x01					// Check SGM2BIT
			BRNE SGM2SEND							// If SGM2BIT not 1 jump to SGM2SEND
		CPI YL,	0x00							// Check Y LSB
			BRNE SGM2SEND							// If Y LSB not 0 jump to SGM2SEND
		LDI SGMOUTREG,	0x8A					//Load start and sync bit to SGMOUTREG

	SGM2SEND:
		// Prepare and output SGM data. Correct clock phase to rising edge.
		BST SGM2DATA,	7						// Copy SGM2DATA bit 7 to T
		BLD SGMOUTREG,	6						// Copy T to SGMOUTREG bit 1
		BST SGM2DATA,	6						// Copy SGM2DATA bit 6 to T
		BLD SGMOUTREG,	2						// Copy T to SGMOUTREG bit 5
		DEC SGM2BIT								// Decrement SGM1BIT
		LSL SGM2DATA							// Shift SGM2DATA left by 1 possition
		LSL SGM2DATA							// Shift SGM2DATA left by 1 possition
		OUT USIDR,	SGMOUTREG					// Output from SGMOUTREG to USART data register
		LDI SGMOUTREG,	1<<USIOIF | 0			// Reset USI Overflow Flag & Set USI Counter to 0
		SBIS PINC,	PC1							// Check clock phase
			LDI SGMOUTREG,	1<<USIOIF | 1			// Correct clock phase (Interrupt on rising edge, output on falling edge)
		OUT USISR,	SGMOUTREG					// Output to USISR
		LDI SGMOUTREG,	0x88					// Load startbit to SGMOUTREG for next Interrupt
		OUT SREG,	S_SREG						// Restore SREG
		RETI									// Exit interrupt
		
//******************************Programm End *******************************

.org FLASHEND-9
.db "DMX2SGM V2 by Gabs'e"

//********************************File End *********************************