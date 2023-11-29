            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Name:  Noah Pangilinan
;Date:  10/17/23
;Class:  CMPE-250
;Section:  L4, Thursday, 2:00
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates

;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
MAX_STRING	EQU	 0x4F	

;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX EQU (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->10:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK EQU (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0-> 16:UART0 open drain enable (disabled)
; 0-> 02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR EQU (SIM_SOPT5_UART0ODE_MASK :OR: SIM_SOPT5_UART0RXSRC_MASK :OR: SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
; 0-> 7:LIN break detect IE (disabled)
; 0-> 6:RxD input active edge IE (disabled)
; 0-> 5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600 EQU 0x01
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDL_9600 EQU 0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select
; (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=paR0ty enable (disabled)
;0-->0:PT=paR0ty type (even paR0ty--no effect PE=0)
UART0_C1_8N1 EQU 0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:R0E=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
; 10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
; 10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
; (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:OR0E=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=paR0ty error IE for PF (disabled)
UART0_C3_NO_TXINV EQU 0x00
;---------------------------------------------------------------
;UART0_C4
; 0--> 7:MAEN1=match address mode enable 1 (disabled)
; 0--> 6:MAEN2=match address mode enable 2 (disabled)
; 0--> 5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
; = 1 + OSR for 3 <= OSR <= 31
; = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16 EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16
UART0_S1_CLEAR_FLAGS EQU (UART0_S1_IDLE_MASK :OR: UART0_S1_OR_MASK :OR: UART0_S1_NF_MASK :OR: UART0_S1_FE_MASK :OR: UART0_S1_PF_MASK)
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;UART0_C5
; 0--> 7:TDMAE=transmitter DMA enable (disabled)
; 0--> 6:Reserved; read-only; always 0
; 0--> 5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
; 0--> 1:BOTHEDGE=both edge sampling (R0sing edge only)
; 0--> 0:RES
;****************************************************************
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
			EXPORT	PutChar
			IMPORT  LengthStringSB
			IMPORT  Startup

Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
				CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
		    BL		Init_UART0_Polling
			LDR		R0,=STRING		;resets the string when program starts
			MOVS	R1,#NULL		;
			STRB	R1,[R0,#0]		;
putprompt	
			MOVS	R2,#0			;beginging of main loop

			
			LDR		R0,=prompt		;loads and prints prompt
			MOVS	R1,#MAX_STRING
			BL		PutStringSB		
char		BL		GetChar			;gets char as an input
			MOVS	R6,R0
			MOVS	R5,#0x60		;checks if it is uppercase
			CMP 	r0,R5		
        	BLO		tolowercase		;branches to make any uppercase letters lowercase
alllower							
			CMP 	r0,#'g'
			BEQ		g				;checks for the letter g
			CMP 	r0,#'l'
			BEQ		l				;checks for the letter l
			CMP 	r0,#'i'
			BEQ		i				;checks for the letter i
			CMP 	r0,#'p'
			BEQ		p				;checks for the letter p
			B		char
;###############
capitalg
			MOVS	R0,R2
			B		capitalgr	
			
capitall
			MOVS	R0,R2
			B		capitallr
			
capitali
			MOVS	R0,R2
			B		capitalir
capitalp
			MOVS	R0,R2
			B		capitalpr
;Stay here	
g				
			CMP		R2,#0		
			BGT		capitalg	; Comes here if the letter is g
capitalgr	BL		PutChar		;prints to confirm input
			BL		newline		;goes to next line
			LDR		R0,=lthan	;prints the "<" 
			BL		PutStringSB
			
			LDR		R0,=STRING
			MOVS	R1,#MAX_STRING
			
			BL		GetStringSB	;takes a string as input and stores it in memory	
			BL		newline		;newline
			B		putprompt	;restarts main loop

l
			CMP		R2,#0	
			BGT		capitall		;if a capital was entered, print a capital 
capitallr	BL		PutChar			; Comes here if the letter is l
			BL		newline			;new line
			LDR		R0,=length	
			BL		PutStringSB		;prints "Length:"
			LDR		R0,=STRING
			MOVS	R1,#MAX_STRING
			BL		LengthStringSB	;gets the length of the string and puts it in R0
			BL		PutNumU			;returns the length of the string 
			BL		newline			;new line
			B		putprompt		;repeats main loop
					
i
			CMP		R2,#0
			BGT		capitali		;if a capital was entered, prints a capital
capitalir	BL		PutChar			; Comes here if the letter is i
			LDR		R0,=STRING		
			MOVS	R1,#NULL
			STRB	R1,[R0,#0]		;stores null into the string
			BL		newline
			B		putprompt		;restarts main loop


p; Comes here if the letter is p
			CMP		R2,#0
			BGT		capitalp		;prints a capital if one was entered
capitalpr	BL		PutChar			
			BL		newline			;new line
			LDR		R0,=gthan
			BL		PutStringSB		;prints ">"
			LDR		R0,=STRING		
			MOVS	R1,#MAX_STRING
			
			BL		PutStringSB	;prints whatever string is currently stored
			LDR		R0,=gthan		;prints ">"
			BL		PutStringSB
			BL		newline			;new line
			B		putprompt		;restarts loop
			
tolowercase
			MOVS	R2,R0			
			ADDS 	r0,#0x20		;adds ox20 to ascii value and stores the old value
			B		alllower
		
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<

Init_UART0_Polling

   ;Select MCGFLLCLK as UART0 clock source
		;Select MCGFLLCLK as UART0 clock source
		LDR R0,=SIM_SOPT2
		LDR R1,=SIM_SOPT2_UART0SRC_MASK
		LDR R2,[R0,#0]
		BICS R2,R2,R1
		LDR R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
		ORRS R2,R2,R1
		STR R2,[R0,#0]
;Set UART0 for external connection
		LDR R0,=SIM_SOPT5
		LDR R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
		LDR R2,[R0,#0]
		BICS R2,R2,R1
		STR R2,[R0,#0]
;Enable UART0 module clock
		LDR R0,=SIM_SCGC4
		LDR R1,=SIM_SCGC4_UART0_MASK
		LDR R2,[R0,#0]
		ORRS R2,R2,R1
		STR R2,[R0,#0]
;Enable PORT B module clock
		LDR R0,=SIM_SCGC5
		LDR R1,=SIM_SCGC5_PORTB_MASK
		LDR R2,[R0,#0]
		ORRS R2,R2,R1
		STR R2,[R0,#0]
;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
		LDR R0,=PORTB_PCR2
		LDR R1,=PORT_PCR_SET_PTB2_UART0_RX
		STR R1,[R0,#0]
; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
		LDR R0,=PORTB_PCR1
		LDR R1,=PORT_PCR_SET_PTB1_UART0_TX
		STR R1,[R0,#0]
;Disable UART0 receiver and transmitter
		LDR R0,=UART0_BASE
		MOVS R1,#UART0_C2_T_R
		LDRB R2,[R0,#UART0_C2_OFFSET]
		BICS R2,R2,R1
		STRB R2,[R0,#UART0_C2_OFFSET]
;Set UART0 for 9600 baud, 8N1 protocol
		MOVS R1,#UART0_BDH_9600
		STRB R1,[R0,#UART0_BDH_OFFSET]
		MOVS R1,#UART0_BDL_9600
		STRB R1,[R0,#UART0_BDL_OFFSET]
		MOVS R1,#UART0_C1_8N1
		STRB R1,[R0,#UART0_C1_OFFSET]
		MOVS R1,#UART0_C3_NO_TXINV
		STRB R1,[R0,#UART0_C3_OFFSET]
		MOVS R1,#UART0_C4_NO_MATCH_OSR_16
		STRB R1,[R0,#UART0_C4_OFFSET]
		MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
		STRB R1,[R0,#UART0_C5_OFFSET]
		MOVS R1,#UART0_S1_CLEAR_FLAGS
		STRB R1,[R0,#UART0_S1_OFFSET]
		MOVS R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
		STRB R1,[R0,#UART0_S2_OFFSET]
;Enable UART0 receiver and transmitter
		MOVS R1,#UART0_C2_T_R
		STRB R1,[R0,#UART0_C2_OFFSET]
		BX	 LR
;###############################################################################################			
PutChar
		PUSH  {R1,R2,R3}
;Poll TDRE until UART0 ready to transmit
		LDR R1,=UART0_BASE
		MOVS R2,#UART0_S1_TDRE_MASK
PollTx 		LDRB R3,[R1,#UART0_S1_OFFSET]
		ANDS R3,R3,R2
		BEQ PollTx
;Transmit character stored in R0
		STRB R0,[R1,#UART0_D_OFFSET]
		POP	  {R1,R2,R3}
		BX	 LR
;###############################################################################################			
			;Poll RDRF until UART0 ready to receive
GetChar	PUSH  {R1,R2,R3}
		LDR R1,=UART0_BASE
		MOVS R2,#UART0_S1_RDRF_MASK
PollRx 	LDRB R3,[R1,#UART0_S1_OFFSET]
		ANDS R3,R3,R2
		BEQ PollRx
;Receive character and store in R0
		LDRB R0,[R1,#UART0_D_OFFSET]
		POP	  {R1,R2,R3}
		BX	 LR
		

;###############################################################################################
;Takes R0, R1
;R0 = address
;R1 = max string length
;the subroutine stores the string entered into the address ar R0
GetStringSB	PROC	{R0-R14}	;gets char in a while loop and puts every char
		
		PUSH    {LR,R3,R1}
		MOVS	R3,R0
		MOVS	R4,R1
		SUBS	R4,R4,#1	;max string - 1 to account for null char end	
		MOVS	R1,#0		;counter var
whilegetstring
		BL		GetChar		;gets char input
		CMP		R1,R4
		BGE		maxstringreached	;checks for max string
		CMP 	R0,#CR
		BEQ		exitgetstring
		
		CMP		R0,#NULL			;checks for null
		BLE		nullbranch		
		
		BL		PutChar				;prints the char
		STRB	R0,[R3,R1]			;stores the char in order
		ADDS	R1,R1,#1			;increments counter
		B		whilegetstring		;restarts loop
		
nullbranch					;move cursor if null char
		MOVS	R0,#LF
		BL		PutChar		;puts a space 
		B		whilegetstring
		
maxstringreached
		BL		GetChar	;if max string is hit, take chars until the enter key is pressed, but dont store 	
		CMP 	R0,#CR
		BEQ		exitgetstring
		B		maxstringreached
exitgetstring				
		MOVS	R0,#NULL		;adds null termination char
		STRB	R0,[R3,R1]		;stores null
		POP	{PC,R3,R1}			;returns to main
		ENDP
		
		
;###############################################################################################
;###############################################################################################
;Takes R0, R1
;R0 = address
;R1 = max string length
;the subroutine prints the string entered into the address at R0
PutStringSB		PROC	{R0-R14}		;calls put char in a while loop until null char
		PUSH	{LR,R1,R2}
		MOVS	R2,R0
		MOVS	R1,#0	;counter
		
whileputstring
		LDRB	R0,[R2,R1]	;loads current char
		CMP 	R0,#NULL	;checks for null
		BEQ		exitputstring	
		CMP 	R1,#MAX_STRING	;checks for max
		BEQ		exitputstring	
		BL		PutChar			;prints current char
		ADDS	R1,R1,#1		;increments counter
		B		whileputstring	;loops
		
exitputstring
		POP	{PC,R1,R2}			;returns to main
		ENDP
;###############################################################################################		
divu		PROC	{R4-R14} ;unused registers
			PUSH	{R2,R3}
			MOVS	R2,#0
			CMP		R0,R2 ;checks if dividing by zero
			BEQ		divbyzero
			
while		CMP		R1,R0
			BLO		ends
			SUBS	R1,R1,R0
			ADDS	R2,R2,#1
			B		while		
ends
			;move quotient from R2 to R0
			MOVS	R0,R2
			;C flag cleared
			MRS		R2,APSR
			LDR		R3,=0x20000000
			BICS	R2,R2,R3
			MSR		APSR,R2
			B		exitdivu
divbyzero
			;set C flag
			MRS		R2,APSR
			LDR		R3,=0x20000000
			ORRS	R2,R2,R3
			MSR		APSR,R2
exitdivu
			POP		{R2,R3}
			BX		LR
			ENDP
;###############################################################################################
;###############################################################################################
;Takes R0
;R0 = length in hex
;the subroutine prints the length entered into the R0 in decimal
PutNumU		PROC	{R0-R14}
			PUSH	{LR,R3}
			MOVS	R3,#0		;counter
whileputnum	MOVS	R1,R0		
			MOVS	R0,#10		;divides by 10
			BL		divu
			MOVS	R2,R0
			MOVS	R0,R1
			ADDS	R0,R0,#'0'	;adds ascii value for '0' to get real m=number
			
			PUSH	{R0}		;pushes value onto stack to get correct order
			ADDS	R3,R3,#1	;increments counter
			
			CMP		R2,#0		;compares quotient to 0
			BEQ		exitputnum
			MOVS	R0,R2
			B		whileputnum	;goes back to division loop
exitputnum	
			
Reverse		
			CMP		R3,#0		;checks counter
			BEQ		exitfr
			POP		{R0}		;pops r0 value and prints it
			BL		PutChar
			SUBS	R3,R3,#1	;decrements counter 	
			B		Reverse		;restarts pop loop
exitfr		POP		{PC,R3}		;exits 
			ENDP
			
;###################################
;prints a new line
newline	
			PUSH 	{LR,R0,R1}
			MOVS	R0,#CR		 
			BL		PutChar		
			MOVS	R0,#LF
			BL		PutChar
			POP		{PC,R0,R1}
			
			
		

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
prompt		DCB		"Type a string command (G,I,L,P):",NULL
length		DCB		"Length:",NULL

gthan		DCB		">",NULL
lthan		DCB		"<",NULL
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
STRING		SPACE 	MAX_STRING
;>>>>>   end variables here <<<<<
            ALIGN
            END
