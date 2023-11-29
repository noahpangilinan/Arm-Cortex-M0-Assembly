;****************************************************************
;Name:  Noah Pangilinan
;Date:  10/26/23
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
WORDNUM		EQU	 0x03	
;****************************************************************
;EQUates
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue buffer contents
Q_REC_SZ    EQU   500  ;Queue management record
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'

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
			IMPORT  LengthStringSB
			EXPORT  PutChar
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
putprompt	
			LDR		R0,=prompt
			BL		PutStringSB
			LDR		R0,=Add1
			MOVS	R1,#WORDNUM
cont		BL		GetHexIntMulti
			BCS		fail
			LDR		R0,=prompt2
			BL		PutStringSB
			LDR		R0,=Add2
			MOVS	R1,#WORDNUM
cont2		BL		GetHexIntMulti
			BCS		fail2
			
			
			LDR		R0,=printsum
			BL		PutStringSB
			LDR		R0,=Sum
			LDR		R1,=Add1
			LDR		R2,=Add2
			MOVS	R3,#WORDNUM
			BL		AddIntMulitU
			BL		PutHexIntMulti
			BL		newline
			
			
			B		putprompt

			
fail		BL		newline
			LDR		R0,=failure
			BL		PutStringSB
			LDR		R0,=Add1

			B		cont
			
fail2		BL		newline
			LDR		R0,=failure
			BL		PutStringSB
			LDR		R0,=Add2

			B		cont2


	

					


			

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
GetStringSB					;gets char in a while loop and puts every char
		PUSH    {LR,R3,R1}
		MOVS	R3,R0
		MOVS	R4,R1
		SUBS	R4,R4,#1
		MOVS	R1,#0
whilegetstring
		BL		GetChar
		CMP		R1,R4
		BGE		maxstringreached
		CMP 	R0,#CR
		BEQ		exitgetstring
		
		CMP		R0,#NULL
		BLE		nullbranch		
		
		BL		PutChar
		STRB	R0,[R3,R1]
		ADDS	R1,R1,#1
		B		whilegetstring
		
nullbranch					;move cursor if null char
		MOVS	R0,#LF
		BL		PutChar
		B		whilegetstring
		
maxstringreached
		BL		GetChar
		CMP 	R0,#CR
		BEQ		exitgetstring
		B		maxstringreached
exitgetstring				
		MOVS	R0,#NULL
		STRB	R0,[R3,R1]
		POP	{PC,R3,R1}
		
		
;###############################################################################################
PutStringSB				;calls put char in a while loop until null char
		PUSH	{LR,R1,R2}
		MOVS	R2,R0
		MOVS	R1,#0
		
whileputstring
		LDRB	R0,[R2,R1]
		CMP 	R0,#NULL
		BEQ		exitputstring
		CMP 	R1,#MAX_STRING
		BEQ		exitputstring
		BL		PutChar
		ADDS	R1,R1,#1
		B		whileputstring
		
exitputstring
		POP	{PC,R1,R2}
		
;###############################################################################################		
newline	
			PUSH 	{LR,R0,R1}
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			POP		{PC,R0,R1}
			BX		LR
			
;######################################
;R0 = addy
;R1 = n
				
PutHexIntMulti	PROC	{R1-R14}	
				PUSH	{LR,R1,R2,R3,R4}
				BCC		nooverflow
				LDR		R0,=overflow
nooverflow		BL		PutStringSB
				POP		{PC,R1,R2,R3,R4}
				ENDP
				
;###########################################
PutNumHex	
				PUSH	{LR,R1,R2,R3,R4}
				MOVS	R3,#28
				MOVS	R4,R0

whileputnumhex	
				MOVS	R2,R4
				LSRS	R2,R2,R3

				MOVS	R1,#0xF
				ANDS	R2,R2,R1
				CMP		R2,#10
				BLO		digits
				ADDS	R2,R2,#'7'
				B		letter
digits			ADDS	R2,R2,#'0'
letter			
				MOVS	R0,R2
				BL		PutChar
				CMP		R3,#0
				BEQ		exitputnumhex
				SUBS	R3,R3,#4
					

				B		whileputnumhex
exitputnumhex	
				POP		{PC,R1,R2,R3,R4}
;###################################################
				
GetHexIntMulti	PROC	{R1-R14}
				PUSH	{LR,R0,R1,R2,R3,R4,R5,R6,R7}
				MOVS	R5,R0
				

				MOVS	R7,#8
				MULS	R1,R7,R1
				ADDS	R1,R1,#1
				LDR		R0,=tempstr
				BL		GetStringSB
				LDR		R0,=tempstr
				BL		LengthStringSB
				SUBS	R1,R1,#1



				SUBS	R6,R1,R0
				;###
				MOVS	R1,#0
				LDR		R2,=tempstr
whilecheckstringmulti
				
				LDRB	R0,[R2,R1]
				CMP 	R0,#NULL
				BEQ		validhex
				CMP		R0,#'0'
				BLT		notvalidhex
				CMP		R0,#'9'
				BGT		ninecheck
				
ninecheckdone	CMP		R0,#'F'
				BGT		fcheck
				
fcheckdone		CMP		R0,#'f'
				BGT		notvalidhex		
				ADDS	R1,R1,#1
				B		whilecheckstringmulti
			
			
			
ninecheck		CMP		R0,#'A'		
				BLT		notvalidhex
				B		ninecheckdone
				
fcheck			CMP		R0,#'a'		
				BLT		notvalidhex
				CMP		R0,#'f'
				BGT		notvalidhex	
				SUBS	R0,R0,#0x20
				B		fcheckdone
			
notvalidhex		
				MRS		R2,APSR
				LDR		R3,=0x20000000
				ORRS	R2,R2,R3
				MSR		APSR,R2
				B		endgetmultihex
				
				
validhex		BL		newline
				MOVS	R4,#0
zeroloop
				MOVS	R0,#'0'
				CMP		R6,R4
				BEQ		nomorezero
				STRB	R0,[R5,R4]
				;BCS		notvalidhex
				ADDS	R4,R4,#1
				B		zeroloop
				
nomorezero		;
				LDR		R0,=tempstr
				MOVS	R7,R0
				BL		LengthStringSB
				MOVS	R6,R0
				MOVS	R2,#0
leadzerodoneloop	
				CMP		R6,R2
				BEQ		whilenqstring
				LDRB	R0,[R7,R2]
				STRB	R0,[R5,R4]
				ADDS	R4,R4,#1
				ADDS	R2,R2,#1
				B		leadzerodoneloop
				

		
whilenqstring	MOVS	R0,#NULL
				STRB	R0,[R5,R4]

exitnqstring
;###
				MRS		R2,APSR
				LDR		R3,=0x20000000
				BICS	R2,R2,R3
				MSR		APSR,R2
				
				
endgetmultihex				
				POP		{PC,R0,R1,R2,R3,R4,R5,R6,R7}
				ENDP

;#############################################################
;R0 = sum output
;R1 = add1
;r2 = add2
;R3 = n

AddIntMulitU	PROC	{R1-R14}
				PUSH	{LR,R0,R1,R2,R3,R4,R5,R6,R7}
				MOVS	R4,R0
				MOVS	R5,R1
				MOVS	R6,R2
				MOVS	R7,#NULL
				

				MOVS	R7,#8
				MULS	R3,R7,R3
				MOVS	R7,#NULL

				STRB	R7,[R0,R3]
				SUBS	R3,R3,#1
				MOVS	R1,#0

addloop			
				LDRB	R0,[R5,R3]
				LDRB	R2,[R6,R3]
				CMP		R0,#'A'
				BGE		letter1
				CMP		R0,#'0'
				BGE		digit
cont1			CMP		R2,#'A'
				BGE		letter2
				CMP		R2,#'0'
				BGE		digit2
cont2fr			CMP		R1,#0
				BEQ		nocarry
				PUSH	{R2,R3}
				MRS		R2,APSR
				LDR		R3,=0x20000000
				ORRS	R2,R2,R3
				MSR		APSR,R2
				POP		{R2,R3}
				B		madcarry
nocarry			PUSH	{R2,R3}
				MRS		R2,APSR
				LDR		R3,=0x20000000
				BICS	R2,R2,R3
				MSR		APSR,R2
				POP		{R2,R3}
madcarry		ADCS	R0,R2,R0
				MOVS	R7,R0
				CMP		R7,#0xF
				BHI		carrytime
				MOVS	R1,#0
carrytimedone				
				CMP		R7,#10
				BLO		digitsa
				ADDS	R7,R7,#'7'
				B		lettera
digitsa			ADDS	R7,R7,#'0'
lettera			
				STRB	R7,[R4,R3]
				CMP		R3,#0
				BEQ		exitaddloop
				SUBS	R3,R3,#1
				B		addloop
carrytime		
				MOVS	R1,#0xF
				ANDS	R7,R7,R1
				MOVS	R1,#1
				B		carrytimedone
				
				
letter1			SUBS	R0,R0,#'7'
				B		cont1
letter2			SUBS	R2,R2,#'7'
				B		cont2fr

digit			SUBS	R0,R0,#'0'
				B		cont1
digit2			SUBS	R2,R2,#'0'
				B		cont2fr
				
exitaddloop		
				
				CMP		R1,#0
				BEQ		nocarryfr
				PUSH	{R2,R3}
				MRS		R2,APSR
				LDR		R3,=0x20000000
				ORRS	R2,R2,R3
				MSR		APSR,R2
				POP		{R2,R3}
				B		madcarryfr
nocarryfr		PUSH	{R2,R3}
				MRS		R2,APSR
				LDR		R3,=0x20000000
				BICS	R2,R2,R3
				MSR		APSR,R2
				POP		{R2,R3}
madcarryfr		
				
					
			
				POP		{PC,R0,R1,R2,R3,R4,R5,R6,R7}
				ENDP		
				
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
			ALIGN
prompt		DCB		" Enter first 96-bit hex number: 0x",NULL
prompt2		DCB		"Enter 96-bit hex number to add: 0x",NULL
printsum	DCB		"                           Sum: 0x",NULL
failure		DCB		"     Invalid number--try again: 0x",NULL
overflow	DCB		"OVERFLOW",NULL

;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Add1		SPACE	Q_REC_SZ
			ALIGN
Add2		SPACE	Q_REC_SZ
			ALIGN
Sum			SPACE	Q_REC_SZ
			ALIGN
tempstr		SPACE	Q_REC_SZ

;>>>>>   end variables here <<<<<
            ALIGN
            END
