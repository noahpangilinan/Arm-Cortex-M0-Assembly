            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
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
Q_REC_SZ    EQU   18  ;Queue management record
RQ_BUF_SZ    EQU   80   ;Queue buffer contents
RQ_REC_SZ    EQU   18  ;Queue management record
TQ_BUF_SZ    EQU   80   ;Queue buffer contents
TQ_REC_SZ    EQU   18  ;Queue management record
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
			IMPORT  Startup

Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
				CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
			
			BL		Init_UART0_IRQ
			BL		Init_PIT_IRQ

			CPSIE		I
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			LDR		R0,=QBuffer
			LDR		R1,=QRecord
			MOVS	R2,#Q_BUF_SZ
			BL		InitQueue

putprompt	
			MOVS	R2,#0
			
			LDR		R0,=prompt
			BL		PutStringSB
			BL		newline
			LDR		R0,=gthan
			BL		PutStringSB
			
char		BL		GetChar

			MOVS	R6,R0
			MOVS	R5,#0x60		;checks if it is uppercase
			CMP 	r0,R5	
        	BLT		tolowercase1
alllower			;checks for the letter d
			CMP 	r0,#'d'
			BEQ		d
			CMP 	r0,#'c'
			BEQ		c				;checks for the letter e
			CMP 	r0,#'h'
			BEQ		h				;checks for the letter h
			CMP 	r0,#'p'
			BEQ		p				;checks for the letter p
			CMP 	r0,#'t'
			BEQ		t				;checks for the letter s
			B		char
;###############
capitald
			MOVS	R0,R2
			B		capitaldr
			
capitalc
			MOVS	R0,R2
			B		capitalcr
			
capitalh
			MOVS	R0,R2
			B		capitalhr
capitalp
			MOVS	R0,R2
			B		capitalpr
capitalt
			MOVS	R0,R2
			B		capitaltr
;Stay here	
d				
			CMP		R2,#0
			BGT		capitald	; Comes here if the letter is d and dequeues a char if queue is not empty
capitaldr	BL		PutChar
			
			MOVS	R0,#' '
			BL		PutChar
			LDR		R0,=COUNT
			LDR		R0,[R0,#0]
			BL		PutNumU
			LDR		R0,=dstr
			BL		PutStringSB
			BL		newline
			B		putprompt

c
			CMP		R2,#0
			BGT		capitalc
capitalcr	BL		PutChar			; Comes here if the letter is e and enqueues char if valid
			LDR	R0,=COUNT
			MOVS R1,#0
			STR	R1,[R0,#0]

			BL		newline
			B		putprompt
					
h
			CMP		R2,#0
			BGT		capitalh
capitalhr	BL		PutChar			; Comes here if the letter is h, prints help string
			BL		newline
			LDR		R0,=helpstr
			MOVS	R1,#NULL
			BL		PutStringSB
			BL		newline
			B		putprompt
tolowercase1
			B		tolowercase

p; Comes here if the letter is p, prints queue
			CMP		R2,#0
			BGT		capitalp
capitalpr	BL		PutChar
			LDR	R0,=RunStopWatch
			MOVS R1,#0
			STRB	R1,[R0,#0]
			BL		newline
			B		putprompt
			
t; Comes here if the letter is s, prints status
			CMP		R2,#0
			BGT		capitalt
capitaltr	BL		PutChar
			LDR		R0,=RunStopWatch
			MOVS 	R1,#1
			STRB	R1,[R0,#0]
			BL		newline	

			B		putprompt			
			
			
tolowercase
			MOVS	R2,R0
			ADDS 	r0,#0x20
			B		alllower
		
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<

			
;##############################################################
Init_PIT_IRQ	PROC	{R0-R14}
			PUSH {R0-R4}
			;#########
			;Enable clock for PIT module
			LDR	R0,=RunStopWatch
			MOVS R1,#0
			STRB	R1,[R0,#0]
			LDR	R0,=COUNT
			MOVS R1,#0
			STR	R1,[R0,#0]
			LDR R0,=SIM_SCGC6
			LDR R1,=SIM_SCGC6_PIT_MASK
			LDR R2,[R0,#0]
			ORRS R2,R2,R1
			STR R2,[R0,#0]
			;#############
			;Disable PIT timer 0
			LDR R0,=PIT_CH0_BASE
			LDR R1,=PIT_TCTRL_TEN_MASK
			LDR R2,[R0,#PIT_TCTRL_OFFSET]
			BICS R2,R2,R1
			STR R2,[R0,#PIT_TCTRL_OFFSET]
	;Set PIT interrupt priority
			LDR R0,=PIT_IPR
			LDR R1,=NVIC_IPR_PIT_MASK
			;LDR Rk,=NVIC_IPR_PIT_PRI_0
			LDR R2,[R0,#0]
			BICS R2,R2,R1
			;ORRS Rl,Rl,Rk
			STR R2,[R0,#0]
			;Clear any pending PIT interrupts
			LDR R1,=NVIC_ICPR
			LDR R1,=NVIC_ICPR_PIT_MASK
			STR R1,[R0,#0]
;Unmask PIT interrupts
			LDR R0,=NVIC_ISER
			LDR R1,=NVIC_ISER_PIT_MASK
			STR R1,[R0,#0]
;Enable PIT module
			LDR R0,=PIT_BASE
			LDR R1,=PIT_MCR_EN_FRZ
			STR R1,[R0,#PIT_MCR_OFFSET]
;Set PIT timer 0 period for 0.01 s
			LDR R0,=PIT_CH0_BASE
			LDR R1,=PIT_LDVAL_10ms
			STR R1,[R0,#PIT_LDVAL_OFFSET]
;Enable PIT timer 0 interrupt
			LDR R1,=PIT_TCTRL_CH_IE
			STR R1,[R0,#PIT_TCTRL_OFFSET]
			POP	{R0,R1,R2,R3,R4}
			BX	LR
			ENDP		



;#################################
Init_UART0_IRQ	PROC	{R0-R14}
			;#########
			PUSH	{LR,R0,R1,R2,R3}
			LDR		R0,=RQBuffer
			LDR		R1,=RQRecord
			MOVS	R2,#RQ_BUF_SZ
			BL		InitQueue
			
			LDR		R0,=TQBuffer
			LDR		R1,=TQRecord
			MOVS	R2,#TQ_BUF_SZ
			BL		InitQueue
			;#############
					
;Select MCGFLLCLK as UART0 clock source
			LDR 	R0,=SIM_SOPT2
			LDR 	R1,=SIM_SOPT2_UART0SRC_MASK
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			LDR 	R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
;Set UART0 for external connection
			LDR 	R0,=SIM_SOPT5
			LDR 	R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			STR 	R2,[R0,#0]
;Enable UART0 module clock
			LDR 	R0,=SIM_SCGC4
			LDR 	R1,=SIM_SCGC4_UART0_MASK
			LDR		R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
;Enable PORT B module clock
			LDR 	R0,=SIM_SCGC5
			LDR 	R1,=SIM_SCGC5_PORTB_MASK
			LDR 	R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
			LDR 	R0,=PORTB_PCR2
			LDR 	R1,=PORT_PCR_SET_PTB2_UART0_RX
			STR 	R1,[R0,#0]
; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
			LDR 	R0,=PORTB_PCR1
			LDR 	R1,=PORT_PCR_SET_PTB1_UART0_TX
			STR 	R1,[R0,#0]
;Disable UART0 receiver and transmitter
			LDR 	R0,=UART0_BASE
			MOVS 	R1,#UART0_C2_T_R
			LDRB 	R2,[R0,#UART0_C2_OFFSET]
			BICS 	R2,R2,R1
			STRB 	R2,[R0,#UART0_C2_OFFSET]
;Set UART0 IRQ priority
			LDR 	R0,=UART0_IPR
;LDR R1,=NVIC_IPR_UART0_MASK
			LDR 	R2,=NVIC_IPR_UART0_PRI_3
			LDR 	R3,[R0,#0]
;BICS R3,R3,R1
			ORRS 	R3,R3,R2
			STR 	R3,[R0,#0]
;Clear any pending UART0 interrupts
			LDR 	R0,=NVIC_ICPR
			LDR 	R1,=NVIC_ICPR_UART0_MASK
			STR 	R1,[R0,#0]
;Unmask UART0 interrupts
			LDR		R0,=NVIC_ISER
			LDR 	R1,=NVIC_ISER_UART0_MASK
			STR 	R1,[R0,#0]
;Set UART0 for 9600 baud, 8N1 protocol
			LDR		R0, =UART0_BASE
			MOVS 	R1,#UART0_BDH_9600
			STRB 	R1,[R0,#UART0_BDH_OFFSET]
			MOVS 	R1,#UART0_BDL_9600
			STRB 	R1,[R0,#UART0_BDL_OFFSET]
			MOVS	R1,#UART0_C1_8N1
			STRB 	R1,[R0,#UART0_C1_OFFSET]
			MOVS 	R1,#UART0_C3_NO_TXINV
			STRB 	R1,[R0,#UART0_C3_OFFSET]
			MOVS 	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB 	R1,[R0,#UART0_C4_OFFSET]
			MOVS 	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB 	R1,[R0,#UART0_C5_OFFSET]
			MOVS 	R1,#UART0_S1_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S1_OFFSET]
			MOVS 	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S2_OFFSET]
;Enable UART0 receiver and transmitter
			LDR		R0,=UART0_BASE
			MOVS 	R1,#UART0_C2_T_RI
			STRB 	R1,[R0,#UART0_C2_OFFSET]
			
			POP	    {PC,R0,R1,R2,R3}
			ENDP
;###############################################################################################			
PutChar	PROC	{R1-R14}
		PUSH  {LR,R1,R2,R3}
		LDR 	R1,=TQRecord
PollTx	CPSID	I
		BL		Enqueue
		CPSIE	I
		BCS		PollTx
		LDR 	R1,=UART0_BASE
		MOVS 	R2,#UART0_C2_TI_RI
		STRB 	R2,[R1,#UART0_C2_OFFSET]
		POP	  	{PC,R1,R2,R3}
		ENDP
;###############################################################################################			
			;Poll RDRF until UART0 ready to receive
GetChar	PROC	{R1-R14}
		PUSH  {LR,R1,R2,R3}
		LDR 	R1,=RQRecord
PollRx	CPSID	I
		BL		Dequeue
		CPSIE	I
		BCS		PollRx
		POP	  {PC,R1,R2,R3}
		ENDP
		

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
PutStringSB	PROC	{R1-R14}			;calls put char in a while loop until null char
		PUSH	{LR,R1,R2}
		MOVS	R2,R0
		MOVS	R1,#0
		
whileputstring
		LDRB	R0,[R2,R1]
		CMP 	R0,#NULL
		BEQ		exitputstring
		BL		PutChar
		ADDS	R1,R1,#1
		B		whileputstring
		
exitputstring
		POP	{PC,R1,R2}
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
;Takes R0 and puts the decimal value
PutNumU		PUSH	{LR,R3}
			MOVS	R3,#0
whileputnum	MOVS	R1,R0
			MOVS	R0,#10
			BL		divu
			MOVS	R2,R0
			MOVS	R0,R1
			ADDS	R0,R0,#'0'
			
			PUSH	{R0}
			ADDS	R3,R3,#1
			
			CMP		R2,#0
			BEQ		exitputnum
			MOVS	R0,R2
			B		whileputnum
exitputnum	
			
Reverse		
			CMP		R3,#0
			BEQ		exitfr
			POP		{R0}
			BL		PutChar
			SUBS	R3,R3,#1
			B		Reverse
exitfr		POP		{PC,R3}

;###################################
newline	;prints to the new line
			PUSH 	{LR,R0,R1}
			MOVS	R0,#CR
			BL		PutChar
			MOVS	R0,#LF
			BL		PutChar
			POP		{PC,R0,R1}
			BX		LR
			
;######################################
PutNumUB	;masks last byte and prints it
			PUSH	{LR}
			LDR		R1,=0x0000000F
			ANDS	R0,R0,R1
			BL		PutNumU
			POP		{PC}
			
			
;######################################
InitQueue		PROC	{R1-R14};initilizes the queue



				STR		R0,[R1,#IN_PTR]
				STR		R0,[R1,#OUT_PTR]
				STR		R0,[R1,#BUF_STRT]
				ADDS	R0,R0,R2
				STR		R0,[R1,#BUF_PAST]
				STRB	R2,[R1,#BUF_SIZE]
				MOVS	R0,#0
				STRB	R0,[R1,#NUM_ENQD]
				

	
				
				BX		LR
				ENDP
;######################################
;takes r1 as the addy
;puts the next item of the queue and removes it from said queue
Dequeue			PROC	{R1-R14}
				PUSH	{R1,R2,R3,R4}
				LDRB	R0,[R1,#NUM_ENQD]
				CMP		R0,#0
				BEQ		exitdequeue
				LDR		R2,[R1,#OUT_PTR]
				LDRB	R5,[R2,#0]
				SUBS	R0,R0,#1
				STRB	R0,[R1,#NUM_ENQD]
				ADDS	R2,R2,#1
				STR		R2,[R1,#OUT_PTR]
				LDR		R3,[R1,#BUF_PAST]
				CMP		R2,R3
				BGE		ajoutptr
contdq			MRS		R2,APSR	;clears c bit
				LDR		R3,=0x20000000
				BICS	R2,R2,R3
				MSR		APSR,R2
				B		enddqfr

ajoutptr		LDR		R4,[R1,#BUF_STRT]
				STR		R4,[R1,#OUT_PTR]
				B		contdq
exitdequeue		MRS		R2,APSR	;sets c bit
				LDR		R3,=0x20000000
				ORRS	R2,R2,R3
				MSR		APSR,R2
enddqfr			MOVS	R0,R5
				POP	{R1,R2,R3,R4}
				
				BX		LR
				ENDP
;###########################################
;takes R1 as add
;takes char in R0 and adds it to queue
Enqueue			PROC	{R1-R14}
				PUSH	{R0,R2,R3,R4}
				LDRB	R2,[R1,#NUM_ENQD]
				LDRB	R3,[R1,#BUF_SIZE]
				CMP		R2,R3
				BGE		exitenq
				
				LDR		R4,[R1,#IN_PTR]
				STRB	R0,[R4,#0]
				
				ADDS	R4,R4,#1
				STR		R4,[R1,#IN_PTR]
				
				ADDS	R2,R2,#1
				STRB	R2,[R1,#NUM_ENQD]
				
				LDR		R3,[R1,#BUF_PAST]
				CMP		R4,R3
				BGE		ajinptr
contenq			MRS		R2,APSR;clears c bit
				LDR		R3,=0x20000000
				BICS	R2,R2,R3
				MSR		APSR,R2
				B		endenqfr

ajinptr			LDR		R4,[R1,#BUF_STRT]
				STR		R4,[R1,#IN_PTR]
				B		contenq
exitenq			MRS		R2,APSR;sets c bit
				LDR		R3,=0x20000000
				ORRS	R2,R2,R3
				MSR		APSR,R2
endenqfr		POP	{R1,R2,R3,R4}
				
				BX		LR
				ENDP
				
				

				
				
				
				
;###########################################
PutNumHex	;puts hex value of r1 to terminal
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
				;#########################################
UART0_ISR	PROC	{R0-R14}
			CPSID	I							
			PUSH	{R0-R7,LR}
			LDR 	R4,=UART0_BASE
			LDRB	R2,[R4,#UART0_C2_OFFSET]	
			MOVS 	R3,#UART0_C2_TIE_MASK
			TST 	R3,R2
			BEQ		TxInteruptEnabled
			MOVS 	R2,#UART0_S1_TDRE_MASK  
			LDRB 	R5,[R4,#UART0_S1_OFFSET]
            TST 	R5,R2						
			BEQ		TxInteruptEnabled
			LDR 	R1,=TQRecord					
            BL 		Dequeue	
			BCS		dequeueunsuccessful					
			LDR		R2,=UART0_BASE
            STRB 	R0,[R2,#UART0_D_OFFSET]
			B		TxInteruptEnabled
dequeueunsuccessful			
			LDR		R2,=UART0_BASE
			MOVS	R0,#UART0_C2_T_RI
			STRB	R0,[R2,#UART0_C2_OFFSET]			
TxInteruptEnabled	
			MOVS 	R5,#UART0_S1_RDRF_MASK
			LDR		R4,=UART0_BASE
			LDRB 	R6,[R4,#UART0_S1_OFFSET]    
			TST		R6,R5
			BEQ		endisr
			LDRB 	R0,[R4,#UART0_D_OFFSET]
			LDR 	R1,=RQRecord
			BL 		Enqueue
endisr 		CPSIE	I							
			POP 	{R0-R7,PC}
			ENDP
				
			;#########################################
PIT_ISR		PROC	{R0-R14}							
			LDR		R0,=RunStopWatch
			LDRB	R0,[R0,#0]
			CMP		R0,#1
			BEQ		INCREMENTCOUNT
			B		DONTINCREMENTCOUNT
INCREMENTCOUNT	
			LDR		R1,=COUNT
			LDR		R0,[R1,#0]
			ADDS	R0,#1
			STR		R0,[R1,#0]
DONTINCREMENTCOUNT			
			LDR		R0,=PIT_CH0_BASE
			LDR		R1,=PIT_TFLG_TIF_MASK
			STR		R1,[R0,#PIT_TFLG_OFFSET]
			BX		LR
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
            DCD    UART0_ISR	      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR      ;38:PIT
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
			
prompt		DCB		"Type a command (C,D,H,P,T):",NULL
char2nq		DCB		"Character to enqueue:",NULL
succ		DCB		"Success:",NULL
fail		DCB		"Failure:",NULL
helpstr		DCB		"C(lear),D(isplay),H(elp),P(ause),T(ime)",NULL
gthan		DCB		">",NULL
lthan		DCB		"<",NULL
status		DCB		"Status:",NULL
inhexprmpt	DCB		"In=0x",NULL
outhexprmpt	DCB		" Out=0x",NULL
numprmpt	DCB		" Num=",NULL
colon		DCB		":",NULL
dstr		DCB		" x 0.01 s",NULL
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
QRecord		SPACE	Q_REC_SZ
			ALIGN
QBuffer		SPACE	Q_BUF_SZ
			ALIGN	
RQRecord	SPACE	RQ_REC_SZ
			ALIGN
RQBuffer	SPACE	RQ_BUF_SZ
			ALIGN	
TQRecord	SPACE	TQ_REC_SZ
			ALIGN
TQBuffer	SPACE	TQ_BUF_SZ
			ALIGN	
STRING		SPACE 	MAX_STRING
			ALIGN
RunStopWatch	SPACE  1
			ALIGN	
COUNT		SPACE		4
			

;>>>>>   end variables here <<<<<
            ALIGN
            END
