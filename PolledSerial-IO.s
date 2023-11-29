
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
			EXPORT PutChar
			IMPORT Cain
			IMPORT Melton
			IMPORT PutPrompt
			IMPORT Shaaban
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
MainLoop2	BL		PutPrompt
MainLoop
NotChar			BL		GetChar		;Gets the char inputted
			MOVS	R5,#0x60		;checks if it is uppercase
			CMP 	r0,R5
			BLT		adder
			MOVS	R2,#0			;if uppercase, add 0x20 to make lowercase
continue
			
			MOVS	R5,#0x63		;checks for the letter c
			CMP 	r0,R5
			BEQ		c
			MOVS	R5,#0x6D
			CMP 	r0,R5
			BEQ		m				;checks for the letter m
			MOVS	R5,#0x73
			CMP 	r0,R5
			BEQ		s				;checks for the letter s

			B 		MainLoop		;loops and gets another input if not c, m, s

adder
			MOVS	R2,R0
			ADDS 	r0,#0x20
			
			B continue 
;>>>>>   end main program code <<<<<
capitalc
			MOVS	R0,R2
			B		capitalcr
			
capitalm
			MOVS	R0,R2
			B		capitalmr
			
capitals
			MOVS	R0,R2
			B		capitalsr
;Stay here	
c				
			CMP		R2,#0
			BGT		capitalc	; Comes here if the letter is c, then calls the Cain program
capitalcr	BL		PutChar
			BL		Cain
			B 		MainLoop2
m
			CMP		R2,#0
			BGT		capitalm
capitalmr	BL		PutChar			; Comes here if the letter is m, then calls the Melton program
			BL		Melton
			B 		MainLoop2
s
			CMP		R2,#0
			BGT		capitals
capitalsr	BL		PutChar			; Comes here if the letter is s, then calls the Shaaban program
			BL		Shaaban
			B 		MainLoop2			
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
			
PutChar
;Poll TDRE until UART0 ready to transmit
		LDR R1,=UART0_BASE
		MOVS R2,#UART0_S1_TDRE_MASK
PollTx 		LDRB R3,[R1,#UART0_S1_OFFSET]
		ANDS R3,R3,R2
		BEQ PollTx
;Transmit character stored in R0
		STRB R0,[R1,#UART0_D_OFFSET]
		BX	 LR
			
			;Poll RDRF until UART0 ready to receive
GetChar
		LDR R1,=UART0_BASE
		MOVS R2,#UART0_S1_RDRF_MASK
PollRx 	LDRB R3,[R1,#UART0_S1_OFFSET]
		ANDS R3,R3,R2
		BEQ PollRx
;Receive character and store in R0
		LDRB R0,[R1,#UART0_D_OFFSET]
		BX	 LR
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
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;>>>>>   end variables here <<<<<
            ALIGN
            END
