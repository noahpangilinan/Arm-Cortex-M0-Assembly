            TTL Program Title for Listing Header Goes Here
;****************************************************************
;
;Implements a divide function and tests it
;Name:  Noah Pangilinan
;Date:  9/28/23
;Class:  CMPE-250
;Section:  4, Thursday, 2pm
;---------------------------------------------------------------
;Keil Simulator Template for KL05
;R. W. Melton
;January 21, 2021
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
;Standard data masks
BYTE_MASK         EQU  0xFF
NIBBLE_MASK       EQU  0x0F
;Standard data sizes (in bits)
BYTE_BITS         EQU  8
NIBBLE_BITS       EQU  4
;Architecture data sizes (in bytes)
WORD_SIZE         EQU  4  ;Cortex-M0+
HALFWORD_SIZE     EQU  2  ;Cortex-M0+
;Architecture data masks
HALFWORD_MASK     EQU  0xFFFF
;Return                 
RET_ADDR_T_MASK   EQU  1  ;Bit 0 of ret. addr. must be
                          ;set for BX, BLX, or POP
                          ;mask in thumb mode
;---------------------------------------------------------------
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0  ;KL05
VECTOR_SIZE       EQU 4           ;Bytes per vector
;---------------------------------------------------------------
;CPU CONTROL:  Control register
;31-2:(reserved)
;   1:SPSEL=current stack pointer select
;           0=MSP (main stack pointer) (reset value)
;           1=PSP (process stack pointer)
;   0:nPRIV=not privileged
;        0=privileged (Freescale/NXP "supervisor") (reset value)
;        1=not privileged (Freescale/NXP "user")
CONTROL_SPSEL_MASK   EQU  2
CONTROL_SPSEL_SHIFT  EQU  1
CONTROL_nPRIV_MASK   EQU  1
CONTROL_nPRIV_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PRIMASK:  Interrupt mask register
;31-1:(reserved)
;   0:PM=prioritizable interrupt mask:
;        0=all interrupts unmasked (reset value)
;          (value after CPSIE I instruction)
;        1=prioritizable interrrupts masked
;          (value after CPSID I instruction)
PRIMASK_PM_MASK   EQU  1
PRIMASK_PM_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PSR:  Program status register
;Combined APSR, EPSR, and IPSR
;----------------------------------------------------------
;CPU APSR:  Application Program Status Register
;31  :N=negative flag
;30  :Z=zero flag
;29  :C=carry flag
;28  :V=overflow flag
;27-0:(reserved)
APSR_MASK     EQU  0xF0000000
APSR_SHIFT    EQU  28
APSR_N_MASK   EQU  0x80000000
APSR_N_SHIFT  EQU  31
APSR_Z_MASK   EQU  0x40000000
APSR_Z_SHIFT  EQU  30
APSR_C_MASK   EQU  0x20000000
APSR_C_SHIFT  EQU  29
APSR_V_MASK   EQU  0x10000000
APSR_V_SHIFT  EQU  28
;----------------------------------------------------------
;CPU EPSR
;31-25:(reserved)
;   24:T=Thumb state bit
;23- 0:(reserved)
EPSR_MASK     EQU  0x01000000
EPSR_SHIFT    EQU  24
EPSR_T_MASK   EQU  0x01000000
EPSR_T_SHIFT  EQU  24
;----------------------------------------------------------
;CPU IPSR
;31-6:(reserved)
; 5-0:Exception number=number of current exception
;      0=thread mode
;      1:(reserved)
;      2=NMI
;      3=hard fault
;      4-10:(reserved)
;     11=SVCall
;     12-13:(reserved)
;     14=PendSV
;     15=SysTick
;     16=IRQ0
;     16-47:IRQ(Exception number - 16)
;     47=IRQ31
;     48-63:(reserved)
IPSR_MASK             EQU  0x0000003F
IPSR_SHIFT            EQU  0
IPSR_EXCEPTION_MASK   EQU  0x0000003F
IPSR_EXCEPTION_SHIFT  EQU  0
;----------------------------------------------------------
PSR_N_MASK           EQU  APSR_N_MASK
PSR_N_SHIFT          EQU  APSR_N_SHIFT
PSR_Z_MASK           EQU  APSR_Z_MASK
PSR_Z_SHIFT          EQU  APSR_Z_SHIFT
PSR_C_MASK           EQU  APSR_C_MASK
PSR_C_SHIFT          EQU  APSR_C_SHIFT
PSR_V_MASK           EQU  APSR_V_MASK
PSR_V_SHIFT          EQU  APSR_V_SHIFT
PSR_T_MASK           EQU  EPSR_T_MASK
PSR_T_SHIFT          EQU  EPSR_T_SHIFT
PSR_EXCEPTION_MASK   EQU  IPSR_EXCEPTION_MASK
PSR_EXCEPTION_SHIFT  EQU  IPSR_EXCEPTION_SHIFT
;----------------------------------------------------------
;Stack
SSTACK_SIZE EQU  0x00000100
	

;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
			IMPORT InitData
			IMPORT LoadData
			IMPORT TestData
Reset_Handler  PROC {}
main
;---------------------------------------------------------------
;Initialize registers R0-R12
            BL      RegInit
;>>>>> begin main program code <<<<<

			BL 	 InitData
MAINLOOP
			BL	 LoadData
			BCS  ENDPROG

			
			LDR R1,=P
			LDR	 R1,[R1,#0]
			LDR R0,=Q
			LDR	 R0,[R0,#0]
			BL	 divu
			BCS  SETF
			LDR		R2,=P
			STR		R0,[R2,#0]
			LDR		R2,=Q
			STR		R1,[R2,#0]
			B	 TESTDATAPQ
			
SETF	;sets as 0xFFFFFFFF if carry is set after divu
			MOVS	R2,#1
			RSBS	R3,R2,#0
			MOVS	R3,R3
			LDR		R2,=P
			STR		R3,[R2,#0]
			LDR		R2,=Q
			STR		R3,[R2,#0]



TESTDATAPQ	BL 	 TestData
			B	 MAINLOOP
			
ENDPROG			
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;---------------------------------------------------------------
RegInit     PROC  {}
;****************************************************************
;Initializes register n to value 0xnnnnnnnn, for n in 
;{0x1-0xC,0xE}.  Initializes R0 to 0x05082523
;****************************************************************
;Put return on stack
            PUSH    {LR}
;Initialize registers
            LDR     R1,=0x11111111
            ADDS    R2,R1,R1
            ADDS    R3,R2,R1
            ADDS    R4,R3,R1
            ADDS    R5,R4,R1
            ADDS    R6,R5,R1
            ADDS    R7,R6,R1
            ADDS    R0,R7,R1
            MOV     R8,R0
            ADDS    R0,R0,R1
            MOV     R9,R0
            ADDS    R0,R0,R1
            MOV     R10,R0
            ADDS    R0,R0,R1
            MOV     R11,R0
            ADDS    R0,R0,R1
            MOV     R12,R0
            ADDS    R0,R0,R1
            ADDS    R0,R0,R1
            MOV     R14,R0
            LDR     R0,=0x05082523
            POP     {PC}
            ENDP
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<

divu		PROC	{R4-R14} ;unused registers
			PUSH	{R2,R3}
			MOVS	R2,#0
			CMP		R0,R2 ;checks if dividing by zero
			BEQ		divbyzero
			
while		CMP		R1,R0
			BLO		end
			SUBS	R1,R1,R0
			ADDS	R2,R2,#1
			B		while		
end
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
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFFC00|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
			EXPORT P
			EXPORT Q
			EXPORT Results
;>>>>> begin variables here <<<<<
P			SPACE	4
Q			SPACE	4
Results		SPACE	200
;>>>>>   end variables here <<<<<
            END
