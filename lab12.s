            TTL Embedded LED Game Functions and Drivers
;****************************************************************
; Serial I/O driver with queue operations and ISR support,
; PIT driver for pseudorandom number generation,
; For use by C LED game embedded program
;Name:  Paul Kelly
;Date:  11-13-2018
;Class:  CMPE-250
;Section: 01 Tues. 1100-1250
;---------------------------------------------------------------
;Keil Template for KL46
;R. W. Melton
;February 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1             ;Turn on listing
;****************************************************************
;EQUates
;****************************************************************
;---------------------------------------------------------------
;Characters
CR          EQU  	0x0D
LF          EQU  	0x0A
BS			EQU		0x08
NULL        EQU  	0x00
COLON		EQU		0x3A
SPA			EQU		0x20
RIGHT		EQU		0x3E
LEFT		EQU		0x3C

; Queue
QRECSIZE    EQU     18
QBUFSIZE    EQU     4
	
IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17

FIVESEC	    EQU		500

;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
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
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
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
;--PIT
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << UART0_PRI_POS)
;--UART0
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU  (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU  (UART0_IRQ_PRIORITY << UART0_PRI_POS)
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
;Clock ticks for 0.01 s at 24 MHz count rate
;0.01 s * 24,000,000 Hz = 240,000
;TSV = 240,000 - 1
PIT_LDVAL_10ms  EQU  239999
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRLn:  PIT timer control register n
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
; LED Port Connections
; Port D
PTD5_MUX_GPIO		EQU		(1 << PORT_PCR_MUX_SHIFT)
SET_PTD5_GPIO		EQU		(PORT_PCR_ISF_MASK :OR: \
								PTD5_MUX_GPIO)
; Port E
PTE29_MUX_GPIO		EQU		(1 << PORT_PCR_MUX_SHIFT)
SET_PTE29_GPIO		EQU		(PORT_PCR_ISF_MASK :OR: \
								PTE29_MUX_GPIO)
;----------------------------------------------------------------								
; Useful EQUates for KL46Z LEDS
POS_RED				EQU 	29
POS_GREEN			EQU		5

LED_RED_MASK		EQU 	(1 << POS_RED)
LED_GREEN_MASK		EQU		(1 << POS_GREEN)

LED_PORTD_MASK		EQU		LED_GREEN_MASK
LED_PORTE_MASK		EQU		LED_RED_MASK


;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
            
            EXPORT      GetChar
            ;EXPORT      GetStringSB
            EXPORT      Init_UART0_IRQ
			EXPORT		Init_PIT_IRQ
			EXPORT		Init_LED
			EXPORT		Set_LED
			EXPORT		KeyPressed
            EXPORT      PutChar
            EXPORT      PutNumHex 
            EXPORT      PutNumUB   
            EXPORT      PutStringSB
			EXPORT		PutNumU
            EXPORT      StartTimer
			
			EXPORT		PIT_IRQHandler                
            EXPORT      UART0_IRQHandler
                    
;>>>>> begin subroutine code <<<<<

;****************************************************************
;	StartTimer
;
;	Init timer by setting RunStopWatch to 1 and reset Count to 0
;****************************************************************
StartTimer		PUSH	{R0, R1}

				LDR		R0, =Count
				MOVS	R1, #0
				STR		R1, [R0, #0]
				
				LDR		R0, =RunStopWatch
				MOVS	R1, #1
				STRB	R1, [R0, #0]
				
				POP		{R0, R1}
				
				BX 		LR
				

;****************************************************************
;	Init_LED
;
;	Initialize the Green and Red LEDs through PORT D and E
;****************************************************************
Init_LED		PUSH	{R0-R2}
				
				; Enable clock for ports D and E
				LDR		R0, =SIM_SCGC5
				LDR		R1, =(SIM_SCGC5_PORTD_MASK :OR: \
								SIM_SCGC5_PORTE_MASK)
				LDR		R2, [R0, #0]
				ORRS	R2, R2, R1
				STR		R2, [R0, #0]
				
				; Select PORT E Pin 29 for GPIO to red LED
				LDR		R0, =PORTE_BASE
				LDR		R1, =SET_PTE29_GPIO
				STR		R1, [R0, #PORTE_PCR29_OFFSET]
				
				; Select PORT D Pin 5 for GPIO to green LED
				LDR		R0, =PORTD_BASE
				LDR		R1, =SET_PTD5_GPIO
				STR		R1, [R0, #PORTD_PCR5_OFFSET]
				
				; Select data direction
				LDR		R0, =FGPIOD_BASE
				LDR		R1, =LED_PORTD_MASK
				STR		R1, [R0, #GPIO_PDDR_OFFSET]
				
				LDR		R0, =FGPIOE_BASE
				LDR		R1, =LED_PORTE_MASK
				STR		R1, [R0, #GPIO_PDDR_OFFSET]
				
				POP		{R0-R2}
				BX		LR

;****************************************************************
;	Set_LED
;	Turn red or green LED on or off
;	Input: R0	bit 1	bit 0
;				red		green
;				0 off	0 off
;				1 on	1 on
;			   
;****************************************************************
Set_LED			PUSH	{R0-R2}

SetGreen		LDR		R1, =FGPIOD_BASE
				LDR		R2, =LED_GREEN_MASK				; prepare green addresses
				
				LSRS	R0, R0, #1						; check if green to be set on
				BCS		TurnGreenOn							
				
				STR		R2, [R1, #GPIO_PSOR_OFFSET]		; set LED off				
				B		SetRed

TurnGreenOn		STR		R2, [R1, #GPIO_PCOR_OFFSET]		; set LED on
				
SetRed			LDR		R1, =FGPIOE_BASE
				LDR		R2, =LED_RED_MASK				; prepare red addresses
				
				LSRS	R0, R0, #1						; check if red to be set on
				BCS		TurnRedOn
				
				STR		R2, [R1, #GPIO_PSOR_OFFSET]		; set LED off
				B		Return
				
TurnRedOn		STR		R2, [R1, #GPIO_PCOR_OFFSET]		; set LED on
				
Return			POP		{R0-R2}
				BX		LR
				
				
;****************************************************************
;	KeyPressed
;	Determine if a key has been pressed without 
;	dequeueing the character or waiting until a 
;	key has been pressed.
;
;	Return: R0 = NUM_ENQD for RxQueueRecord
;			(Key has been pressed when R0 > 0)
;*****************************************************************
KeyPressed		LDR		R0, =RxQRecord
				LDRB	R0, [R0, #NUM_ENQD]
				
				BX		LR


;>>>>> below has been tested and is working <<<<<
;****************************************************************
; 	Subroutine to initialize the PIT to generate
;	an interrupt every 0.01 s
;
;	Timer LDVAL: 239,999 cycles (~0.01s)
;
;	Init to highest priority 0
;****************************************************************
Init_PIT_IRQ	PROC	{R0-R14}
                CPSID	I
                PUSH	{LR, R0-R2}

                ; set SIM_CGC6 for PIT Clock Enabled
                LDR	    R0, =SIM_SCGC6
                LDR	    R1, =SIM_SCGC6_PIT_MASK
                LDR	    R2, [R0, #0]			; load current SIM_SCGC6 value
                ORRS	R2, R2, R1				; set only PIT bit
                STR	    R2, [R0, #0]			; update value

                ; disable PIT timer 0 (PIT_TCTRL0)
                LDR	    R0, =PIT_CH0_BASE
                LDR	    R1, =PIT_TCTRL_TEN_MASK
                LDR	    R2, [R0, #PIT_TCTRL_OFFSET]
                BICS	R2, R2, R1
                STR	    R2, [R0, #PIT_TCTRL_OFFSET]

                ; set PIT IRQ priority to 0
                LDR	    R0, =PIT_IPR
                LDR	    R1, =(NVIC_IPR_PIT_MASK)
                ;LDR	R3, =(PIT_IRQ_PRI << PIT_PRI_POS)
                LDR	    R2, [R0, #0]
                BICS	R2, R2, R1
                ;ORRS	R2, R2, R3
                STR	    R2, [R0, #0]

                ; clear any pending PIT interrupts
                LDR	    R0, =PIT_CH0_BASE
                LDR	    R1, =PIT_TFLG_TIF_MASK
                STR	    R1, [R0, #PIT_TFLG_OFFSET]
            
                ; unmask PIT Interrupts
                LDR	    R0, =NVIC_ISER
                LDR	    R1, =PIT_IRQ_MASK
                STR	    R1, [R0, #0]

                ; enable PIT module
                LDR	    R0, =PIT_BASE
                LDR	    R1, =PIT_MCR_EN_FRZ	; enable FRZ to stop timer in debug
                STR	    R1, [R0, #PIT_MCR_OFFSET]

                ; request interrupts for every 0.01 seconds
                LDR	    R0, =PIT_CH0_BASE
                LDR	    R1, =PIT_LDVAL_10ms
                STR	    R1, [R0, #PIT_LDVAL_OFFSET]

                ; enable PIT timer ch 0 for interrupts
                LDR	    R0, =PIT_CH0_BASE
                MOVS	R1, #PIT_TCTRL_CH_IE
                STR	    R1, [R0, #PIT_TCTRL_OFFSET]			
        
                CPSIE	I
                POP	    {R0-R2, PC}
                ENDP

;****************************************************************
;   	PIT_ISR subroutine
;   	Interrupt Service Routine for PIT driver
;  
;	On interrupt, while RunStopWatch != 0, increment Count
;	
;	On return, interrupt condition is cleared
;*****************************************************************
PIT_IRQHandler
PIT_ISR		PROC	{R0-R14}
			
			LDR	    R0, =RunStopWatch
			LDRB	R0, [R0, #0]
			CMP	    R0, #0
			BEQ	    ClearInt
			
			LDR	    R0, =Count
			LDR	    R1, [R0, #0]
			ADDS	R1, R1, #1
			STR	    R1, [R0, #0] 

ClearInt    LDR	    R0, =PIT_CH0_BASE
            LDR	    R1, =PIT_TFLG_TIF_MASK
            STR	    R1, [R0, #PIT_TFLG_OFFSET]

			BX	    LR
			ENDP 

;****************************************************************
;	Subroutine to initialize UART0 
;	for interrupt-based serial I/O
;****************************************************************
Init_UART0_IRQ	PROC    {R0-R14}
                PUSH	{LR, R0-R2}

                ; initialize Tx queue
                LDR     R0, =TxQ
                LDR     R1, =TxQRecord                
                MOVS    R2, #QBUFSIZE
                BL      InitQueue
                
                ; initialize Rx queue
                LDR     R0, =RxQ
                LDR     R1, =RxQRecord                
                BL      InitQueue

                ;Select MCGPLLCLK / 2 as UART0 clock 
                LDR		R0,=SIM_SOPT2
                LDR		R1,=SIM_SOPT2_UART0SRC_MASK
                LDR		R2,[R0,#0]
                BICS	R2,R2,R1
                LDR		R1,=SIM_SOPT2_UART0_MCGPLLCLK_DIV2
                ORRS	R2,R2,R1
                STR		R2,[R0,#0]

                ;Enable external connection for UART0
                LDR		R0,=SIM_SOPT5
                LDR		R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
                LDR 	R2,[R0,#0]
                BICS	R2,R2,R1
                STR		R2,[R0,#0]
	
                ;Enable clock for UART0 module
                LDR		R0,=SIM_SCGC4
                LDR		R1,=SIM_SCGC4_UART0_MASK
                LDR		R2,[R0,#0]
                ORRS	R2,R2,R1
                STR		R2,[R0,#0]

                ;Enable clock for Port A module
                LDR		R0,=SIM_SCGC5
                LDR		R1,=SIM_SCGC5_PORTA_MASK
                LDR		R2,[R0,#0]
                ORRS	R2,R2,R1
                STR		R2,[R0,#0]

                ;Connect Port A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
                LDR		R0,=PORTA_PCR1
                LDR		R1,=PORT_PCR_SET_PTA1_UART0_RX
                STR		R1,[R0,#0]

                ;Connect Port A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
                LDR		R0,=PORTA_PCR2
                LDR		R1,=PORT_PCR_SET_PTA2_UART0_TX
                STR		R1,[R0,#0]
                
                ; Disable UART0 receiver and transmitter
                LDR     R0, =UART0_BASE
                MOVS    R1, #UART0_C2_T_R
                LDRB    R2, [R0, #UART0_C2_OFFSET]
                BICS    R2, R2, R1
                STRB    R2, [R0, #UART0_C2_OFFSET]
	
                ; Init NVIC for UART0 interrupts
                LDR     R0, =UART0_IPR
                ;LDR    R1, =NVIC_IPR_UART0_MASK
                LDR     R2, =NVIC_IPR_UART0_PRI_3
                LDR     R3, [R0, #0]
                ;BICS   R3, R3, R1
                ORRS    R3, R3, R2
                STR     R3, [R0, #0]
                
                ; clear any pending UART0 pending interrupts
                LDR     R0, =NVIC_ICPR
                LDR     R1, =NVIC_ICPR_UART0_MASK
                STR     R1, [R0, #0]
                
                ;unmask UART0 interrupts
                LDR     R0, =NVIC_ISER
                LDR     R1, =NVIC_ISER_UART0_MASK
                STR     R1, [R0, #0]
		
                ;Set UART0 for 9600 baud, 8N1 protocol
                LDR     R0, =UART0_BASE
                MOVS	R1,#UART0_BDH_9600
                STRB	R1,[R0,#UART0_BDH_OFFSET]
                MOVS	R1,#UART0_BDL_9600
                STRB	R1,[R0,#UART0_BDL_OFFSET]
                MOVS	R1,#UART0_C1_8N1
                STRB	R1,[R0,#UART0_C1_OFFSET]
                MOVS	R1,#UART0_C3_NO_TXINV
                STRB	R1,[R0,#UART0_C3_OFFSET]
                MOVS	R1,#UART0_C4_NO_MATCH_OSR_16
                STRB	R1,[R0,#UART0_C4_OFFSET]
                MOVS	R1,#UART0_C5_NO_DMA_SSR_SYNC
                STRB	R1,[R0,#UART0_C5_OFFSET]
                MOVS	R1,#UART0_S1_CLEAR_FLAGS
                STRB	R1,[R0,#UART0_S1_OFFSET]
                MOVS	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
                STRB	R1,[R0,#UART0_S2_OFFSET]
		
                ;Enable UART0 receiver and transmitter
                MOVS	R1,#UART0_C2_T_RI
                STRB	R1,[R0,#UART0_C2_OFFSET]

                POP		{R0-R2, PC}
                ENDP


;****************************************************************
;   UART0_ISR subroutine
;   Interrupt Service Routine for UART driver
;   
;   Check status of interrupt and handle accordingly
;   Tx Int enabled: write to transmit register
;   Rx Int enabled: enqueue to transmit queue from UART receive register
*****************************************************************
UART0_IRQHandler
UART0_ISR       CPSID   I                               ; mask interrupts
                PUSH    {R0-R3, LR}
                       
                LDR     R0, =UART0_BASE                 
                LDRB    R1, [R0, #UART0_C2_OFFSET]      ; load control register 2
                MOVS    R2, #0x80                       ; set bit 7 in R2
                TST     R1, R2                          ; test with UART C2 
                BEQ     RxCheck                         ; if not set, check if Rx Enabled
                
                LDRB    R1, [R0, #UART0_S1_OFFSET]      ; load serial register
                TST     R1, R2                          ; check bit 7
                BEQ     RxCheck                         ; if not set, check if Rx Enabled 
                
                LDR     R1, =TxQRecord                  ; load tx queue record address
                BL      Dequeue
                BCS     DisableTx                       ; dequeue was unsuccessful, disable transmit interrupt
                
                LDR     R1, =UART0_BASE                 ; dequeue was successful
                STRB    R0, [R1, #UART0_D_OFFSET]       ; transmit character stored in R0
                B       EndISR
                
DisableTx       MOVS    R1, #UART0_C2_T_RI
                STRB    R1, [R0, #UART0_C2_OFFSET]      ; disable tx interrupt 
                B       EndISR                          ; and return
                
RxCheck         LDR     R0, =UART0_BASE
                LDRB    R1,[R0,#UART0_S1_OFFSET]        ; load Rx status
                MOVS    R2, #0x10		                ; set bit 5
                TST     R1, R2                          ; check if Rx bit is set
                BEQ     EndISR		                    ; if not, return
                
                LDRB    R3, [R0, #UART0_D_OFFSET]       ; recieve character and store in R3
                MOVS    R0, R3                          ; relocate to R0 for enqueue subroutine
                LDR     R1, =RxQRecord                  ; load Rx Q record structure address
                BL      Enqueue                         ; attempt to enqueue character, or ignore if full    

EndISR          CPSIE   I
                POP     {R0-R3, PC}
                        

;****************************************************************
;	GetChar Subroutine
;	Reads a single character from serial into R0
;****************************************************************
GetChar	        PROC	{R1-R14}
                PUSH	{R1-R2, LR}

                LDR     R1, =RxQRecord          ; load address of Receive Queue record
DequeueLoop     CPSID   I                       ; disable all interrupts
                BL      Dequeue                 ; dequeue char into R0
                CPSIE   I                       ; enable interrupts 
                BCS     DequeueLoop             ; branch back while unsuccessful        

                POP		{R1-R2, PC}
                ENDP
		
;****************************************************************
;	PutChar Subroutine
;	Transmits character stored in R0
;****************************************************************
PutChar	        PROC	{R0-R14}
                PUSH	{R0-R1, LR}
		
                LDR     R1, =TxQRecord              ; load address of Transmit Queue record
EnqueueLoop     CPSID   I                           ; mask all interrupts
                BL      Enqueue                     ; enqueue character in R0
                CPSIE   I                           ; enable interrupts
                BCS     EnqueueLoop                 ; branch back while not successful
                
                LDR     R0, =UART0_BASE
                MOVS    R1, #UART0_C2_TI_RI
                STRB    R1, [R0, #UART0_C2_OFFSET]  ; enable Tx interrupt

                POP		{R0-R1, PC}
                ENDP


;****************************************************************
;		PutStringSB
;
;		Displays a null terminated string from memory,
;		starting at the address where R0 points, to the
;		terminal screen.
;
;		Parameters
;		Input:
;		R0: Pointer to source string
;		R1: Buffer capacity
;
;		Modify:
;		APSR
;
;		Uses:
;		PutChar
;****************************************************************
PutStringSB		PROC	{R0-R14}
				PUSH	{LR}
				PUSH	{R0-R2}

				MOVS	R2,R0			; move address to R2
PutLoop			LDRB	R0,[R2,#0]		; load byte of string
				CMP		R0,#NULL		; if character is null,
				BEQ		StringEnd		; the string has terminated
				BL		PutChar			; print character to terminal
				ADDS	R2,R2,#1		; increment address
				SUBS	R1,R1,#1		; decrement buffer capacity
				BNE		PutLoop			; loop back until buffer is empty

StringEnd		POP		{R0-R2}
				POP		{PC}
				ENDP

;****************************************************************
;		InitQueue
;
;		Initialize queue record structure
;		
;		Parameters
;		Input:
;		R0: pointer for empty queue buffer 
;		R1: pointer for new queue record structure
;		R2: queue size
;
;		Modify:
;		APSR
;****************************************************************
InitQueue	PROC 	{R0-R14}
			PUSH 	{R1-R3}

			STM		R1!, {R0}		; save to IN_PTR, increment address
			STM		R1!, {R0}		; save to OUT_PTR, increment
			STM		R1!, {R0}		; save to BUF_STRT, increment
			ADDS	R3, R0, R2		; save queue past address to R3
			STM		R1!, {R3}		; save to BUF_PAST
			STRB	R2, [R1,#0]		; save size to BUF_SIZE
			MOVS	R3, #NULL
			STRB	R3, [R1,#1]		; save null to NUM_ENQD
		
			POP		{R1-R3}
			BX		LR
			ENDP
			
;****************************************************************
;		Dequeue
;
;		Attempts to get a queue from the record.
;		
;		Parameters
;		Input:
;		R1: pointer for queue record structure
;
;		Output:
;		R0: contains the successfully dequeued character
;		PSR C Flag: 0 on success, 1 on failure 
;
;		Modify:
;		APSR
;		R0
;****************************************************************
Dequeue		PROC 	{R1-R14}
			PUSH 	{R1-R4}

			LDRB	R2, [R1, #NUM_ENQD]		; R2 is NUM_ENQD value
			CMP		R2, #0
			BEQ		EmptyQueue				; if 0, branch to set carry
			
			LDR		R3, [R1, #OUT_PTR]		; R3 is pointer to queue out addr
			LDRB	R0, [R3, #0]			; put character stored at maddr R3 in R0

			ADDS	R3, R3, #1				; increment OUT_PTR address
			LDR		R4, [R1, #BUF_PAST]		; R4 is BUF_PAST stored address
			CMP		R3, R4
			BNE	 	SaveOutPtr				; skip next line if OUT_PTR still within queue limit
			LDR		R3, [R1, #BUF_STRT]		; set R3 to BUF_STRT addr
SaveOutPtr	STR		R3, [R1, #OUT_PTR]		; store adjusted out pointer addr to OUT_PTR
			SUBS	R2, R2, #1				; decrement NUM_ENQD
			STRB	R2, [R1, #NUM_ENQD]		; store new NUM_ENQD
			
			MRS		R3, APSR				; move APSR to R3
			MOVS	R2, #0xDF
			LSLS	R2, #24					; set AND mask
			ANDS	R3, R3, R2				; clear C flag in APSR
			MSR		APSR, R3				; set APSR back
			B		Done

EmptyQueue	MRS		R3, APSR			; move APSR to R3
			MOVS	R2, #0x20
			LSLS	R2, #24				; set AND mask
			ORRS	R3, R3, R2			; set C flag in APSR
			MSR		APSR, R3			; set APSR back

Done		POP		{R1-R4}
			BX		LR
			ENDP


;****************************************************************
;		Enqueue
;
;		Attempts to insert a new character into the queue.
;		
;		Parameters
;		Input:
;		R0: character byte to enqueue
;		R1: pointer for queue record structure
;
;		Output:
;		PSR C Flag: 0 on success, 1 on failure 
;
;		Modify:
;		APSR
;****************************************************************
Enqueue		PROC 	{R1-R14}
			PUSH 	{R0-R4}

			LDRB	R2, [R1, #NUM_ENQD]		; R2 is NUM_ENQD value
			LDRB	R3, [R1, #BUF_SIZE]		; R3 is BUF_SIZE
			CMP		R2, R3
			BGE		QueueFull				; if 0, branch to set carry
			
			LDR		R3, [R1, #IN_PTR]		; R3 is pointer to queue in addr
			STRB	R0, [R3, #0]			; enqueue value stored in R0
			
			ADDS	R3, R3, #1				; increment IN_PTR address
			LDR		R4, [R1, #BUF_PAST]		; R4 is BUF_PAST stored address
			CMP		R3, R4
			BNE	 	SaveInPtr				; Skip next line if IN_PTR still within queue buffer
			LDR		R3, [R1, #BUF_STRT]		; set R3 to BUF_STRT addr
SaveInPtr	STR		R3, [R1, #IN_PTR]		; store adjusted in pointer addr to IN_PTR
			ADDS	R2, R2, #1				; increment NUM_ENQD
			STRB	R2, [R1, #NUM_ENQD]		; store new NUM_ENQD
			
			MRS		R3, APSR				; move APSR to R3
			MOVS	R2, #0xDF
			LSLS	R2, #24					; set AND mask
			ANDS	R3, R3, R2				; clear C flag in APSR
			MSR		APSR, R3				; set APSR back
			B		Finish

QueueFull	MRS		R3, APSR				; move APSR to R3
			MOVS	R2, #0x20	
			LSLS	R2, #24					; set AND mask
			ORRS	R3, R3, R2				; set C flag in APSR
			MSR		APSR, R3				; set APSR back

Finish		POP		{R0-R4}
			BX		LR
			ENDP


;****************************************************************
;		PutNumHex
;
;		Prints to the terminal screen the text
;		hexadecimal representation of the unsigned word in R0
;		
;		Parameters
;		Input:
;		R0: Unsigned number in hexadecimal
;
;		Modify:
;		APSR
;		
;		Uses:
;		PutChar
;****************************************************************
PutNumHex	PROC	{R0-R14}
			PUSH	{R0-R3, LR}

			MOVS	R2, #0x0F		; init mask in R2
			MOVS	R3, #0			; init counter
Loop		CMP		R3, #8			; compare counter to limit, 8
			BEQ		PrintOut		; Branch to print chars if limit reached
			MOVS	R1, R0			; move R0 to R1 for ANDS
			ANDS	R1, R1, R2		; put 4 MSBs in R1
			ADDS	R1, R1, #0x30	; Add 0x30 to put in number range
			CMP		R1, #0x39		; Check if value in number or letter range
			BLE		Number			; if < 0, skip the next line
			ADDS	R1, R1, #0x07	; if > 0, is letter, put character into letter range
Number		PUSH	{R1}			; push to stack

			LSRS	R0, R0, #4		; shift number over for next 4 MSBs
			ADDS	R3, R3, #1		; increment counter
			B		Loop

PrintOut	POP		{R0}
			BL		PutChar			; print the value			
			SUBS	R3, R3, #1		; increment counter
			BNE		PrintOut

RestoreRegs	POP		{R0-R3, PC}
			ENDP
		

;****************************************************************
;		PutNumUB
;
;		Prints to the terminal screen the text
;		decimal representation of the unsigned byte in R0
;		
;		Parameters
;		Input:
;		R0: Unsigned number in hexadecimal
;
;		Modify:
;		APSR
;		
;		Uses:
;		PutNumU
;****************************************************************
PutNumUB	PROC		{R0-R14}
			PUSH		{R0-R1, LR}
			
			MOVS		R1, #0xFF
			ANDS		R0, R0, R1
			BL			PutNumU

			POP			{R0-R1, PC}
			ENDP


;****************************************************************
;		PutNumU
;
;		Prints to the terminal screen the text
;		decimal representation of the number in R0
;		
;		Parameters
;		Input:
;		R0: Unsigned number in hexadecimal
;
;		Modify:
;		APSR
;		
;		Uses:
;		PutChar
;		DivU
;****************************************************************
PutNumU		PROC	{R0-R14}
			PUSH	{LR}
			PUSH	{R0-R3}

			MOVS	R3,#0			; init counter
DivTen		MOVS	R1,R0			; move R0 to R1 
			MOVS	R0,#0x0A		; put 10 in R0
			BL		DIVU			; R0 holds quotient, R1 holds remainder
			ADDS	R1,R1,#0x30		; put number into ascii range
			PUSH	{R1}			; push remainder onto stack
			ADDS	R3,R3,#1		; increment counter
			CMP		R0,#0
			BEQ		PrintNum		; when quotient = 0, str is fully converted
			B		DivTen

PrintNum	POP		{R0}			; pop the latest remainder into R0
			BL		PutChar			; send to terminal
			SUBS	R3,R3,#1		; subtract counter
			BNE		PrintNum		; loop until counter = 0

			POP		{R0-R3}
			POP		{PC}
			ENDP

	
;---------------------------------------------------------------
;	DIVISION SUBROUTINE
;		input:		R0 = Divisor
; 					R1 = Dividend
;		operation:	R1/R0
;		returns:	R0 = Quotient
;					R1 = Remainder
;---------------------------------------------------------------

DIVU	PROC 	{R2-R14},{}	
		PUSH	{R2-R3}
			
		MOVS	R2, #0x20 		; Store mask in R2
		LSLS	R2, #24			; Left shift to mask 2nd MSB (carry)
		MRS		R3, APSR		; Move APSR to R3
		ORRS	R2, R2, R3		; Or APSR to set C
		MSR		APSR, R2		; Set APSR back 

		CMP		R0, #0			; if R0 is 0
		BEQ		Div0			; Jump to divide by 0
		
		MOVS	R2, #0
DivL			CMP 	R1, R0	; while R1 > R0
		BLO		DivF				
		SUBS	R1, R1, R0		; R1 = R1 - R0
		ADDS	R2, R2, #1		; Add 1 to remainder
		B		DivL

DivF	MRS		R3, APSR		; Move APSR to R3
		BICS	R3, R3, R2		; Clear carry
		MSR		APSR, R3		; Move APSR back
		B		Fin				; Skip Div0

Div0	MOVS	R2, #0			; Clear mask in R2

Fin		MOVS	R0, R2			; Move quotient to R0
		POP		{R2-R3}			; Pop stored stack registers
		BX		LR				; Return
		ENDP

;>>>>>   end subroutine code <<<<<
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
				EXPORT	Count
				EXPORT	RunStopWatch
				EXPORT	RxQRecord
;>>>>> begin variables here <<<<<
TxQ             SPACE   QBUFSIZE
TxQRecord       SPACE   QRECSIZE
                ALIGN
RxQ             SPACE   QBUFSIZE
RxQRecord       SPACE   QRECSIZE
                ALIGN
MainQ           SPACE   QBUFSIZE
MainQRecord     SPACE   QRECSIZE
				ALIGN
RunStopWatch    SPACE	1
				ALIGN
Count		    SPACE	4
;>>>>>   end variables here <<<<<
            ALIGN
            END
