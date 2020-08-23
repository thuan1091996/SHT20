
;CodeVisionAVR C Compiler V3.12 Advanced
;(C) Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com

;Build configuration    : Debug
;Chip type              : ATmega8A
;Program type           : Application
;Clock frequency        : 8.000000 MHz
;Memory model           : Small
;Optimize for           : Size
;(s)printf features     : int, width
;(s)scanf features      : int, width
;External RAM size      : 0
;Data Stack size        : 256 byte(s)
;Heap size              : 0 byte(s)
;Promote 'char' to 'int': Yes
;'char' is unsigned     : Yes
;8 bit enums            : Yes
;Global 'const' stored in FLASH: Yes
;Enhanced function parameter passing: Yes
;Enhanced core instructions: On
;Automatic register allocation for global variables: On
;Smart register allocation: On

	#define _MODEL_SMALL_

	#pragma AVRPART ADMIN PART_NAME ATmega8A
	#pragma AVRPART MEMORY PROG_FLASH 8192
	#pragma AVRPART MEMORY EEPROM 512
	#pragma AVRPART MEMORY INT_SRAM SIZE 1024
	#pragma AVRPART MEMORY INT_SRAM START_ADDR 0x60

	.LISTMAC
	.EQU UDRE=0x5
	.EQU RXC=0x7
	.EQU USR=0xB
	.EQU UDR=0xC
	.EQU SPSR=0xE
	.EQU SPDR=0xF
	.EQU EERE=0x0
	.EQU EEWE=0x1
	.EQU EEMWE=0x2
	.EQU EECR=0x1C
	.EQU EEDR=0x1D
	.EQU EEARL=0x1E
	.EQU EEARH=0x1F
	.EQU WDTCR=0x21
	.EQU MCUCR=0x35
	.EQU GICR=0x3B
	.EQU SPL=0x3D
	.EQU SPH=0x3E
	.EQU SREG=0x3F

	.DEF R0X0=R0
	.DEF R0X1=R1
	.DEF R0X2=R2
	.DEF R0X3=R3
	.DEF R0X4=R4
	.DEF R0X5=R5
	.DEF R0X6=R6
	.DEF R0X7=R7
	.DEF R0X8=R8
	.DEF R0X9=R9
	.DEF R0XA=R10
	.DEF R0XB=R11
	.DEF R0XC=R12
	.DEF R0XD=R13
	.DEF R0XE=R14
	.DEF R0XF=R15
	.DEF R0X10=R16
	.DEF R0X11=R17
	.DEF R0X12=R18
	.DEF R0X13=R19
	.DEF R0X14=R20
	.DEF R0X15=R21
	.DEF R0X16=R22
	.DEF R0X17=R23
	.DEF R0X18=R24
	.DEF R0X19=R25
	.DEF R0X1A=R26
	.DEF R0X1B=R27
	.DEF R0X1C=R28
	.DEF R0X1D=R29
	.DEF R0X1E=R30
	.DEF R0X1F=R31

	.EQU __SRAM_START=0x0060
	.EQU __SRAM_END=0x045F
	.EQU __DSTACK_SIZE=0x0100
	.EQU __HEAP_SIZE=0x0000
	.EQU __CLEAR_SRAM_SIZE=__SRAM_END-__SRAM_START+1

	.MACRO __CPD1N
	CPI  R30,LOW(@0)
	LDI  R26,HIGH(@0)
	CPC  R31,R26
	LDI  R26,BYTE3(@0)
	CPC  R22,R26
	LDI  R26,BYTE4(@0)
	CPC  R23,R26
	.ENDM

	.MACRO __CPD2N
	CPI  R26,LOW(@0)
	LDI  R30,HIGH(@0)
	CPC  R27,R30
	LDI  R30,BYTE3(@0)
	CPC  R24,R30
	LDI  R30,BYTE4(@0)
	CPC  R25,R30
	.ENDM

	.MACRO __CPWRR
	CP   R@0,R@2
	CPC  R@1,R@3
	.ENDM

	.MACRO __CPWRN
	CPI  R@0,LOW(@2)
	LDI  R30,HIGH(@2)
	CPC  R@1,R30
	.ENDM

	.MACRO __ADDB1MN
	SUBI R30,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDB2MN
	SUBI R26,LOW(-@0-(@1))
	.ENDM

	.MACRO __ADDW1MN
	SUBI R30,LOW(-@0-(@1))
	SBCI R31,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW2MN
	SUBI R26,LOW(-@0-(@1))
	SBCI R27,HIGH(-@0-(@1))
	.ENDM

	.MACRO __ADDW1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1FN
	SUBI R30,LOW(-2*@0-(@1))
	SBCI R31,HIGH(-2*@0-(@1))
	SBCI R22,BYTE3(-2*@0-(@1))
	.ENDM

	.MACRO __ADDD1N
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	SBCI R22,BYTE3(-@0)
	SBCI R23,BYTE4(-@0)
	.ENDM

	.MACRO __ADDD2N
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	SBCI R24,BYTE3(-@0)
	SBCI R25,BYTE4(-@0)
	.ENDM

	.MACRO __SUBD1N
	SUBI R30,LOW(@0)
	SBCI R31,HIGH(@0)
	SBCI R22,BYTE3(@0)
	SBCI R23,BYTE4(@0)
	.ENDM

	.MACRO __SUBD2N
	SUBI R26,LOW(@0)
	SBCI R27,HIGH(@0)
	SBCI R24,BYTE3(@0)
	SBCI R25,BYTE4(@0)
	.ENDM

	.MACRO __ANDBMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ANDWMNN
	LDS  R30,@0+(@1)
	ANDI R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ANDI R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ANDD1N
	ANDI R30,LOW(@0)
	ANDI R31,HIGH(@0)
	ANDI R22,BYTE3(@0)
	ANDI R23,BYTE4(@0)
	.ENDM

	.MACRO __ANDD2N
	ANDI R26,LOW(@0)
	ANDI R27,HIGH(@0)
	ANDI R24,BYTE3(@0)
	ANDI R25,BYTE4(@0)
	.ENDM

	.MACRO __ORBMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	.ENDM

	.MACRO __ORWMNN
	LDS  R30,@0+(@1)
	ORI  R30,LOW(@2)
	STS  @0+(@1),R30
	LDS  R30,@0+(@1)+1
	ORI  R30,HIGH(@2)
	STS  @0+(@1)+1,R30
	.ENDM

	.MACRO __ORD1N
	ORI  R30,LOW(@0)
	ORI  R31,HIGH(@0)
	ORI  R22,BYTE3(@0)
	ORI  R23,BYTE4(@0)
	.ENDM

	.MACRO __ORD2N
	ORI  R26,LOW(@0)
	ORI  R27,HIGH(@0)
	ORI  R24,BYTE3(@0)
	ORI  R25,BYTE4(@0)
	.ENDM

	.MACRO __DELAY_USB
	LDI  R24,LOW(@0)
__DELAY_USB_LOOP:
	DEC  R24
	BRNE __DELAY_USB_LOOP
	.ENDM

	.MACRO __DELAY_USW
	LDI  R24,LOW(@0)
	LDI  R25,HIGH(@0)
__DELAY_USW_LOOP:
	SBIW R24,1
	BRNE __DELAY_USW_LOOP
	.ENDM

	.MACRO __GETD1S
	LDD  R30,Y+@0
	LDD  R31,Y+@0+1
	LDD  R22,Y+@0+2
	LDD  R23,Y+@0+3
	.ENDM

	.MACRO __GETD2S
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	LDD  R24,Y+@0+2
	LDD  R25,Y+@0+3
	.ENDM

	.MACRO __PUTD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R31
	STD  Y+@0+2,R22
	STD  Y+@0+3,R23
	.ENDM

	.MACRO __PUTD2S
	STD  Y+@0,R26
	STD  Y+@0+1,R27
	STD  Y+@0+2,R24
	STD  Y+@0+3,R25
	.ENDM

	.MACRO __PUTDZ2
	STD  Z+@0,R26
	STD  Z+@0+1,R27
	STD  Z+@0+2,R24
	STD  Z+@0+3,R25
	.ENDM

	.MACRO __CLRD1S
	STD  Y+@0,R30
	STD  Y+@0+1,R30
	STD  Y+@0+2,R30
	STD  Y+@0+3,R30
	.ENDM

	.MACRO __POINTB1MN
	LDI  R30,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW1MN
	LDI  R30,LOW(@0+(@1))
	LDI  R31,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTD1M
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __POINTW1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	LDI  R22,BYTE3(2*@0+(@1))
	LDI  R23,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTB2MN
	LDI  R26,LOW(@0+(@1))
	.ENDM

	.MACRO __POINTW2MN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	.ENDM

	.MACRO __POINTW2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	.ENDM

	.MACRO __POINTD2FN
	LDI  R26,LOW(2*@0+(@1))
	LDI  R27,HIGH(2*@0+(@1))
	LDI  R24,BYTE3(2*@0+(@1))
	LDI  R25,BYTE4(2*@0+(@1))
	.ENDM

	.MACRO __POINTBRM
	LDI  R@0,LOW(@1)
	.ENDM

	.MACRO __POINTWRM
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __POINTBRMN
	LDI  R@0,LOW(@1+(@2))
	.ENDM

	.MACRO __POINTWRMN
	LDI  R@0,LOW(@2+(@3))
	LDI  R@1,HIGH(@2+(@3))
	.ENDM

	.MACRO __POINTWRFN
	LDI  R@0,LOW(@2*2+(@3))
	LDI  R@1,HIGH(@2*2+(@3))
	.ENDM

	.MACRO __GETD1N
	LDI  R30,LOW(@0)
	LDI  R31,HIGH(@0)
	LDI  R22,BYTE3(@0)
	LDI  R23,BYTE4(@0)
	.ENDM

	.MACRO __GETD2N
	LDI  R26,LOW(@0)
	LDI  R27,HIGH(@0)
	LDI  R24,BYTE3(@0)
	LDI  R25,BYTE4(@0)
	.ENDM

	.MACRO __GETB1MN
	LDS  R30,@0+(@1)
	.ENDM

	.MACRO __GETB1HMN
	LDS  R31,@0+(@1)
	.ENDM

	.MACRO __GETW1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	.ENDM

	.MACRO __GETD1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	LDS  R22,@0+(@1)+2
	LDS  R23,@0+(@1)+3
	.ENDM

	.MACRO __GETBRMN
	LDS  R@0,@1+(@2)
	.ENDM

	.MACRO __GETWRMN
	LDS  R@0,@2+(@3)
	LDS  R@1,@2+(@3)+1
	.ENDM

	.MACRO __GETWRZ
	LDD  R@0,Z+@2
	LDD  R@1,Z+@2+1
	.ENDM

	.MACRO __GETD2Z
	LDD  R26,Z+@0
	LDD  R27,Z+@0+1
	LDD  R24,Z+@0+2
	LDD  R25,Z+@0+3
	.ENDM

	.MACRO __GETB2MN
	LDS  R26,@0+(@1)
	.ENDM

	.MACRO __GETW2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	.ENDM

	.MACRO __GETD2MN
	LDS  R26,@0+(@1)
	LDS  R27,@0+(@1)+1
	LDS  R24,@0+(@1)+2
	LDS  R25,@0+(@1)+3
	.ENDM

	.MACRO __PUTB1MN
	STS  @0+(@1),R30
	.ENDM

	.MACRO __PUTW1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	.ENDM

	.MACRO __PUTD1MN
	STS  @0+(@1),R30
	STS  @0+(@1)+1,R31
	STS  @0+(@1)+2,R22
	STS  @0+(@1)+3,R23
	.ENDM

	.MACRO __PUTB1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRB
	.ENDM

	.MACRO __PUTW1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRW
	.ENDM

	.MACRO __PUTD1EN
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMWRD
	.ENDM

	.MACRO __PUTBR0MN
	STS  @0+(@1),R0
	.ENDM

	.MACRO __PUTBMRN
	STS  @0+(@1),R@2
	.ENDM

	.MACRO __PUTWMRN
	STS  @0+(@1),R@2
	STS  @0+(@1)+1,R@3
	.ENDM

	.MACRO __PUTBZR
	STD  Z+@1,R@0
	.ENDM

	.MACRO __PUTWZR
	STD  Z+@2,R@0
	STD  Z+@2+1,R@1
	.ENDM

	.MACRO __GETW1R
	MOV  R30,R@0
	MOV  R31,R@1
	.ENDM

	.MACRO __GETW2R
	MOV  R26,R@0
	MOV  R27,R@1
	.ENDM

	.MACRO __GETWRN
	LDI  R@0,LOW(@2)
	LDI  R@1,HIGH(@2)
	.ENDM

	.MACRO __PUTW1R
	MOV  R@0,R30
	MOV  R@1,R31
	.ENDM

	.MACRO __PUTW2R
	MOV  R@0,R26
	MOV  R@1,R27
	.ENDM

	.MACRO __ADDWRN
	SUBI R@0,LOW(-@2)
	SBCI R@1,HIGH(-@2)
	.ENDM

	.MACRO __ADDWRR
	ADD  R@0,R@2
	ADC  R@1,R@3
	.ENDM

	.MACRO __SUBWRN
	SUBI R@0,LOW(@2)
	SBCI R@1,HIGH(@2)
	.ENDM

	.MACRO __SUBWRR
	SUB  R@0,R@2
	SBC  R@1,R@3
	.ENDM

	.MACRO __ANDWRN
	ANDI R@0,LOW(@2)
	ANDI R@1,HIGH(@2)
	.ENDM

	.MACRO __ANDWRR
	AND  R@0,R@2
	AND  R@1,R@3
	.ENDM

	.MACRO __ORWRN
	ORI  R@0,LOW(@2)
	ORI  R@1,HIGH(@2)
	.ENDM

	.MACRO __ORWRR
	OR   R@0,R@2
	OR   R@1,R@3
	.ENDM

	.MACRO __EORWRR
	EOR  R@0,R@2
	EOR  R@1,R@3
	.ENDM

	.MACRO __GETWRS
	LDD  R@0,Y+@2
	LDD  R@1,Y+@2+1
	.ENDM

	.MACRO __PUTBSR
	STD  Y+@1,R@0
	.ENDM

	.MACRO __PUTWSR
	STD  Y+@2,R@0
	STD  Y+@2+1,R@1
	.ENDM

	.MACRO __MOVEWRR
	MOV  R@0,R@2
	MOV  R@1,R@3
	.ENDM

	.MACRO __INWR
	IN   R@0,@2
	IN   R@1,@2+1
	.ENDM

	.MACRO __OUTWR
	OUT  @2+1,R@1
	OUT  @2,R@0
	.ENDM

	.MACRO __CALL1MN
	LDS  R30,@0+(@1)
	LDS  R31,@0+(@1)+1
	ICALL
	.ENDM

	.MACRO __CALL1FN
	LDI  R30,LOW(2*@0+(@1))
	LDI  R31,HIGH(2*@0+(@1))
	RCALL __GETW1PF
	ICALL
	.ENDM

	.MACRO __CALL2EN
	PUSH R26
	PUSH R27
	LDI  R26,LOW(@0+(@1))
	LDI  R27,HIGH(@0+(@1))
	RCALL __EEPROMRDW
	POP  R27
	POP  R26
	ICALL
	.ENDM

	.MACRO __CALL2EX
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	RCALL __EEPROMRDD
	ICALL
	.ENDM

	.MACRO __GETW1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1STACK
	IN   R30,SPL
	IN   R31,SPH
	ADIW R30,@0+1
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z
	MOVW R30,R0
	.ENDM

	.MACRO __NBST
	BST  R@0,@1
	IN   R30,SREG
	LDI  R31,0x40
	EOR  R30,R31
	OUT  SREG,R30
	.ENDM


	.MACRO __PUTB1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SN
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNS
	LDD  R26,Y+@0
	LDD  R27,Y+@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMN
	LDS  R26,@0
	LDS  R27,@0+1
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1PMNS
	LDS  R26,@0
	LDS  R27,@0+1
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RN
	MOVW R26,R@0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RNS
	MOVW R26,R@0
	ADIW R26,@1
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RNS
	MOVW R26,R@0
	ADIW R26,@1
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RON
	MOV  R26,R@0
	MOV  R27,R@1
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	RCALL __PUTDP1
	.ENDM

	.MACRO __PUTB1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X,R30
	.ENDM

	.MACRO __PUTW1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1RONS
	MOV  R26,R@0
	MOV  R27,R@1
	ADIW R26,@2
	RCALL __PUTDP1
	.ENDM


	.MACRO __GETB1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R30,Z
	.ENDM

	.MACRO __GETB1HSX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	.ENDM

	.MACRO __GETW1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R31,Z
	MOV  R30,R0
	.ENDM

	.MACRO __GETD1SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R0,Z+
	LD   R1,Z+
	LD   R22,Z+
	LD   R23,Z
	MOVW R30,R0
	.ENDM

	.MACRO __GETB2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R26,X
	.ENDM

	.MACRO __GETW2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	.ENDM

	.MACRO __GETD2SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R1,X+
	LD   R24,X+
	LD   R25,X
	MOVW R26,R0
	.ENDM

	.MACRO __GETBRSX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	LD   R@0,Z
	.ENDM

	.MACRO __GETWRSX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	LD   R@0,Z+
	LD   R@1,Z
	.ENDM

	.MACRO __GETBRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	LD   R@0,X
	.ENDM

	.MACRO __GETWRSX2
	MOVW R26,R28
	SUBI R26,LOW(-@2)
	SBCI R27,HIGH(-@2)
	LD   R@0,X+
	LD   R@1,X
	.ENDM

	.MACRO __LSLW8SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	LD   R31,Z
	CLR  R30
	.ENDM

	.MACRO __PUTB1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __CLRW1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __CLRD1SX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	ST   X+,R30
	ST   X+,R30
	ST   X+,R30
	ST   X,R30
	.ENDM

	.MACRO __PUTB2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z,R26
	.ENDM

	.MACRO __PUTW2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z,R27
	.ENDM

	.MACRO __PUTD2SX
	MOVW R30,R28
	SUBI R30,LOW(-@0)
	SBCI R31,HIGH(-@0)
	ST   Z+,R26
	ST   Z+,R27
	ST   Z+,R24
	ST   Z,R25
	.ENDM

	.MACRO __PUTBSRX
	MOVW R30,R28
	SUBI R30,LOW(-@1)
	SBCI R31,HIGH(-@1)
	ST   Z,R@0
	.ENDM

	.MACRO __PUTWSRX
	MOVW R30,R28
	SUBI R30,LOW(-@2)
	SBCI R31,HIGH(-@2)
	ST   Z+,R@0
	ST   Z,R@1
	.ENDM

	.MACRO __PUTB1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X,R30
	.ENDM

	.MACRO __PUTW1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X,R31
	.ENDM

	.MACRO __PUTD1SNX
	MOVW R26,R28
	SUBI R26,LOW(-@0)
	SBCI R27,HIGH(-@0)
	LD   R0,X+
	LD   R27,X
	MOV  R26,R0
	SUBI R26,LOW(-@1)
	SBCI R27,HIGH(-@1)
	ST   X+,R30
	ST   X+,R31
	ST   X+,R22
	ST   X,R23
	.ENDM

	.MACRO __MULBRR
	MULS R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRRU
	MUL  R@0,R@1
	MOVW R30,R0
	.ENDM

	.MACRO __MULBRR0
	MULS R@0,R@1
	.ENDM

	.MACRO __MULBRRU0
	MUL  R@0,R@1
	.ENDM

	.MACRO __MULBNWRU
	LDI  R26,@2
	MUL  R26,R@0
	MOVW R30,R0
	MUL  R26,R@1
	ADD  R31,R0
	.ENDM

;NAME DEFINITIONS FOR GLOBAL VARIABLES ALLOCATED TO REGISTERS
	.DEF _System_fail=R4
	.DEF _System_fail_msb=R5
	.DEF __lcd_x=R7
	.DEF __lcd_y=R6
	.DEF __lcd_maxx=R9

	.CSEG
	.ORG 0x00

;START OF CODE MARKER
__START_OF_CODE:

;INTERRUPT VECTORS
	RJMP __RESET
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP 0x00
	RJMP _twi_int_handler
	RJMP 0x00

_tbl10_G102:
	.DB  0x10,0x27,0xE8,0x3,0x64,0x0,0xA,0x0
	.DB  0x1,0x0
_tbl16_G102:
	.DB  0x0,0x10,0x0,0x1,0x10,0x0,0x1,0x0

;REGISTER BIT VARIABLES INITIALIZATION
__REG_BIT_VARS:
	.DW  0x0000

;GLOBAL REGISTER VARIABLES INITIALIZATION
__REG_VARS:
	.DB  0x0,0x0

_0x0:
	.DB  0x4E,0x68,0x69,0x65,0x74,0x20,0x64,0x6F
	.DB  0x3A,0x20,0x0,0x44,0x6F,0x20,0x61,0x6D
	.DB  0x3A,0x20,0x0
_0x2000003:
	.DB  0x80,0xC0
_0x2020003:
	.DB  0x7
_0x2060060:
	.DB  0x1
_0x2060000:
	.DB  0x2D,0x4E,0x41,0x4E,0x0,0x49,0x4E,0x46
	.DB  0x0

__GLOBAL_INI_TBL:
	.DW  0x01
	.DW  0x02
	.DW  __REG_BIT_VARS*2

	.DW  0x02
	.DW  0x04
	.DW  __REG_VARS*2

	.DW  0x0B
	.DW  _0x7
	.DW  _0x0*2

	.DW  0x08
	.DW  _0x7+11
	.DW  _0x0*2+11

	.DW  0x02
	.DW  __base_y_G100
	.DW  _0x2000003*2

	.DW  0x01
	.DW  _twi_result
	.DW  _0x2020003*2

	.DW  0x01
	.DW  __seed_G103
	.DW  _0x2060060*2

_0xFFFFFFFF:
	.DW  0

#define __GLOBAL_INI_TBL_PRESENT 1

__RESET:
	CLI
	CLR  R30
	OUT  EECR,R30

;INTERRUPT VECTORS ARE PLACED
;AT THE START OF FLASH
	LDI  R31,1
	OUT  GICR,R31
	OUT  GICR,R30
	OUT  MCUCR,R30

;CLEAR R2-R14
	LDI  R24,(14-2)+1
	LDI  R26,2
	CLR  R27
__CLEAR_REG:
	ST   X+,R30
	DEC  R24
	BRNE __CLEAR_REG

;CLEAR SRAM
	LDI  R24,LOW(__CLEAR_SRAM_SIZE)
	LDI  R25,HIGH(__CLEAR_SRAM_SIZE)
	LDI  R26,__SRAM_START
__CLEAR_SRAM:
	ST   X+,R30
	SBIW R24,1
	BRNE __CLEAR_SRAM

;GLOBAL VARIABLES INITIALIZATION
	LDI  R30,LOW(__GLOBAL_INI_TBL*2)
	LDI  R31,HIGH(__GLOBAL_INI_TBL*2)
__GLOBAL_INI_NEXT:
	LPM  R24,Z+
	LPM  R25,Z+
	SBIW R24,0
	BREQ __GLOBAL_INI_END
	LPM  R26,Z+
	LPM  R27,Z+
	LPM  R0,Z+
	LPM  R1,Z+
	MOVW R22,R30
	MOVW R30,R0
__GLOBAL_INI_LOOP:
	LPM  R0,Z+
	ST   X+,R0
	SBIW R24,1
	BRNE __GLOBAL_INI_LOOP
	MOVW R30,R22
	RJMP __GLOBAL_INI_NEXT
__GLOBAL_INI_END:

;HARDWARE STACK POINTER INITIALIZATION
	LDI  R30,LOW(__SRAM_END-__HEAP_SIZE)
	OUT  SPL,R30
	LDI  R30,HIGH(__SRAM_END-__HEAP_SIZE)
	OUT  SPH,R30

;DATA STACK POINTER INITIALIZATION
	LDI  R28,LOW(__SRAM_START+__DSTACK_SIZE)
	LDI  R29,HIGH(__SRAM_START+__DSTACK_SIZE)

	RJMP _main

	.ESEG
	.ORG 0

	.DSEG
	.ORG 0x160

	.CSEG
;/*******************************************************
;This program was created by the
;CodeWizardAVR V3.12 Advanced
;Automatic Program Generator
;© Copyright 1998-2014 Pavel Haiduc, HP InfoTech s.r.l.
;http://www.hpinfotech.com
;
;
;//-----------------0.Documentation section--------------------//
;Project : SHT20 - ATMega8 communication using I2C display with LCD 16x2
;This program using ATMega8 I2C to read the temp and humility of SHT20
;Then display the value through LED 16x2
;Process: 	- Implementation LCD display  (OK)
;			- Implementation I2C		  (OK)
;			- Reading value of SHT20
;				a. Display receive value to LCD with TWI lib: Failed with TWI library, 3 receive data are the same
;				b. Display receive value to LCD with I2C lib
;Version : 1.0
;Date    : 7/03/2019
;Author  : Tran Minh Thuan (ItachiVN)
;Company : Viet Mold Machine
;
;Chip type               : ATmega8A
;Program type            : Application
;AVR Core Clock frequency: 8.000000 MHz
;Memory model            : Small
;External RAM size       : 0
;Data Stack size         : 256
;*******************************************************/
;//-----------------1. Pre-processor Directives Section --------------------//
;#include <mega8.h>
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x80
	.EQU __sm_mask=0x70
	.EQU __sm_powerdown=0x20
	.EQU __sm_powersave=0x30
	.EQU __sm_standby=0x60
	.EQU __sm_ext_standby=0x70
	.EQU __sm_adc_noise_red=0x10
	.SET power_ctrl_reg=mcucr
	#endif
;#include <alcd.h>       // Alphanumeric LCD library
;#include <delay.h>      // Delay library
;#include <twi.h>        // TWI library
;#include <stdio.h>
;#include <stdlib.h>
;//-----------------2. Global declaration section--------------------//
;#define ERROR_I2C_TIMEOUT                     998
;#define ERROR_BAD_CRC                         999
;#define SHT20_ADDR	                          0x40
;#define TRIGGER_TEMP_MEASURE_HOLD             0xE3
;#define TRIGGER_HUMD_MEASURE_HOLD             0xE5
;#define TRIGGER_TEMP_MEASURE_NOHOLD           0xF3
;#define TRIGGER_HUMD_MEASURE_NOHOLD           0xF5
;#define WRITE_USER_REG                        0xE6
;#define READ_USER_REG                         0xE7
;#define SOFT_RESET                            0xFE
;#define USER_REGISTER_RESOLUTION_MASK         0x81
;#define USER_REGISTER_RESOLUTION_RH12_TEMP14  0x00
;#define USER_REGISTER_RESOLUTION_RH8_TEMP12   0x01
;#define USER_REGISTER_RESOLUTION_RH10_TEMP13  0x80
;#define USER_REGISTER_RESOLUTION_RH11_TEMP11  0x81
;#define USER_REGISTER_END_OF_BATTERY          0x40
;#define USER_REGISTER_HEATER_ENABLED          0x04
;#define USER_REGISTER_DISABLE_OTP_RELOAD      0x02
;#define MAX_WAIT                              100
;#define DELAY_INTERVAL                        10
;#define SHIFTED_DIVISOR                       0x988000
;#define MAX_COUNTER                           (MAX_WAIT/DELAY_INTERVAL) // 100/10=10
;/* --------------------Global variable----------------------*/
;unsigned char Data_Recv[5]={0};			//Pointer to data receive of slave I2C
;unsigned char Data_Disp[16]={0};
;unsigned int System_fail=0;             //Monitor CRC fails
;/* --------------------Function decleration---------------------*/
;unsigned int Read_SHT20(unsigned char u8cmd);
;unsigned char CheckCRC(unsigned int message_from_sensor, unsigned char check_value_from_sensor);
;float ReadTemperature_SHT20(void);
;float ReadHumidity_SHT20(void);
;void Display_Number(unsigned char numb);
;/* --------------------Function definition---------------------*/
;void main(void)
; 0000 0048 {

	.CSEG
_main:
; .FSTART _main
; 0000 0049     twi_master_init(100); //TWI initialization 100 kHz
	LDI  R26,LOW(100)
	LDI  R27,0
	RCALL _twi_master_init
; 0000 004A     lcd_init(16);         //LCD initialization 16x2
	LDI  R26,LOW(16)
	RCALL _lcd_init
; 0000 004B     #asm("sei")           // Global enable interrupts
	sei
; 0000 004C 	while (1)
_0x3:
; 0000 004D     {
; 0000 004E         float Temp=0;
; 0000 004F         float Humi=0;
; 0000 0050         Temp=ReadTemperature_SHT20();
	RCALL SUBOPT_0x0
;	Temp -> Y+4
;	Humi -> Y+0
	RCALL _ReadTemperature_SHT20
	__PUTD1S 4
; 0000 0051         if(Temp!=0)
	RCALL SUBOPT_0x1
	RCALL __CPD10
	BREQ _0x6
; 0000 0052         {
; 0000 0053             lcd_gotoxy(0,0);
	RCALL SUBOPT_0x2
	LDI  R26,LOW(0)
	RCALL _lcd_gotoxy
; 0000 0054             lcd_puts("Nhiet do: ");
	__POINTW2MN _0x7,0
	RCALL _lcd_puts
; 0000 0055             ftoa(Temp,3,Data_Disp);
	RCALL SUBOPT_0x1
	RCALL SUBOPT_0x3
; 0000 0056             lcd_puts(Data_Disp);
; 0000 0057         }
; 0000 0058         else System_fail++;
	RJMP _0x8
_0x6:
	MOVW R30,R4
	ADIW R30,1
	MOVW R4,R30
; 0000 0059         Humi= ReadHumidity_SHT20();
_0x8:
	RCALL _ReadHumidity_SHT20
	RCALL SUBOPT_0x4
; 0000 005A         if(Humi!=0)
	RCALL SUBOPT_0x5
	RCALL __CPD10
	BREQ _0x9
; 0000 005B         {
; 0000 005C             lcd_gotoxy(0,1);
	RCALL SUBOPT_0x2
	LDI  R26,LOW(1)
	RCALL _lcd_gotoxy
; 0000 005D             lcd_puts("Do am: ");
	__POINTW2MN _0x7,11
	RCALL _lcd_puts
; 0000 005E             ftoa(Humi,3,Data_Disp);
	RCALL SUBOPT_0x5
	RCALL SUBOPT_0x3
; 0000 005F             lcd_puts(Data_Disp);
; 0000 0060         }
; 0000 0061         else System_fail++;
	RJMP _0xA
_0x9:
	MOVW R30,R4
	ADIW R30,1
	MOVW R4,R30
; 0000 0062         Display_Number(System_fail);
_0xA:
	MOV  R26,R4
	RCALL _Display_Number
; 0000 0063         delay_ms(100);
	LDI  R26,LOW(100)
	RCALL SUBOPT_0x6
; 0000 0064 
; 0000 0065     }
	ADIW R28,8
	RJMP _0x3
; 0000 0066 }
_0xB:
	RJMP _0xB
; .FEND

	.DSEG
_0x7:
	.BYTE 0x13
;/* Read SHT by sending proper command
;   Input: 8-bit command (Table 6- Datasheet)
;   Output: 14 -bit received from SHT20 or 0 if wrong data
;   Note:
;*/
;unsigned int Read_SHT20(unsigned char u8cmd)
; 0000 006D {

	.CSEG
_Read_SHT20:
; .FSTART _Read_SHT20
; 0000 006E     unsigned char Command_Send[5]={0};	  //Pointer to data array send to Slave I2C
; 0000 006F     unsigned int u16_datarecv=0;
; 0000 0070     Command_Send[0]=u8cmd;
	ST   -Y,R26
	SBIW R28,5
	RCALL SUBOPT_0x7
	LDI  R30,LOW(0)
	STD  Y+3,R30
	STD  Y+4,R30
	RCALL __SAVELOCR2
;	u8cmd -> Y+7
;	Command_Send -> Y+2
;	u16_datarecv -> R16,R17
	__GETWRN 16,17,0
	LDD  R30,Y+7
	STD  Y+2,R30
; 0000 0071     twi_master_trans(SHT20_ADDR,&Command_Send[0],2,0,0);                //send request
	LDI  R30,LOW(64)
	ST   -Y,R30
	MOVW R30,R28
	ADIW R30,3
	RCALL SUBOPT_0x8
	LDI  R30,LOW(2)
	ST   -Y,R30
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	RCALL SUBOPT_0x8
	LDI  R26,LOW(0)
	RCALL _twi_master_trans
; 0000 0072     delay_ms(100);
	LDI  R26,LOW(100)
	RCALL SUBOPT_0x6
; 0000 0073     if(twi_master_trans(SHT20_ADDR,&Command_Send[2],0,&Data_Recv[0],3)) //if data collect is true
	LDI  R30,LOW(64)
	ST   -Y,R30
	MOVW R30,R28
	ADIW R30,5
	RCALL SUBOPT_0x8
	RCALL SUBOPT_0x2
	LDI  R30,LOW(_Data_Recv)
	LDI  R31,HIGH(_Data_Recv)
	RCALL SUBOPT_0x8
	LDI  R26,LOW(3)
	RCALL _twi_master_trans
	CPI  R30,0
	BREQ _0xC
; 0000 0074     {
; 0000 0075         u16_datarecv=(Data_Recv[0]<<8)|Data_Recv[1];
	LDS  R27,_Data_Recv
	LDI  R26,LOW(0)
	__GETB1MN _Data_Recv,1
	LDI  R31,0
	OR   R30,R26
	OR   R31,R27
	MOVW R16,R30
; 0000 0076         if(CheckCRC(u16_datarecv,Data_Recv[2])!=0) return 0;            //return 0 if CRC is wrong
	ST   -Y,R17
	ST   -Y,R16
	__GETB2MN _Data_Recv,2
	RCALL _CheckCRC
	CPI  R30,0
	BREQ _0xD
	LDI  R30,LOW(0)
	LDI  R31,HIGH(0)
	RJMP _0x20E000A
; 0000 0077         else u16_datarecv=u16_datarecv&0xFFFC;                          //14-bit resolution
_0xD:
	ANDI R16,LOW(65532)
; 0000 0078     }
; 0000 0079     return u16_datarecv;
_0xC:
	MOVW R30,R16
_0x20E000A:
	RCALL __LOADLOCR2
	ADIW R28,8
	RET
; 0000 007A }
; .FEND
;
;/* Read SHT20 Temperature
;   Input: None
;   Output: Float number of Temperature
;   Note:
;*/
;float ReadTemperature_SHT20(void)
; 0000 0082 {
_ReadTemperature_SHT20:
; .FSTART _ReadTemperature_SHT20
; 0000 0083     float tempTemperature=0;
; 0000 0084     float realTemperature=0;
; 0000 0085     unsigned int rawTemperature = Read_SHT20(TRIGGER_TEMP_MEASURE_NOHOLD);
; 0000 0086     if(rawTemperature!=0) //No error CRC occur
	RCALL SUBOPT_0x0
	RCALL __SAVELOCR2
;	tempTemperature -> Y+6
;	realTemperature -> Y+2
;	rawTemperature -> R16,R17
	LDI  R26,LOW(243)
	RCALL SUBOPT_0x9
	BREQ _0xF
; 0000 0087     {
; 0000 0088          tempTemperature = rawTemperature * (175.72 / 65536.0);
	RCALL SUBOPT_0xA
	__GETD2N 0x3B2FB852
	RCALL SUBOPT_0xB
; 0000 0089          realTemperature = tempTemperature - 46.85;
	__GETD2N 0x423B6666
	RCALL SUBOPT_0xC
; 0000 008A     }
; 0000 008B     return (realTemperature);
_0xF:
	RJMP _0x20E0009
; 0000 008C }
; .FEND
;
;/* Read SHT20 Humidity
;   Input: None
;   Output: Float number of Humidity
;   Note:
;*/
;float ReadHumidity_SHT20(void)
; 0000 0094 {
_ReadHumidity_SHT20:
; .FSTART _ReadHumidity_SHT20
; 0000 0095     float tempRH=0;
; 0000 0096     float rh=0;
; 0000 0097     unsigned int rawHumidity = Read_SHT20(TRIGGER_HUMD_MEASURE_NOHOLD);
; 0000 0098     if(rawHumidity!=0)
	RCALL SUBOPT_0x0
	RCALL __SAVELOCR2
;	tempRH -> Y+6
;	rh -> Y+2
;	rawHumidity -> R16,R17
	LDI  R26,LOW(245)
	RCALL SUBOPT_0x9
	BREQ _0x10
; 0000 0099     {
; 0000 009A         tempRH = rawHumidity * (125.0 / 65536.0);
	RCALL SUBOPT_0xA
	__GETD2N 0x3AFA0000
	RCALL SUBOPT_0xB
; 0000 009B         rh = tempRH - 6.0;
	__GETD2N 0x40C00000
	RCALL SUBOPT_0xC
; 0000 009C     }
; 0000 009D     return (rh);
_0x10:
_0x20E0009:
	__GETD1S 2
	RCALL __LOADLOCR2
	ADIW R28,10
	RET
; 0000 009E }
; .FEND
;
;/* Display_Number through LCD
;   Input: numb to display
;   Output: none
;   Note: 0<numb<99
;*/
;void Display_Number(unsigned char numb)
; 0000 00A6 {
_Display_Number:
; .FSTART _Display_Number
; 0000 00A7     unsigned char tens=0,digit=0;
; 0000 00A8     tens=numb/10;
	ST   -Y,R26
	RCALL __SAVELOCR2
;	numb -> Y+2
;	tens -> R17
;	digit -> R16
	LDI  R17,0
	LDI  R16,0
	LDD  R26,Y+2
	LDI  R27,0
	LDI  R30,LOW(10)
	LDI  R31,HIGH(10)
	RCALL __DIVW21
	MOV  R17,R30
; 0000 00A9     digit=numb%(tens*10);
	LDD  R26,Y+2
	CLR  R27
	LDI  R30,LOW(10)
	MUL  R30,R17
	MOVW R30,R0
	RCALL __MODW21
	MOV  R16,R30
; 0000 00AA     lcd_gotoxy(14,1);
	LDI  R30,LOW(14)
	ST   -Y,R30
	LDI  R26,LOW(1)
	RCALL _lcd_gotoxy
; 0000 00AB     lcd_putchar(tens+'0');
	MOV  R26,R17
	SUBI R26,-LOW(48)
	RCALL _lcd_putchar
; 0000 00AC     lcd_gotoxy(15,1);
	LDI  R30,LOW(15)
	ST   -Y,R30
	LDI  R26,LOW(1)
	RCALL _lcd_gotoxy
; 0000 00AD     lcd_putchar(digit+'0');
	MOV  R26,R16
	SUBI R26,-LOW(48)
	RCALL _lcd_putchar
; 0000 00AE }
	RCALL __LOADLOCR2
	RJMP _0x20E0007
; .FEND
;
;unsigned char CheckCRC(unsigned int message_from_sensor, unsigned char check_value_from_sensor)
; 0000 00B1 {
_CheckCRC:
; .FSTART _CheckCRC
; 0000 00B2     unsigned long int remainder=0,divsor=0;
; 0000 00B3     unsigned char i=0;
; 0000 00B4     remainder = (unsigned long int)message_from_sensor << 8;
	ST   -Y,R26
	RCALL SUBOPT_0x0
	ST   -Y,R17
;	message_from_sensor -> Y+10
;	check_value_from_sensor -> Y+9
;	remainder -> Y+5
;	divsor -> Y+1
;	i -> R17
	LDI  R17,0
	LDD  R26,Y+10
	LDD  R27,Y+10+1
	CLR  R24
	CLR  R25
	LDI  R30,LOW(8)
	RCALL __LSLD12
	RCALL SUBOPT_0xD
; 0000 00B5     remainder |= check_value_from_sensor;
	LDD  R30,Y+9
	RCALL SUBOPT_0xE
	CLR  R31
	CLR  R22
	CLR  R23
	RCALL __ORD12
	RCALL SUBOPT_0xD
; 0000 00B6     divsor = SHIFTED_DIVISOR;
	__GETD1N 0x988000
	RCALL SUBOPT_0xF
; 0000 00B7     for(i = 0 ; i < 16 ; i++){
	LDI  R17,LOW(0)
_0x12:
	CPI  R17,16
	BRSH _0x13
; 0000 00B8         if(remainder & (unsigned long int)1 << (23 - i))
	MOV  R30,R17
	LDI  R31,0
	LDI  R26,LOW(23)
	LDI  R27,HIGH(23)
	RCALL __SWAPW12
	SUB  R30,R26
	SBC  R31,R27
	RCALL __CWD1
	__GETD2N 0x1
	RCALL __LSLD12
	RCALL SUBOPT_0xE
	RCALL __ANDD12
	RCALL __CPD10
	BREQ _0x14
; 0000 00B9 		{
; 0000 00BA             remainder ^= divsor;
	RCALL SUBOPT_0x10
	RCALL SUBOPT_0xE
	RCALL __XORD12
	RCALL SUBOPT_0xD
; 0000 00BB         }
; 0000 00BC         divsor >>= 1;
_0x14:
	RCALL SUBOPT_0x10
	RCALL __LSRD1
	RCALL SUBOPT_0xF
; 0000 00BD     }
	SUBI R17,-1
	RJMP _0x12
_0x13:
; 0000 00BE     return (unsigned char)remainder;
	LDD  R30,Y+5
	LDD  R17,Y+0
	ADIW R28,12
	RET
; 0000 00BF }
; .FEND
;
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x80
	.EQU __sm_mask=0x70
	.EQU __sm_powerdown=0x20
	.EQU __sm_powersave=0x30
	.EQU __sm_standby=0x60
	.EQU __sm_ext_standby=0x70
	.EQU __sm_adc_noise_red=0x10
	.SET power_ctrl_reg=mcucr
	#endif

	.DSEG

	.CSEG
__lcd_write_nibble_G100:
; .FSTART __lcd_write_nibble_G100
	ST   -Y,R26
	IN   R30,0x18
	ANDI R30,LOW(0xF0)
	MOV  R26,R30
	LD   R30,Y
	SWAP R30
	ANDI R30,0xF
	OR   R30,R26
	OUT  0x18,R30
	RCALL SUBOPT_0x11
	SBI  0x12,7
	RCALL SUBOPT_0x11
	CBI  0x12,7
	RCALL SUBOPT_0x11
	RJMP _0x20E0008
; .FEND
__lcd_write_data:
; .FSTART __lcd_write_data
	ST   -Y,R26
	LD   R26,Y
	RCALL __lcd_write_nibble_G100
    ld    r30,y
    swap  r30
    st    y,r30
	LD   R26,Y
	RCALL __lcd_write_nibble_G100
	__DELAY_USB 133
	RJMP _0x20E0008
; .FEND
_lcd_gotoxy:
; .FSTART _lcd_gotoxy
	ST   -Y,R26
	LD   R30,Y
	LDI  R31,0
	SUBI R30,LOW(-__base_y_G100)
	SBCI R31,HIGH(-__base_y_G100)
	LD   R30,Z
	LDD  R26,Y+1
	ADD  R26,R30
	RCALL __lcd_write_data
	LDD  R7,Y+1
	LDD  R6,Y+0
	ADIW R28,2
	RET
; .FEND
_lcd_clear:
; .FSTART _lcd_clear
	LDI  R26,LOW(2)
	RCALL __lcd_write_data
	LDI  R26,LOW(3)
	RCALL SUBOPT_0x6
	LDI  R26,LOW(12)
	RCALL __lcd_write_data
	LDI  R26,LOW(1)
	RCALL __lcd_write_data
	LDI  R26,LOW(3)
	RCALL SUBOPT_0x6
	LDI  R30,LOW(0)
	MOV  R6,R30
	MOV  R7,R30
	RET
; .FEND
_lcd_putchar:
; .FSTART _lcd_putchar
	ST   -Y,R26
	LD   R26,Y
	CPI  R26,LOW(0xA)
	BREQ _0x2000005
	CP   R7,R9
	BRLO _0x2000004
_0x2000005:
	RCALL SUBOPT_0x2
	INC  R6
	MOV  R26,R6
	RCALL _lcd_gotoxy
	LD   R26,Y
	CPI  R26,LOW(0xA)
	BREQ _0x20E0008
_0x2000004:
	INC  R7
	SBI  0x12,5
	LD   R26,Y
	RCALL __lcd_write_data
	CBI  0x12,5
	RJMP _0x20E0008
; .FEND
_lcd_puts:
; .FSTART _lcd_puts
	ST   -Y,R27
	ST   -Y,R26
	ST   -Y,R17
_0x2000008:
	LDD  R26,Y+1
	LDD  R27,Y+1+1
	LD   R30,X+
	STD  Y+1,R26
	STD  Y+1+1,R27
	MOV  R17,R30
	CPI  R30,0
	BREQ _0x200000A
	MOV  R26,R17
	RCALL _lcd_putchar
	RJMP _0x2000008
_0x200000A:
	RJMP _0x20E0006
; .FEND
_lcd_init:
; .FSTART _lcd_init
	ST   -Y,R26
	IN   R30,0x17
	ORI  R30,LOW(0xF)
	OUT  0x17,R30
	SBI  0x11,7
	SBI  0x11,5
	SBI  0x11,6
	CBI  0x12,7
	CBI  0x12,5
	CBI  0x12,6
	LDD  R9,Y+0
	LD   R30,Y
	SUBI R30,-LOW(128)
	__PUTB1MN __base_y_G100,2
	LD   R30,Y
	SUBI R30,-LOW(192)
	__PUTB1MN __base_y_G100,3
	LDI  R26,LOW(20)
	RCALL SUBOPT_0x6
	RCALL SUBOPT_0x12
	RCALL SUBOPT_0x12
	RCALL SUBOPT_0x12
	LDI  R26,LOW(32)
	RCALL __lcd_write_nibble_G100
	__DELAY_USW 200
	LDI  R26,LOW(40)
	RCALL __lcd_write_data
	LDI  R26,LOW(4)
	RCALL __lcd_write_data
	LDI  R26,LOW(133)
	RCALL __lcd_write_data
	LDI  R26,LOW(6)
	RCALL __lcd_write_data
	RCALL _lcd_clear
_0x20E0008:
	ADIW R28,1
	RET
; .FEND
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x80
	.EQU __sm_mask=0x70
	.EQU __sm_powerdown=0x20
	.EQU __sm_powersave=0x30
	.EQU __sm_standby=0x60
	.EQU __sm_ext_standby=0x70
	.EQU __sm_adc_noise_red=0x10
	.SET power_ctrl_reg=mcucr
	#endif

	.DSEG

	.CSEG
_twi_master_init:
; .FSTART _twi_master_init
	ST   -Y,R27
	ST   -Y,R26
	ST   -Y,R17
	SET
	BLD  R2,1
	LDI  R30,LOW(7)
	STS  _twi_result,R30
	LDI  R30,LOW(0)
	STS  _twi_slave_rx_handler_G101,R30
	STS  _twi_slave_rx_handler_G101+1,R30
	STS  _twi_slave_tx_handler_G101,R30
	STS  _twi_slave_tx_handler_G101+1,R30
	SBI  0x15,4
	SBI  0x15,5
	OUT  0x36,R30
	IN   R30,0x1
	ANDI R30,LOW(0xFC)
	OUT  0x1,R30
	LDD  R30,Y+1
	LDD  R31,Y+1+1
	LDI  R26,LOW(4000)
	LDI  R27,HIGH(4000)
	RCALL __DIVW21U
	MOV  R17,R30
	CPI  R17,8
	BRLO _0x2020004
	SUBI R17,LOW(8)
_0x2020004:
	OUT  0x0,R17
	IN   R30,0x36
	ANDI R30,LOW(0x80)
	ORI  R30,LOW(0x45)
	OUT  0x36,R30
_0x20E0006:
	LDD  R17,Y+0
_0x20E0007:
	ADIW R28,3
	RET
; .FEND
_twi_master_trans:
; .FSTART _twi_master_trans
	ST   -Y,R26
	SBIW R28,4
	SBRS R2,1
	RJMP _0x2020005
	LDD  R30,Y+10
	LSL  R30
	STS  _slave_address_G101,R30
	LDD  R30,Y+8
	LDD  R31,Y+8+1
	STS  _twi_tx_buffer_G101,R30
	STS  _twi_tx_buffer_G101+1,R31
	LDI  R30,LOW(0)
	STS  _twi_tx_index,R30
	LDD  R30,Y+7
	STS  _bytes_to_tx_G101,R30
	LDD  R30,Y+5
	LDD  R31,Y+5+1
	STS  _twi_rx_buffer_G101,R30
	STS  _twi_rx_buffer_G101+1,R31
	LDI  R30,LOW(0)
	STS  _twi_rx_index,R30
	LDD  R30,Y+4
	STS  _bytes_to_rx_G101,R30
	LDI  R30,LOW(6)
	STS  _twi_result,R30
	sei
	LDD  R30,Y+7
	CPI  R30,0
	BREQ _0x2020006
	LDD  R30,Y+8
	LDD  R31,Y+8+1
	SBIW R30,0
	BREQ _0x20E0005
	LDD  R30,Y+4
	CPI  R30,0
	BREQ _0x2020009
	LDD  R26,Y+5
	LDD  R27,Y+5+1
	SBIW R26,0
	BREQ _0x202000A
_0x2020009:
	RJMP _0x2020008
_0x202000A:
	RJMP _0x20E0005
_0x2020008:
	SET
	RJMP _0x2020065
_0x2020006:
	LDD  R30,Y+4
	CPI  R30,0
	BREQ _0x202000C
	LDD  R30,Y+5
	LDD  R31,Y+5+1
	SBIW R30,0
	BREQ _0x20E0005
	LDS  R30,_slave_address_G101
	ORI  R30,1
	STS  _slave_address_G101,R30
	CLT
_0x2020065:
	BLD  R2,0
	CLT
	BLD  R2,1
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xA0)
	OUT  0x36,R30
	__GETD1N 0x7A120
	RCALL SUBOPT_0x4
_0x202000E:
	SBRC R2,1
	RJMP _0x2020010
	RCALL SUBOPT_0x5
	SBIW R30,1
	SBCI R22,0
	SBCI R23,0
	RCALL SUBOPT_0x4
	BRNE _0x2020011
	LDI  R30,LOW(5)
	STS  _twi_result,R30
	SET
	BLD  R2,1
	RJMP _0x20E0005
_0x2020011:
	RJMP _0x202000E
_0x2020010:
_0x202000C:
	LDS  R26,_twi_result
	LDI  R30,LOW(0)
	RCALL __EQB12
	RJMP _0x20E0004
_0x2020005:
_0x20E0005:
	LDI  R30,LOW(0)
_0x20E0004:
	ADIW R28,11
	RET
; .FEND
_twi_int_handler:
; .FSTART _twi_int_handler
	ST   -Y,R0
	ST   -Y,R1
	ST   -Y,R15
	ST   -Y,R22
	ST   -Y,R23
	ST   -Y,R24
	ST   -Y,R25
	ST   -Y,R26
	ST   -Y,R27
	ST   -Y,R30
	ST   -Y,R31
	IN   R30,SREG
	ST   -Y,R30
	RCALL __SAVELOCR6
	LDS  R17,_twi_rx_index
	LDS  R16,_twi_tx_index
	LDS  R19,_bytes_to_tx_G101
	LDS  R18,_twi_result
	MOV  R30,R17
	LDS  R26,_twi_rx_buffer_G101
	LDS  R27,_twi_rx_buffer_G101+1
	LDI  R31,0
	ADD  R30,R26
	ADC  R31,R27
	MOVW R20,R30
	IN   R30,0x1
	ANDI R30,LOW(0xF8)
	CPI  R30,LOW(0x8)
	BRNE _0x2020017
	LDI  R18,LOW(0)
	RJMP _0x2020018
_0x2020017:
	CPI  R30,LOW(0x10)
	BRNE _0x2020019
_0x2020018:
	LDS  R30,_slave_address_G101
	RJMP _0x2020067
_0x2020019:
	CPI  R30,LOW(0x18)
	BREQ _0x202001D
	CPI  R30,LOW(0x28)
	BRNE _0x202001E
_0x202001D:
	CP   R16,R19
	BRSH _0x202001F
	MOV  R30,R16
	SUBI R16,-1
	LDS  R26,_twi_tx_buffer_G101
	LDS  R27,_twi_tx_buffer_G101+1
	LDI  R31,0
	ADD  R26,R30
	ADC  R27,R31
	LD   R30,X
_0x2020067:
	OUT  0x3,R30
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
	OUT  0x36,R30
	RJMP _0x2020020
_0x202001F:
	LDS  R30,_bytes_to_rx_G101
	CP   R17,R30
	BRSH _0x2020021
	LDS  R30,_slave_address_G101
	ORI  R30,1
	STS  _slave_address_G101,R30
	CLT
	BLD  R2,0
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xA0)
	OUT  0x36,R30
	RJMP _0x2020016
_0x2020021:
	RJMP _0x2020022
_0x2020020:
	RJMP _0x2020016
_0x202001E:
	CPI  R30,LOW(0x50)
	BRNE _0x2020023
	IN   R30,0x3
	MOVW R26,R20
	ST   X,R30
	SUBI R17,-LOW(1)
	RJMP _0x2020024
_0x2020023:
	CPI  R30,LOW(0x40)
	BRNE _0x2020025
_0x2020024:
	LDS  R30,_bytes_to_rx_G101
	SUBI R30,LOW(1)
	CP   R17,R30
	BRLO _0x2020026
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
	RJMP _0x2020068
_0x2020026:
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
_0x2020068:
	OUT  0x36,R30
	RJMP _0x2020016
_0x2020025:
	CPI  R30,LOW(0x58)
	BRNE _0x2020028
	IN   R30,0x3
	MOVW R26,R20
	ST   X,R30
	SUBI R17,-LOW(1)
	RJMP _0x2020029
_0x2020028:
	CPI  R30,LOW(0x20)
	BRNE _0x202002A
_0x2020029:
	RJMP _0x202002B
_0x202002A:
	CPI  R30,LOW(0x30)
	BRNE _0x202002C
_0x202002B:
	RJMP _0x202002D
_0x202002C:
	CPI  R30,LOW(0x48)
	BRNE _0x202002E
_0x202002D:
	CPI  R18,0
	BRNE _0x202002F
	SBRS R2,0
	RJMP _0x2020030
	CP   R16,R19
	BRLO _0x2020032
	RJMP _0x2020033
_0x2020030:
	LDS  R30,_bytes_to_rx_G101
	CP   R17,R30
	BRSH _0x2020034
_0x2020032:
	LDI  R18,LOW(4)
_0x2020034:
_0x2020033:
_0x202002F:
_0x2020022:
	RJMP _0x2020069
_0x202002E:
	CPI  R30,LOW(0x38)
	BRNE _0x2020037
	LDI  R18,LOW(2)
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
	RJMP _0x202006A
_0x2020037:
	CPI  R30,LOW(0x68)
	BREQ _0x202003A
	CPI  R30,LOW(0x78)
	BRNE _0x202003B
_0x202003A:
	LDI  R18,LOW(2)
	RJMP _0x202003C
_0x202003B:
	CPI  R30,LOW(0x60)
	BREQ _0x202003F
	CPI  R30,LOW(0x70)
	BRNE _0x2020040
_0x202003F:
	LDI  R18,LOW(0)
_0x202003C:
	LDI  R17,LOW(0)
	CLT
	BLD  R2,0
	LDS  R30,_twi_rx_buffer_size_G101
	CPI  R30,0
	BRNE _0x2020041
	LDI  R18,LOW(1)
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
	RJMP _0x202006B
_0x2020041:
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
_0x202006B:
	OUT  0x36,R30
	RJMP _0x2020016
_0x2020040:
	CPI  R30,LOW(0x80)
	BREQ _0x2020044
	CPI  R30,LOW(0x90)
	BRNE _0x2020045
_0x2020044:
	SBRS R2,0
	RJMP _0x2020046
	LDI  R18,LOW(1)
	RJMP _0x2020047
_0x2020046:
	IN   R30,0x3
	MOVW R26,R20
	ST   X,R30
	SUBI R17,-LOW(1)
	LDS  R30,_twi_rx_buffer_size_G101
	CP   R17,R30
	BRSH _0x2020048
	LDS  R30,_twi_slave_rx_handler_G101
	LDS  R31,_twi_slave_rx_handler_G101+1
	SBIW R30,0
	BRNE _0x2020049
	LDI  R18,LOW(6)
	RJMP _0x2020047
_0x2020049:
	LDI  R26,LOW(0)
	__CALL1MN _twi_slave_rx_handler_G101,0
	CPI  R30,0
	BREQ _0x202004A
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
	OUT  0x36,R30
	RJMP _0x2020016
_0x202004A:
	RJMP _0x202004B
_0x2020048:
	SET
	BLD  R2,0
_0x202004B:
	RJMP _0x202004C
_0x2020045:
	CPI  R30,LOW(0x88)
	BRNE _0x202004D
_0x202004C:
	RJMP _0x202004E
_0x202004D:
	CPI  R30,LOW(0x98)
	BRNE _0x202004F
_0x202004E:
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
	OUT  0x36,R30
	RJMP _0x2020016
_0x202004F:
	CPI  R30,LOW(0xA0)
	BRNE _0x2020050
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
	OUT  0x36,R30
	SET
	BLD  R2,1
	LDS  R30,_twi_slave_rx_handler_G101
	LDS  R31,_twi_slave_rx_handler_G101+1
	SBIW R30,0
	BREQ _0x2020051
	LDI  R26,LOW(1)
	__CALL1MN _twi_slave_rx_handler_G101,0
	RJMP _0x2020052
_0x2020051:
	LDI  R18,LOW(6)
_0x2020052:
	RJMP _0x2020016
_0x2020050:
	CPI  R30,LOW(0xB0)
	BRNE _0x2020053
	LDI  R18,LOW(2)
	RJMP _0x2020054
_0x2020053:
	CPI  R30,LOW(0xA8)
	BRNE _0x2020055
_0x2020054:
	LDS  R30,_twi_slave_tx_handler_G101
	LDS  R31,_twi_slave_tx_handler_G101+1
	SBIW R30,0
	BREQ _0x2020056
	LDI  R26,LOW(0)
	__CALL1MN _twi_slave_tx_handler_G101,0
	MOV  R19,R30
	CPI  R30,0
	BREQ _0x2020058
	LDI  R18,LOW(0)
	RJMP _0x2020059
_0x2020056:
_0x2020058:
	LDI  R18,LOW(6)
	RJMP _0x2020047
_0x2020059:
	LDI  R16,LOW(0)
	CLT
	BLD  R2,0
	RJMP _0x202005A
_0x2020055:
	CPI  R30,LOW(0xB8)
	BRNE _0x202005B
_0x202005A:
	SBRS R2,0
	RJMP _0x202005C
	LDI  R18,LOW(1)
	RJMP _0x2020047
_0x202005C:
	MOV  R30,R16
	SUBI R16,-1
	LDS  R26,_twi_tx_buffer_G101
	LDS  R27,_twi_tx_buffer_G101+1
	LDI  R31,0
	ADD  R26,R30
	ADC  R27,R31
	LD   R30,X
	OUT  0x3,R30
	CP   R16,R19
	BRSH _0x202005D
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
	RJMP _0x202006C
_0x202005D:
	SET
	BLD  R2,0
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,0x80
_0x202006C:
	OUT  0x36,R30
	RJMP _0x2020016
_0x202005B:
	CPI  R30,LOW(0xC0)
	BREQ _0x2020060
	CPI  R30,LOW(0xC8)
	BRNE _0x2020061
_0x2020060:
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xC0)
	OUT  0x36,R30
	LDS  R30,_twi_slave_tx_handler_G101
	LDS  R31,_twi_slave_tx_handler_G101+1
	SBIW R30,0
	BREQ _0x2020062
	LDI  R26,LOW(1)
	__CALL1MN _twi_slave_tx_handler_G101,0
_0x2020062:
	RJMP _0x2020035
_0x2020061:
	CPI  R30,0
	BRNE _0x2020016
	LDI  R18,LOW(3)
_0x2020047:
_0x2020069:
	IN   R30,0x36
	ANDI R30,LOW(0xF)
	ORI  R30,LOW(0xD0)
_0x202006A:
	OUT  0x36,R30
_0x2020035:
	SET
	BLD  R2,1
_0x2020016:
	STS  _twi_rx_index,R17
	STS  _twi_tx_index,R16
	STS  _twi_result,R18
	STS  _bytes_to_tx_G101,R19
	RCALL __LOADLOCR6
	ADIW R28,6
	LD   R30,Y+
	OUT  SREG,R30
	LD   R31,Y+
	LD   R30,Y+
	LD   R27,Y+
	LD   R26,Y+
	LD   R25,Y+
	LD   R24,Y+
	LD   R23,Y+
	LD   R22,Y+
	LD   R15,Y+
	LD   R1,Y+
	LD   R0,Y+
	RETI
; .FEND
	#ifndef __SLEEP_DEFINED__
	#define __SLEEP_DEFINED__
	.EQU __se_bit=0x80
	.EQU __sm_mask=0x70
	.EQU __sm_powerdown=0x20
	.EQU __sm_powersave=0x30
	.EQU __sm_standby=0x60
	.EQU __sm_ext_standby=0x70
	.EQU __sm_adc_noise_red=0x10
	.SET power_ctrl_reg=mcucr
	#endif

	.CSEG

	.CSEG
_ftoa:
; .FSTART _ftoa
	ST   -Y,R27
	ST   -Y,R26
	SBIW R28,4
	RCALL SUBOPT_0x7
	LDI  R30,LOW(63)
	STD  Y+3,R30
	RCALL __SAVELOCR2
	LDD  R30,Y+11
	LDD  R31,Y+11+1
	CPI  R30,LOW(0xFFFF)
	LDI  R26,HIGH(0xFFFF)
	CPC  R31,R26
	BRNE _0x206000D
	RCALL SUBOPT_0x13
	__POINTW2FN _0x2060000,0
	RCALL _strcpyf
	RJMP _0x20E0003
_0x206000D:
	CPI  R30,LOW(0x7FFF)
	LDI  R26,HIGH(0x7FFF)
	CPC  R31,R26
	BRNE _0x206000C
	RCALL SUBOPT_0x13
	__POINTW2FN _0x2060000,1
	RCALL _strcpyf
	RJMP _0x20E0003
_0x206000C:
	LDD  R26,Y+12
	TST  R26
	BRPL _0x206000F
	RCALL SUBOPT_0x14
	RCALL __ANEGF1
	RCALL SUBOPT_0x15
	RCALL SUBOPT_0x16
	LDI  R30,LOW(45)
	ST   X,R30
_0x206000F:
	LDD  R26,Y+8
	CPI  R26,LOW(0x7)
	BRLO _0x2060010
	LDI  R30,LOW(6)
	STD  Y+8,R30
_0x2060010:
	LDD  R17,Y+8
_0x2060011:
	MOV  R30,R17
	SUBI R17,1
	CPI  R30,0
	BREQ _0x2060013
	RCALL SUBOPT_0x17
	RCALL SUBOPT_0x18
	RCALL SUBOPT_0x19
	RJMP _0x2060011
_0x2060013:
	RCALL SUBOPT_0x1A
	RCALL __ADDF12
	RCALL SUBOPT_0x15
	LDI  R17,LOW(0)
	__GETD1N 0x3F800000
	RCALL SUBOPT_0x19
_0x2060014:
	RCALL SUBOPT_0x1A
	RCALL __CMPF12
	BRLO _0x2060016
	RCALL SUBOPT_0x17
	RCALL SUBOPT_0x1B
	RCALL SUBOPT_0x19
	SUBI R17,-LOW(1)
	CPI  R17,39
	BRLO _0x2060017
	RCALL SUBOPT_0x13
	__POINTW2FN _0x2060000,5
	RCALL _strcpyf
	RJMP _0x20E0003
_0x2060017:
	RJMP _0x2060014
_0x2060016:
	CPI  R17,0
	BRNE _0x2060018
	RCALL SUBOPT_0x16
	LDI  R30,LOW(48)
	ST   X,R30
	RJMP _0x2060019
_0x2060018:
_0x206001A:
	MOV  R30,R17
	SUBI R17,1
	CPI  R30,0
	BREQ _0x206001C
	RCALL SUBOPT_0x17
	RCALL SUBOPT_0x18
	__GETD2N 0x3F000000
	RCALL __ADDF12
	MOVW R26,R30
	MOVW R24,R22
	RCALL _floor
	RCALL SUBOPT_0x19
	RCALL SUBOPT_0x1A
	RCALL __DIVF21
	RCALL __CFD1U
	MOV  R16,R30
	RCALL SUBOPT_0x16
	RCALL SUBOPT_0x1C
	RCALL SUBOPT_0x17
	RCALL __CWD1
	RCALL __CDF1
	RCALL __MULF12
	RCALL SUBOPT_0x1D
	RCALL __SWAPD12
	RCALL __SUBF12
	RCALL SUBOPT_0x15
	RJMP _0x206001A
_0x206001C:
_0x2060019:
	LDD  R30,Y+8
	CPI  R30,0
	BREQ _0x20E0002
	RCALL SUBOPT_0x16
	LDI  R30,LOW(46)
	ST   X,R30
_0x206001E:
	LDD  R30,Y+8
	SUBI R30,LOW(1)
	STD  Y+8,R30
	SUBI R30,-LOW(1)
	BREQ _0x2060020
	RCALL SUBOPT_0x1D
	RCALL SUBOPT_0x1B
	RCALL SUBOPT_0x15
	RCALL SUBOPT_0x14
	RCALL __CFD1U
	MOV  R16,R30
	RCALL SUBOPT_0x16
	RCALL SUBOPT_0x1C
	RCALL SUBOPT_0x1D
	RCALL __CWD1
	RCALL __CDF1
	RCALL __SWAPD12
	RCALL __SUBF12
	RCALL SUBOPT_0x15
	RJMP _0x206001E
_0x2060020:
_0x20E0002:
	LDD  R26,Y+6
	LDD  R27,Y+6+1
	LDI  R30,LOW(0)
	ST   X,R30
_0x20E0003:
	RCALL __LOADLOCR2
	ADIW R28,13
	RET
; .FEND

	.DSEG

	.CSEG

	.CSEG
_strcpyf:
; .FSTART _strcpyf
	ST   -Y,R27
	ST   -Y,R26
    ld   r30,y+
    ld   r31,y+
    ld   r26,y+
    ld   r27,y+
    movw r24,r26
strcpyf0:
	lpm  r0,z+
    st   x+,r0
    tst  r0
    brne strcpyf0
    movw r30,r24
    ret
; .FEND

	.CSEG

	.CSEG
_ftrunc:
; .FSTART _ftrunc
	RCALL __PUTPARD2
   ldd  r23,y+3
   ldd  r22,y+2
   ldd  r31,y+1
   ld   r30,y
   bst  r23,7
   lsl  r23
   sbrc r22,7
   sbr  r23,1
   mov  r25,r23
   subi r25,0x7e
   breq __ftrunc0
   brcs __ftrunc0
   cpi  r25,24
   brsh __ftrunc1
   clr  r26
   clr  r27
   clr  r24
__ftrunc2:
   sec
   ror  r24
   ror  r27
   ror  r26
   dec  r25
   brne __ftrunc2
   and  r30,r26
   and  r31,r27
   and  r22,r24
   rjmp __ftrunc1
__ftrunc0:
   clt
   clr  r23
   clr  r30
   clr  r31
   clr  r22
__ftrunc1:
   cbr  r22,0x80
   lsr  r23
   brcc __ftrunc3
   sbr  r22,0x80
__ftrunc3:
   bld  r23,7
   ld   r26,y+
   ld   r27,y+
   ld   r24,y+
   ld   r25,y+
   cp   r30,r26
   cpc  r31,r27
   cpc  r22,r24
   cpc  r23,r25
   bst  r25,7
   ret
; .FEND
_floor:
; .FSTART _floor
	RCALL __PUTPARD2
	RCALL __GETD2S0
	RCALL _ftrunc
	RCALL SUBOPT_0x4
    brne __floor1
__floor0:
	RCALL SUBOPT_0x5
	RJMP _0x20E0001
__floor1:
    brtc __floor0
	RCALL SUBOPT_0x5
	__GETD2N 0x3F800000
	RCALL __SUBF12
_0x20E0001:
	ADIW R28,4
	RET
; .FEND

	.DSEG
_twi_tx_index:
	.BYTE 0x1
_twi_rx_index:
	.BYTE 0x1
_twi_result:
	.BYTE 0x1
_Data_Recv:
	.BYTE 0x5
_Data_Disp:
	.BYTE 0x10
__base_y_G100:
	.BYTE 0x4
_slave_address_G101:
	.BYTE 0x1
_twi_tx_buffer_G101:
	.BYTE 0x2
_bytes_to_tx_G101:
	.BYTE 0x1
_twi_rx_buffer_G101:
	.BYTE 0x2
_bytes_to_rx_G101:
	.BYTE 0x1
_twi_rx_buffer_size_G101:
	.BYTE 0x1
_twi_slave_rx_handler_G101:
	.BYTE 0x2
_twi_slave_tx_handler_G101:
	.BYTE 0x2
__seed_G103:
	.BYTE 0x4

	.CSEG
;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:46 WORDS
SUBOPT_0x0:
	SBIW R28,8
	LDI  R30,LOW(0)
	ST   Y,R30
	STD  Y+1,R30
	STD  Y+2,R30
	STD  Y+3,R30
	STD  Y+4,R30
	STD  Y+5,R30
	STD  Y+6,R30
	STD  Y+7,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x1:
	__GETD1S 4
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x2:
	LDI  R30,LOW(0)
	ST   -Y,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:6 WORDS
SUBOPT_0x3:
	RCALL __PUTPARD1
	LDI  R30,LOW(3)
	ST   -Y,R30
	LDI  R26,LOW(_Data_Disp)
	LDI  R27,HIGH(_Data_Disp)
	RCALL _ftoa
	LDI  R26,LOW(_Data_Disp)
	LDI  R27,HIGH(_Data_Disp)
	RJMP _lcd_puts

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x4:
	RCALL __PUTD1S0
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:10 WORDS
SUBOPT_0x5:
	RCALL __GETD1S0
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x6:
	LDI  R27,0
	RJMP _delay_ms

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:3 WORDS
SUBOPT_0x7:
	LDI  R30,LOW(0)
	ST   Y,R30
	STD  Y+1,R30
	STD  Y+2,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 7 TIMES, CODE SIZE REDUCTION:4 WORDS
SUBOPT_0x8:
	ST   -Y,R31
	ST   -Y,R30
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x9:
	RCALL _Read_SHT20
	MOVW R16,R30
	MOV  R0,R16
	OR   R0,R17
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0xA:
	MOVW R30,R16
	CLR  R22
	CLR  R23
	RCALL __CDF1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:6 WORDS
SUBOPT_0xB:
	RCALL __MULF12
	__PUTD1S 6
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0xC:
	RCALL __SUBF12
	__PUTD1S 2
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:4 WORDS
SUBOPT_0xD:
	__PUTD1S 5
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:4 WORDS
SUBOPT_0xE:
	__GETD2S 5
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0xF:
	__PUTD1S 1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x10:
	__GETD1S 1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x11:
	__DELAY_USB 13
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:8 WORDS
SUBOPT_0x12:
	LDI  R26,LOW(48)
	RCALL __lcd_write_nibble_G100
	__DELAY_USW 200
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x13:
	LDD  R30,Y+6
	LDD  R31,Y+6+1
	RJMP SUBOPT_0x8

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:1 WORDS
SUBOPT_0x14:
	__GETD1S 9
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:10 WORDS
SUBOPT_0x15:
	__PUTD1S 9
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 5 TIMES, CODE SIZE REDUCTION:18 WORDS
SUBOPT_0x16:
	LDD  R26,Y+6
	LDD  R27,Y+6+1
	ADIW R26,1
	STD  Y+6,R26
	STD  Y+6+1,R27
	SBIW R26,1
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x17:
	__GETD2S 2
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x18:
	__GETD1N 0x3DCCCCCD
	RCALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 4 TIMES, CODE SIZE REDUCTION:7 WORDS
SUBOPT_0x19:
	__PUTD1S 2
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:12 WORDS
SUBOPT_0x1A:
	__GETD1S 2
	__GETD2S 9
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x1B:
	__GETD1N 0x41200000
	RCALL __MULF12
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 2 TIMES, CODE SIZE REDUCTION:2 WORDS
SUBOPT_0x1C:
	MOV  R30,R16
	SUBI R30,-LOW(48)
	ST   X,R30
	MOV  R30,R16
	LDI  R31,0
	RET

;OPTIMIZER ADDED SUBROUTINE, CALLED 3 TIMES, CODE SIZE REDUCTION:4 WORDS
SUBOPT_0x1D:
	__GETD2S 9
	RET


	.CSEG
_delay_ms:
	adiw r26,0
	breq __delay_ms1
__delay_ms0:
	__DELAY_USW 0x7D0
	wdr
	sbiw r26,1
	brne __delay_ms0
__delay_ms1:
	ret

__ANEGF1:
	SBIW R30,0
	SBCI R22,0
	SBCI R23,0
	BREQ __ANEGF10
	SUBI R23,0x80
__ANEGF10:
	RET

__ROUND_REPACK:
	TST  R21
	BRPL __REPACK
	CPI  R21,0x80
	BRNE __ROUND_REPACK0
	SBRS R30,0
	RJMP __REPACK
__ROUND_REPACK0:
	ADIW R30,1
	ADC  R22,R25
	ADC  R23,R25
	BRVS __REPACK1

__REPACK:
	LDI  R21,0x80
	EOR  R21,R23
	BRNE __REPACK0
	PUSH R21
	RJMP __ZERORES
__REPACK0:
	CPI  R21,0xFF
	BREQ __REPACK1
	LSL  R22
	LSL  R0
	ROR  R21
	ROR  R22
	MOV  R23,R21
	RET
__REPACK1:
	PUSH R21
	TST  R0
	BRMI __REPACK2
	RJMP __MAXRES
__REPACK2:
	RJMP __MINRES

__UNPACK:
	LDI  R21,0x80
	MOV  R1,R25
	AND  R1,R21
	LSL  R24
	ROL  R25
	EOR  R25,R21
	LSL  R21
	ROR  R24

__UNPACK1:
	LDI  R21,0x80
	MOV  R0,R23
	AND  R0,R21
	LSL  R22
	ROL  R23
	EOR  R23,R21
	LSL  R21
	ROR  R22
	RET

__CFD1U:
	SET
	RJMP __CFD1U0
__CFD1:
	CLT
__CFD1U0:
	PUSH R21
	RCALL __UNPACK1
	CPI  R23,0x80
	BRLO __CFD10
	CPI  R23,0xFF
	BRCC __CFD10
	RJMP __ZERORES
__CFD10:
	LDI  R21,22
	SUB  R21,R23
	BRPL __CFD11
	NEG  R21
	CPI  R21,8
	BRTC __CFD19
	CPI  R21,9
__CFD19:
	BRLO __CFD17
	SER  R30
	SER  R31
	SER  R22
	LDI  R23,0x7F
	BLD  R23,7
	RJMP __CFD15
__CFD17:
	CLR  R23
	TST  R21
	BREQ __CFD15
__CFD18:
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R23
	DEC  R21
	BRNE __CFD18
	RJMP __CFD15
__CFD11:
	CLR  R23
__CFD12:
	CPI  R21,8
	BRLO __CFD13
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R23
	SUBI R21,8
	RJMP __CFD12
__CFD13:
	TST  R21
	BREQ __CFD15
__CFD14:
	LSR  R23
	ROR  R22
	ROR  R31
	ROR  R30
	DEC  R21
	BRNE __CFD14
__CFD15:
	TST  R0
	BRPL __CFD16
	RCALL __ANEGD1
__CFD16:
	POP  R21
	RET

__CDF1U:
	SET
	RJMP __CDF1U0
__CDF1:
	CLT
__CDF1U0:
	SBIW R30,0
	SBCI R22,0
	SBCI R23,0
	BREQ __CDF10
	CLR  R0
	BRTS __CDF11
	TST  R23
	BRPL __CDF11
	COM  R0
	RCALL __ANEGD1
__CDF11:
	MOV  R1,R23
	LDI  R23,30
	TST  R1
__CDF12:
	BRMI __CDF13
	DEC  R23
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R1
	RJMP __CDF12
__CDF13:
	MOV  R30,R31
	MOV  R31,R22
	MOV  R22,R1
	PUSH R21
	RCALL __REPACK
	POP  R21
__CDF10:
	RET

__SWAPACC:
	PUSH R20
	MOVW R20,R30
	MOVW R30,R26
	MOVW R26,R20
	MOVW R20,R22
	MOVW R22,R24
	MOVW R24,R20
	MOV  R20,R0
	MOV  R0,R1
	MOV  R1,R20
	POP  R20
	RET

__UADD12:
	ADD  R30,R26
	ADC  R31,R27
	ADC  R22,R24
	RET

__NEGMAN1:
	COM  R30
	COM  R31
	COM  R22
	SUBI R30,-1
	SBCI R31,-1
	SBCI R22,-1
	RET

__SUBF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R25,0x80
	BREQ __ADDF129
	LDI  R21,0x80
	EOR  R1,R21

	RJMP __ADDF120

__ADDF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R25,0x80
	BREQ __ADDF129

__ADDF120:
	CPI  R23,0x80
	BREQ __ADDF128
__ADDF121:
	MOV  R21,R23
	SUB  R21,R25
	BRVS __ADDF1211
	BRPL __ADDF122
	RCALL __SWAPACC
	RJMP __ADDF121
__ADDF122:
	CPI  R21,24
	BRLO __ADDF123
	CLR  R26
	CLR  R27
	CLR  R24
__ADDF123:
	CPI  R21,8
	BRLO __ADDF124
	MOV  R26,R27
	MOV  R27,R24
	CLR  R24
	SUBI R21,8
	RJMP __ADDF123
__ADDF124:
	TST  R21
	BREQ __ADDF126
__ADDF125:
	LSR  R24
	ROR  R27
	ROR  R26
	DEC  R21
	BRNE __ADDF125
__ADDF126:
	MOV  R21,R0
	EOR  R21,R1
	BRMI __ADDF127
	RCALL __UADD12
	BRCC __ADDF129
	ROR  R22
	ROR  R31
	ROR  R30
	INC  R23
	BRVC __ADDF129
	RJMP __MAXRES
__ADDF128:
	RCALL __SWAPACC
__ADDF129:
	RCALL __REPACK
	POP  R21
	RET
__ADDF1211:
	BRCC __ADDF128
	RJMP __ADDF129
__ADDF127:
	SUB  R30,R26
	SBC  R31,R27
	SBC  R22,R24
	BREQ __ZERORES
	BRCC __ADDF1210
	COM  R0
	RCALL __NEGMAN1
__ADDF1210:
	TST  R22
	BRMI __ADDF129
	LSL  R30
	ROL  R31
	ROL  R22
	DEC  R23
	BRVC __ADDF1210

__ZERORES:
	CLR  R30
	CLR  R31
	CLR  R22
	CLR  R23
	POP  R21
	RET

__MINRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	SER  R23
	POP  R21
	RET

__MAXRES:
	SER  R30
	SER  R31
	LDI  R22,0x7F
	LDI  R23,0x7F
	POP  R21
	RET

__MULF12:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BREQ __ZERORES
	CPI  R25,0x80
	BREQ __ZERORES
	EOR  R0,R1
	SEC
	ADC  R23,R25
	BRVC __MULF124
	BRLT __ZERORES
__MULF125:
	TST  R0
	BRMI __MINRES
	RJMP __MAXRES
__MULF124:
	PUSH R0
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R17
	CLR  R18
	CLR  R25
	MUL  R22,R24
	MOVW R20,R0
	MUL  R24,R31
	MOV  R19,R0
	ADD  R20,R1
	ADC  R21,R25
	MUL  R22,R27
	ADD  R19,R0
	ADC  R20,R1
	ADC  R21,R25
	MUL  R24,R30
	RCALL __MULF126
	MUL  R27,R31
	RCALL __MULF126
	MUL  R22,R26
	RCALL __MULF126
	MUL  R27,R30
	RCALL __MULF127
	MUL  R26,R31
	RCALL __MULF127
	MUL  R26,R30
	ADD  R17,R1
	ADC  R18,R25
	ADC  R19,R25
	ADC  R20,R25
	ADC  R21,R25
	MOV  R30,R19
	MOV  R31,R20
	MOV  R22,R21
	MOV  R21,R18
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	POP  R0
	TST  R22
	BRMI __MULF122
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	RJMP __MULF123
__MULF122:
	INC  R23
	BRVS __MULF125
__MULF123:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__MULF127:
	ADD  R17,R0
	ADC  R18,R1
	ADC  R19,R25
	RJMP __MULF128
__MULF126:
	ADD  R18,R0
	ADC  R19,R1
__MULF128:
	ADC  R20,R25
	ADC  R21,R25
	RET

__DIVF21:
	PUSH R21
	RCALL __UNPACK
	CPI  R23,0x80
	BRNE __DIVF210
	TST  R1
__DIVF211:
	BRPL __DIVF219
	RJMP __MINRES
__DIVF219:
	RJMP __MAXRES
__DIVF210:
	CPI  R25,0x80
	BRNE __DIVF218
__DIVF217:
	RJMP __ZERORES
__DIVF218:
	EOR  R0,R1
	SEC
	SBC  R25,R23
	BRVC __DIVF216
	BRLT __DIVF217
	TST  R0
	RJMP __DIVF211
__DIVF216:
	MOV  R23,R25
	PUSH R17
	PUSH R18
	PUSH R19
	PUSH R20
	CLR  R1
	CLR  R17
	CLR  R18
	CLR  R19
	CLR  R20
	CLR  R21
	LDI  R25,32
__DIVF212:
	CP   R26,R30
	CPC  R27,R31
	CPC  R24,R22
	CPC  R20,R17
	BRLO __DIVF213
	SUB  R26,R30
	SBC  R27,R31
	SBC  R24,R22
	SBC  R20,R17
	SEC
	RJMP __DIVF214
__DIVF213:
	CLC
__DIVF214:
	ROL  R21
	ROL  R18
	ROL  R19
	ROL  R1
	ROL  R26
	ROL  R27
	ROL  R24
	ROL  R20
	DEC  R25
	BRNE __DIVF212
	MOVW R30,R18
	MOV  R22,R1
	POP  R20
	POP  R19
	POP  R18
	POP  R17
	TST  R22
	BRMI __DIVF215
	LSL  R21
	ROL  R30
	ROL  R31
	ROL  R22
	DEC  R23
	BRVS __DIVF217
__DIVF215:
	RCALL __ROUND_REPACK
	POP  R21
	RET

__CMPF12:
	TST  R25
	BRMI __CMPF120
	TST  R23
	BRMI __CMPF121
	CP   R25,R23
	BRLO __CMPF122
	BRNE __CMPF121
	CP   R26,R30
	CPC  R27,R31
	CPC  R24,R22
	BRLO __CMPF122
	BREQ __CMPF123
__CMPF121:
	CLZ
	CLC
	RET
__CMPF122:
	CLZ
	SEC
	RET
__CMPF123:
	SEZ
	CLC
	RET
__CMPF120:
	TST  R23
	BRPL __CMPF122
	CP   R25,R23
	BRLO __CMPF121
	BRNE __CMPF122
	CP   R30,R26
	CPC  R31,R27
	CPC  R22,R24
	BRLO __CMPF122
	BREQ __CMPF123
	RJMP __CMPF121

__ANDD12:
	AND  R30,R26
	AND  R31,R27
	AND  R22,R24
	AND  R23,R25
	RET

__ORD12:
	OR   R30,R26
	OR   R31,R27
	OR   R22,R24
	OR   R23,R25
	RET

__XORD12:
	EOR  R30,R26
	EOR  R31,R27
	EOR  R22,R24
	EOR  R23,R25
	RET

__ANEGW1:
	NEG  R31
	NEG  R30
	SBCI R31,0
	RET

__ANEGD1:
	COM  R31
	COM  R22
	COM  R23
	NEG  R30
	SBCI R31,-1
	SBCI R22,-1
	SBCI R23,-1
	RET

__LSLD12:
	TST  R30
	MOV  R0,R30
	MOVW R30,R26
	MOVW R22,R24
	BREQ __LSLD12R
__LSLD12L:
	LSL  R30
	ROL  R31
	ROL  R22
	ROL  R23
	DEC  R0
	BRNE __LSLD12L
__LSLD12R:
	RET

__LSRD1:
	LSR  R23
	ROR  R22
	ROR  R31
	ROR  R30
	RET

__CWD1:
	MOV  R22,R31
	ADD  R22,R22
	SBC  R22,R22
	MOV  R23,R22
	RET

__EQB12:
	CP   R30,R26
	LDI  R30,1
	BREQ __EQB12T
	CLR  R30
__EQB12T:
	RET

__DIVW21U:
	CLR  R0
	CLR  R1
	LDI  R25,16
__DIVW21U1:
	LSL  R26
	ROL  R27
	ROL  R0
	ROL  R1
	SUB  R0,R30
	SBC  R1,R31
	BRCC __DIVW21U2
	ADD  R0,R30
	ADC  R1,R31
	RJMP __DIVW21U3
__DIVW21U2:
	SBR  R26,1
__DIVW21U3:
	DEC  R25
	BRNE __DIVW21U1
	MOVW R30,R26
	MOVW R26,R0
	RET

__DIVW21:
	RCALL __CHKSIGNW
	RCALL __DIVW21U
	BRTC __DIVW211
	RCALL __ANEGW1
__DIVW211:
	RET

__MODW21:
	CLT
	SBRS R27,7
	RJMP __MODW211
	COM  R26
	COM  R27
	ADIW R26,1
	SET
__MODW211:
	SBRC R31,7
	RCALL __ANEGW1
	RCALL __DIVW21U
	MOVW R30,R26
	BRTC __MODW212
	RCALL __ANEGW1
__MODW212:
	RET

__CHKSIGNW:
	CLT
	SBRS R31,7
	RJMP __CHKSW1
	RCALL __ANEGW1
	SET
__CHKSW1:
	SBRS R27,7
	RJMP __CHKSW2
	COM  R26
	COM  R27
	ADIW R26,1
	BLD  R0,0
	INC  R0
	BST  R0,0
__CHKSW2:
	RET

__GETD1S0:
	LD   R30,Y
	LDD  R31,Y+1
	LDD  R22,Y+2
	LDD  R23,Y+3
	RET

__GETD2S0:
	LD   R26,Y
	LDD  R27,Y+1
	LDD  R24,Y+2
	LDD  R25,Y+3
	RET

__PUTD1S0:
	ST   Y,R30
	STD  Y+1,R31
	STD  Y+2,R22
	STD  Y+3,R23
	RET

__PUTPARD1:
	ST   -Y,R23
	ST   -Y,R22
	ST   -Y,R31
	ST   -Y,R30
	RET

__PUTPARD2:
	ST   -Y,R25
	ST   -Y,R24
	ST   -Y,R27
	ST   -Y,R26
	RET

__SWAPD12:
	MOV  R1,R24
	MOV  R24,R22
	MOV  R22,R1
	MOV  R1,R25
	MOV  R25,R23
	MOV  R23,R1

__SWAPW12:
	MOV  R1,R27
	MOV  R27,R31
	MOV  R31,R1

__SWAPB12:
	MOV  R1,R26
	MOV  R26,R30
	MOV  R30,R1
	RET

__CPD10:
	SBIW R30,0
	SBCI R22,0
	SBCI R23,0
	RET

__SAVELOCR6:
	ST   -Y,R21
__SAVELOCR5:
	ST   -Y,R20
__SAVELOCR4:
	ST   -Y,R19
__SAVELOCR3:
	ST   -Y,R18
__SAVELOCR2:
	ST   -Y,R17
	ST   -Y,R16
	RET

__LOADLOCR6:
	LDD  R21,Y+5
__LOADLOCR5:
	LDD  R20,Y+4
__LOADLOCR4:
	LDD  R19,Y+3
__LOADLOCR3:
	LDD  R18,Y+2
__LOADLOCR2:
	LDD  R17,Y+1
	LD   R16,Y
	RET

;END OF CODE MARKER
__END_OF_CODE:
