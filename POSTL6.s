; Archivo: POSTLAB6.s
; Dispositivo: PIC16F887
; Autor: Diego Aldana

; Programa: contador en hexadecimal cada segundo Y Señal cada 500ms
  

; Creado: 07/02/2022
; Ult. modificaciíon: 07/02/2022
    PROCESSOR 16F887
    #include <xc.inc>

    ; CONFIG1
	CONFIG  FOSC = INTRC_NOCLKOUT	; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
	CONFIG  WDTE = OFF		; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
	CONFIG  PWRTE = OFF		; Power-up Timer Enable bit (PWRT enabled)
	CONFIG  MCLRE = OFF		; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
	CONFIG  CP = OFF		; Code Protection bit (Program memory code protection is disabled)
	CONFIG  CPD = OFF		; Data Code Protection bit (Data memory code protection is disabled)

	CONFIG  BOREN = OFF		; Brown Out Reset Selection bits (BOR disabled)
	CONFIG  IESO = OFF		; Internal External Switchover bit (Internal/External Switchover mode is disabled)
	CONFIG  FCMEN = OFF		; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
	CONFIG  LVP = OFF		; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

    ; CONFIG2
	CONFIG  BOR4V = BOR40V		; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
	CONFIG  WRT = OFF		; Flash Program Memory Self Write Enable bits (Write protection off)
	
    ;------------------------- VALORES EN MEMORIA ------------------------------
    ; Status de las interrupciones
    PSECT udata_shr		    ; Memoria compartida
	WT:		DS 1	    
	ST:		DS 1	    
    
    ; Variables globales
	PSECT udata_bank0	    ; Memoria común
	VTMR1:		DS 1	    ; Varible tmr1
	NIBBLES:	DS 2	    ; multiplexor
	BANDERA:	DS 1	    
	HEXA:		DS 2	    ; aumento en hexadecimal

    ;------------------------------ MACROS -------------------------------------
    ; MACRO PARA TMR0
    VALOR_NTMR0 MACRO VALOR_TIMER
	BANKSEL TMR0		    ; Banco
	MOVLW   VALOR_TIMER	    ; Cargamos w con el valor del timer
	MOVWF   TMR0		    ; Enviamos a TMR0 
	BCF	T0IF		    ; Limpiamos
	endm
	
    ; MACRO PARA TMR1
    VALOR_NTMR1 MACRO TMR1_H, TMR1_L	 
	BANKSEL TMR1H		    ; Banco
	MOVLW   TMR1_H		    ; Cargamos a W el literal 
	MOVWF   TMR1H		    ; Colocamos al literal en TMR1H
	MOVLW   TMR1_L		    ; Cargamos a W otro literal
	MOVWF   TMR1L		    ; Colocamos al otro literal en TMR1L
	BCF	TMR1IF		    ; Limpiamos bandera de interrupción
	endm
	

      
    ;-------------------------- VECTOR RESET -----------------------------------
    PSECT resVect, class=CODE, abs, delta=2
    ORG 00h			    ; Posición 0000h para el reset
    resetVec:
        PAGESEL main		    
        GOTO    main

    ;-------------------- SUBRUTINAS DE INTERRUPCION ---------------------------
    ORG 04h			    ; Posición 0004h para las interrupciones
    PUSH:			
	MOVWF   WT		    ; Guardamos en W el valor de la variable W_T
	SWAPF   STATUS, W	    ; Hacemos swap de nibbles y guardamos en W
	MOVWF   ST		    ; Almacenamos W en variable ST
	
    ISR:			    ; Verificación de banderas de las interrupciones
    
	BTFSC	T0IF		    ; Int TMR0
	CALL	INT_TMR0	    
	
	BTFSC   TMR1IF		    ; Int TMR1
	CALL    INT_TMR1	     
	
	BTFSC   TMR2IF		    ; Int TMR2
	CALL    INT_TMR2	     
	
    POP:				
	SWAPF   ST, W		    ; Hacemos swap de nibbles y guardamos en W
	MOVWF   STATUS		    ; Trasladamos W al registro STATUS
	SWAPF   WT, F		    ; Hacemos swap de nibbles y guardamos en WT 
	SWAPF   WT, W		    ; Hacemos swap de nibbles y guardamos en W
	RETFIE
    
    ;------------------------ RUTINAS PRINCIPALES ------------------------------
    PSECT code, delta=2, abs
    ORG 100h	
    
    main:
	CALL	CONFIG_IO	    ; CoNfiguración de los puertos	
	CALL	CONFIG_CLK	    ; Configuración del oscilador
	CALL	CONFIG_INT	    ; Configuracion de interrupciones
	CALL	CONFIG_TMR0	    ; Configuración de TMR0
	CALL	CONFIG_TMR1	    ; Configuración de TMR1
	CALL	CONFIG_TMR2	    ; Configuración de TMR2
	BANKSEL PORTD		    ; Direccionamiento a banco 00
	CLRF	VTMR1
	
    loop:			    
	MOVF	VTMR1, W	    ; Colocamos el valor de la variable en W
	MOVWF	PORTA		    ; Se carga en el puerto A
	
	CALL	multiplex_config1   ; Rutina para reguar el max de los display
	CALL	multiplex_config2   ; Rutina de cambio a valor en hexadecimal
	
	GOTO	loop		    
	
    ;--------------------------- SUBRUTINAS -----------------------------------
     CONFIG_IO:
	BANKSEL ANSEL		    ; banco
	CLRF    ANSEL		    ; Configurar como digitales
	CLRF    ANSELH		    
	BANKSEL TRISA		    ; banco
	CLRF	TRISA		    ; Salidas
	CLRF	TRISC		    
	BCF	TRISD, 0	    
	BCF	TRISD, 1	    
	BCF	TRISB, 0	   
	BANKSEL PORTA		    ; banco
	CLRF    PORTA		    
	CLRF	PORTC		    
	CLRF	PORTD		    
	CLRF	PORTB		    
	return
	
     CONFIG_CLK:		    
	BANKSEL OSCCON	    
	BSF	OSCCON, 0	    ; reloj interno
	BSF	OSCCON, 4	    ; <110> 500 kHz
	BSF	OSCCON, 5
	BCF	OSCCON, 6	    
	return

    CONFIG_INT:
	BANKSEL	INTCON		    ; banco
	BSF	PEIE		    ; todas las interrupciones perifericas activadas
	BSF	GIE		    ; Todas las interrupciones no perifericas habilitadas
	BCF	T0IF		    ; Limpieza TMR0
	BCF	TMR1IF		    ; TMR1
	BCF	TMR2IF		    ; TMR2
	BANKSEL PIE1		    
	BSF	T0IE		    ; INT TMR0
	BSF	TMR1IE		    ; TMR1
	BSF     TMR2IE		    ; TMR2
	RETURN

    CONFIG_TMR0:
	BANKSEL OPTION_REG	    ; banco
	BCF	T0CS		    ; temporizador
	BCF	PSA		    ; prescaler
	BSF	PS2
	BSF	PS1
	BCF	PS0		    ; PS<2:0> -> 110 (Prescaler 1:128)
	
	VALOR_NTMR0 254			     ; Valor de reinicio
	return

    CONFIG_TMR1:
	BANKSEL T1CON		    ; banco
	BCF	TMR1CS		    ; reloj interno
	BCF	T1OSCEN		    ; Apagamos LP
	BSF	T1CKPS1		    ; Prescaler 1:8
	BSF	T1CKPS0
	BCF	TMR1GE		    ; TMR1 siempre contando
	BSF	TMR1ON		    ; Activamos 
	
	;Valor N para cada segundo 
	VALOR_NTMR0 0xC2, 0xF7	    ; valor de reinicio
	return
	
    CONFIG_TMR2:
	BANKSEL T2CON		    ; Dbanco
	BSF	T2CKPS1		    ; Prescaler 1:16
	BSF	T2CKPS0
	BSF	TOUTPS3		    ; Postscaler 1:16
	BSF	TOUTPS2
	BSF	TOUTPS1
	BSF	TOUTPS0
	BSF	TMR2ON		    ; Activamos 
	BANKSEL PR2		    
	MOVLW   244		    ;N = 244 para 500 ms
	MOVWF   PR2		    ;Cargamos a PR2
	return
	

    INT_TMR0:
	VALOR_NTMR0 254			    ; MACRO TMR0
	CALL	ESTADOS		    ; verificación física de valores
	return
	
    INT_TMR1:
	VALOR_NTMR1 0xC2, 0xF7		    ; ACRO TMR1
	INCF	VTMR1
	return

    INT_TMR2:
	BCF	TMR2IF		    ; Limpieza de bandera
	MOVLW   0x01		    
	XORWF   PORTB		    ; regulamos 1 bit mediante el XOR en puerto E
	return

    ESTADOS:
	BCF	PORTD, 0	    ; limpieza de leds
	BCF	PORTD, 1	    
	BTFSC   BANDERA, 0	    ; Verificamos bandera
	GOTO    DISPLAY_1	 

    DISPLAY_0:			
	    MOVF    HEXA, W	    ; W en variable para valores hexadecimales
	    MOVWF   PORTC	    ; verificación de valor en binario en puerto C
	    BSF	    PORTD, 1	    ; Valor hexadecimal en port D
	    BSF	    BANDERA, 0	    ; configuración del valor de la bandera
	return

    DISPLAY_1:
	    MOVF    HEXA+1, W	    ; W en variable para valores hexadecimales
	    MOVWF   PORTC	    ; verificación de valor en binario en puerto C
	    BSF	    PORTD, 0	    ; Activamos el segundo display
	    BCF	    BANDERA, 0	    ; configuración del valor de la bandera
	return
	
    multiplex_config1:
	MOVLW   0x0F		    ; Colocar el valor 0x0F max del primer display hexadecimal
	ANDWF   VTMR1, W	    ; relación de AND con el contador para verificar el valor
	MOVWF   NIBBLES		    ; Almacenar el valor de W en variable NIBBLES posición 0
	
	MOVLW   0xF0		    ; Colocar el valor 0xF0 max del segundo display hexadecimal
	ANDWF   VTMR1, W	    ; relación de AND con el contador para verificar el valor
	MOVWF   NIBBLES+1	    ; Almacenar el valor de W en variable NIBBLES posición 1
	SWAPF   NIBBLES+1, F	    ; Hacer un SWAP de nibbles de la variable NIBBLES posición 1
	return
	
    multiplex_config2:
	MOVF    NIBBLES, W	    ; regreso del valor del nibble a W
	CALL    TABLA		    ; verificación de los bits para config en hexadecimal
	MOVWF   HEXA		    ; se guarda el valor de tabla en la variable para el primer display

	MOVF    NIBBLES+1, W	    ; regreso del valor del nibble a W
	CALL    TABLA		    ; verificación de los bits para config en hexadecimal
	MOVWF   HEXA+1		    ; se guarda el valor de tabla en la variable para el segundo display
	return
	
    
	
	
    ;------------------------ TABLA  HEXADECIMAL -------------------------------
    ORG 200h
    TABLA:
	CLRF    PCLATH		; Limpiamos registro PCLATH
	BSF	PCLATH, 1	; Posicionamos el PC en dirección 02xxh
	ANDLW   0x0F		; no saltar más del tamaño de la tabla
	ADDWF   PCL		; Apuntamos el PC a caracter en ASCII de CONT
	RETLW   00111111B	; 0
	RETLW   00000110B	; 1
	RETLW   01011011B	; 2
	RETLW   01001111B	; 3
	RETLW   01100110B	; 4
	RETLW   01101101B	; 5
	RETLW   01111101B	; 6
	RETLW   00000111B	; 7
	RETLW   01111111B	; 8
	RETLW   01101111B	; 9
	RETLW   01110111B	; A
	RETLW   01111100B	; b
	RETLW   00111001B	; C
	RETLW   01011110B	; d
	RETLW   01111001B	; E
	RETLW   01110001B	; F
    END



