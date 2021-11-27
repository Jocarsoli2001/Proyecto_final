/*
 * File:   PROYECTO-FINAL.c
 * Author: José Santizo
 *
 * Creado el 24 de noviembre de 2021
 * 
 * Descripción: Garra que se controla con servo motores, permite guardar la posición 
 */

//---------------------Bits de configuración-------------------------------
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


//-----------------Librerías utilizadas en código-------------------- 
#include <xc.h>
#include <stdint.h>
#include <stdio.h>

//-----------------Definición de frecuencia de cristal---------------
#define _XTAL_FREQ 4000000

//-----------------------Constantes----------------------------------
//#define  valor_tmr0 248                         // valor_tmr0 = 156 (0.05 ms)

//-----------------------Variables------------------------------------
char cont = 0;
int limite = 0;
int valor_tmr0 = 0;
//limite mínimo = 14
//Límite máximo = 32

//------------Funciones sin retorno de variables----------------------
void setup(void);                               // Función de setup
void divisor(void);                             // Función para dividir números en dígitos
void tmr0(void);                                // Función para reiniciar TMR0
void displays(void);                            // Función para alternar valores mostrados en displays

//-------------Funciones que retornan variables-----------------------
int  tabla(int a);                              // Tabla para traducir valores a displays de 7 segmentos
int  tabla_p(int a);                            // Tabla que traduce valores a displays de 7 segmentos pero con punto decimal incluido

//----------------------Interrupciones--------------------------------
void __interrupt() isr(void){
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 1){                // Si el channel es 1 (puerto AN1)
            CCPR2L = (ADRESH>>1)+124;           // ADRESH = CCPR2L (duty cycle de 118 a 255)
            CCP2CONbits.DC2B1 = ADRESH & 0b01;  
            CCP2CONbits.DC2B0 = (ADRESL>>7);
            
        }
        else if (ADCON0bits.CHS == 0){          // Si input channel = 0 (puerto AN0)
            CCPR1L = (ADRESH>>1)+124;                 // ADRESH = CCPR1L (duty cycle de 131 a 255)
            CCP1CONbits.DC1B1 = ADRESH & 0b01;  
            CCP1CONbits.DC1B0 = (ADRESL>>7);
        } 
        else if (ADCON0bits.CHS == 2){
            limite = ADRESH;
        }
        PIR1bits.ADIF = 0;                      // Limpiar bander de interrupción ADC
    }
    if(T0IF){
        tmr0();                                 // Reiniciar TMR0
        if(PORTCbits.RC3 == 1){
           valor_tmr0 = 14;
           PORTCbits.RC3 = 0;
        }
        else{
            valor_tmr0 = (255-14);
            PORTCbits.RC3 = 1;
        }
    }
}

//----------------------Main Loop--------------------------------
void main(void) {
    setup();                                    // Subrutina de setup
    ADCON0bits.GO = 1;                          // Comenzar conversión ADC 
    while(1){
        if(ADCON0bits.GO == 0){                 // Si bit GO = 0
            if(ADCON0bits.CHS == 2){            // Si Input Channel = AN1
                ADCON0bits.CHS = 1;             // Asignar input Channel = AN0
                __delay_us(50);                 // Delay de 50 ms
            }
            else if (ADCON0bits.CHS == 1){      // Si Input Channel = AN0
                ADCON0bits.CHS = 0;             // Asignar Input Channel = AN1
                __delay_us(50);                 // Delay de 50 ms
            }
            else {
                ADCON0bits.CHS = 2;
                __delay_us(50);
            }
            __delay_us(50);
            ADCON0bits.GO = 1;                  // Asignar bit GO = 1
        }
        
    }
}

//----------------------Subrutinas--------------------------------
void setup(void){
    
    //Configuración de entradas y salidas
    ANSEL = 0b00000111;                         // Pines 0 y 1 de PORTA como analógicos
    ANSELH = 0;
    
    TRISA = 0b00000111;                         // PORTA, bit 0 y 1 como entrada analógica
    TRISC = 0;                                  // PORTC como salida
    TRISD = 0;                                  // PORTD como salida                           
    TRISE = 0;                                  // PORTE como salida
    
    PORTD = 0;                                  // Limpiar PORTD
    PORTC = 0;                                  // Limpiar PORTC
    PORTE = 0;                                  // Limpiar PORTE
    
    //Configuración de oscilador
    OSCCONbits.IRCF = 0b0110;                   // Oscilador a 8 MHz = 111
    OSCCONbits.SCS = 1;
    
    //Configuración de TMR0
    OPTION_REGbits.T0CS = 0;                    // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;                    // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;                     // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to the WDT
    OPTION_REGbits.PS2 = 1;                     // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = valor_tmr0;                          // preset for timer register
    
    //Configuración del ADC
    ADCON1bits.ADFM = 0;                        // Resultado justificado a la izquierda
    ADCON1bits.VCFG0 = 0;                       // Voltaje 0 de referencia = VSS
    ADCON1bits.VCFG1 = 0;                       // Voltaje 1 de referencia = VDD
    
    ADCON0bits.ADCS = 0b10;                     // Conversión ADC generada con FOSC/32
    ADCON0bits.CHS = 0;                         // Input Channel = AN0
    ADCON0bits.ADON = 1;                        // ADC = enabled
    __delay_us(200);
    
    //Configuración de interrupciones
    INTCONbits.T0IF = 0;                        // Habilitada la bandera de TIMER 0      
    INTCONbits.T0IE = 1;                        // Habilitar las interrupciones de TIMER 0
    INTCONbits.GIE = 1;                         // Habilitar interrupciones globales
    PIR1bits.ADIF = 0;                          // Limpiar bandera de interrupción del ADC
    PIE1bits.ADIE = 1;                          // Interrupción ADC = enabled
    INTCONbits.PEIE = 1;                        // Interrupciones periféricas activadas
    
    //Configuración de PWM
    TRISCbits.TRISC2 = 1;                       // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;                       // RC1/CCP2 como entrada
    PR2 = 255;                                  // Frecuencia de TMR2 = 2 us
    CCP1CONbits.P1M = 0;                        // Solo una salida en CCP1
    CCP1CONbits.CCP1M = 0b1100;                 // Modo de PWM
    CCP2CONbits.CCP2M = 0b1100;                 // Modo de PWM para CCP2
        
    CCPR1L = 0x0f;                               // Duty cicle inicial del PWM en CCP1 y CCP2
    CCPR2L = 0x0f;
    CCP2CONbits.DC2B0 = 0;                      // Bits menos significativos de CCP2
    CCP2CONbits.DC2B1 = 0;
    CCP1CONbits.DC1B = 0;                       // Bits menor significativos de CCP1
    
    //Configuración del Timer 2
    PIR1bits.TMR2IF = 0;                        // Limpiar bandera de TMR2
    T2CONbits.T2CKPS = 0b11;                    // Prescaler en 1:16
    T2CONbits.TMR2ON = 1;
    
    while(PIR1bits.TMR2IF == 0);                // Esperar un ciclo de TMR2
    PIR1bits.TMR2IF = 0;                        // Limpiar bandera de TMR2
    
    TRISCbits.TRISC2 = 0;                       // Salida 1 del PWM en RC2
    TRISCbits.TRISC1 = 0;                       // Salida 2 del PWM en RC1
    
    return;
}

//----------------------------Subrutinas----------------------------------------
void tmr0(void){
    INTCONbits.T0IF = 0;                        // Limpiar bandera de TIMER 0
    TMR0 = valor_tmr0;                          // TMR0 = 255
    return;
}



