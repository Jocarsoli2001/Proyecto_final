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
int tmr0_pos = 7;
int limite_transf = 0;

int ADC_in = 0;
int ADC_map = 0;

// Posiciones de Servo (Con TMR0)
// Limite mín Teórico = 14 / Experimental = 7
// Límite max Teórico = 32 / Experimental = 37
int tmr0_pos_min = 7;
int tmr0_pos_max = 37;

// Posiciones de Servo (Con CCP)
// Limite mín Teórico = 50 / Experimental = 35
// Límite max Teórico = 100 / Experimental = 150
int ccp_pos_min = 35;
int ccp_pos_max = 150;

// Settings de Filtro
int no_muestras_max = 64;          // No. de muestras a promediar
int muestras = 0;                   // Contador del número de muestras acumuladas
int limite_filter = 0;              // Valor final resultante del filtrado
int limite_acum = 0;                // Suma de todas las muestras (muestras acumuladas)

//------------Funciones sin retorno de variables----------------------
void setup(void);                               // Función de setup
void divisor(void);                             // Función para dividir números en dígitos
void tmr0(void);                                // Función para reiniciar TMR0
void displays(void);                            // Función para alternar valores mostrados en displays

// Mapeo de valores
void map2TMR0ServoRange(void);                  // Función para mapear de un rango de 0-255 a 7-37
void map2CCPServoRange(void);                   // Función para mapear de un rango de 0-255 a 35-150

//----------------------Interrupciones--------------------------------
void __interrupt() isr(void){
    
    // ====================
    // ADC INTERRUPT
    // ====================
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 1){                    // Si el channel es 1 (puerto AN1)
            ADC_in = ADRESH;
            map2CCPServoRange();
            CCPR2L = ADC_map;
            
            //CCPR2L = (ADRESH>>1)+124;             // ADRESH = CCPR2L (duty cycle de 118 a 255)
            CCP2CONbits.DC2B1 = ADRESH & 0b01;  
            CCP2CONbits.DC2B0 = (ADRESL>>7);
            
        }
        else if (ADCON0bits.CHS == 0){              // Si input channel = 0 (puerto AN0)
            ADC_in = ADRESH;
            map2CCPServoRange();
            CCPR1L = ADC_map;
            
            //CCPR1L = (ADRESH>>1)+124;             // ADRESH = CCPR1L (duty cycle de 131 a 255)
            CCP1CONbits.DC1B1 = ADRESH & 0b01;  
            CCP1CONbits.DC1B0 = (ADRESL>>7);
        } 
        else if (ADCON0bits.CHS == 2){
            limite = ADRESH;                        // Lee el ADC
            
            // Output Range: (7, 37)
            // Input Range: (0, 255)
            map2TMR0ServoRange();                       // Mapea del rango del ADC al del Servo
            
            // Toma de muestras
            if (muestras < no_muestras_max){
                limite_acum = limite_acum + limite_transf;
                muestras ++;
            }
            // Cálculo de promedio de muestras
            else {
                // Promedia las muestras y resetea las variables asociadas
                limite_filter = limite_acum / no_muestras_max;
                limite_acum = 0;
                muestras = 0;
            }
            
            
        }
        PIR1bits.ADIF = 0;                      // Limpiar bander de interrupción ADC
    }
    
    // ====================
    // TIMER 0 INTERRUPT
    // ====================
    if(T0IF){
        
        if(PORTCbits.RC3 == 1){
           valor_tmr0 = limite_filter;
           PORTCbits.RC3 = 0;
        }
        else{
            valor_tmr0 = (255 - limite_filter);
            PORTCbits.RC3 = 1;
        }
        
        tmr0();                                 // Reiniciar TMR0
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
                __delay_us(50);                 // Delay de 50 us
            }
            else if (ADCON0bits.CHS == 1){      // Si Input Channel = AN0
                ADCON0bits.CHS = 0;             // Asignar Input Channel = AN1
                __delay_us(50);                 // Delay de 50 us
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
    OPTION_REGbits.PSA = 0;                     // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to Timer0 module
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
    
    // Configuración de PWM (CCP)
    // Periodo para servos (Tpwm) = 2 ms
    TRISCbits.TRISC2 = 1;                       // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;                       // RC1/CCP2 como entrada
    CCP1CONbits.P1M = 0b00;                     // Solo una salida en CCP1
    CCP1CONbits.CCP1M = 0b1100;                 // Modo de PWM para CCP1
    CCP2CONbits.CCP2M = 0b1100;                 // Modo de PWM para CCP2
    
    //CCPR1H = 0;
    CCPR1L = 150;                              // Duty cicle inicial del PWM en CCP1 y CCP2
    CCPR2L = 150;
    CCP2CONbits.DC2B0 = 0;                      // Bits menos significativos de CCP2
    CCP2CONbits.DC2B1 = 0;
    CCP1CONbits.DC1B = 0;                       // Bits menor significativos de CCP1
    
    // Configuración del Timer 2
    // PR2 = Tpwm / (4 * Prescaler * Postscaler * Tosc)
    //     = 2 ms / (4 * 16 * 5 * 4 MHz)
    //     = 249
    PR2 = 249;                                  // Frecuencia de TMR2 = 2 us
    PIR1bits.TMR2IF = 0;                        // Limpiar bandera de TMR2
    T2CONbits.T2CKPS = 0b11;                    // Prescaler en 1:16. El LSB no es importante, puede ser 0 o 1.
    T2CONbits.TOUTPS = 0b0100;                  // Postscaler en 1:5
    T2CONbits.TMR2ON = 1;
    
    while(PIR1bits.TMR2IF == 0);                // Esperar un ciclo de TMR2
    PIR1bits.TMR2IF = 0;                        // Limpiar bandera de TMR2
    
    // Puerto C
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

void map2TMR0ServoRange(void){
    
    if (limite >= 0 && limite < 9)           { limite_transf = tmr0_pos_min; }
    else if (limite >= 9 && limite < 17)     { limite_transf = tmr0_pos_min + 1; }
    else if (limite >= 17 && limite < 25)    { limite_transf = tmr0_pos_min + 2; }
    else if (limite >= 25 && limite < 34)    { limite_transf = tmr0_pos_min + 3; }
    else if (limite >= 34 && limite < 42)    { limite_transf = tmr0_pos_min + 4; }
    else if (limite >= 42 && limite < 51)    { limite_transf = tmr0_pos_min + 5; }
    else if (limite >= 51 && limite < 59)    { limite_transf = tmr0_pos_min + 6; }
    else if (limite >= 59 && limite < 68)    { limite_transf = tmr0_pos_min + 7; }
    else if (limite >= 68 && limite < 76)    { limite_transf = tmr0_pos_min + 8; }
    else if (limite >= 76 && limite < 85)    { limite_transf = tmr0_pos_min + 9; }
    else if (limite >= 85 && limite < 93)    { limite_transf = tmr0_pos_min + 10; }
    else if (limite >= 93 && limite < 102)   { limite_transf = tmr0_pos_min + 11; }
    else if (limite >= 102 && limite < 110)  { limite_transf = tmr0_pos_min + 12; }
    else if (limite >= 110 && limite < 119)  { limite_transf = tmr0_pos_min + 13; }
    else if (limite >= 119 && limite < 127)  { limite_transf = tmr0_pos_min + 14; }
    else if (limite >= 127 && limite < 136)  { limite_transf = tmr0_pos_min + 15; }
    else if (limite >= 136 && limite < 144)  { limite_transf = tmr0_pos_min + 16; }
    else if (limite >= 144 && limite < 153)  { limite_transf = tmr0_pos_min + 17; }
    else if (limite >= 153 && limite < 161)  { limite_transf = tmr0_pos_min + 18; }
    else if (limite >= 161 && limite < 170)  { limite_transf = tmr0_pos_min + 19; }
    else if (limite >= 170 && limite < 178)  { limite_transf = tmr0_pos_min + 20; }
    else if (limite >= 178 && limite < 187)  { limite_transf = tmr0_pos_min + 21; }
    else if (limite >= 187 && limite < 195)  { limite_transf = tmr0_pos_min + 22; }
    else if (limite >= 195 && limite < 204)  { limite_transf = tmr0_pos_min + 23; }
    else if (limite >= 204 && limite < 212)  { limite_transf = tmr0_pos_min + 24; }
    else if (limite >= 212 && limite < 221)  { limite_transf = tmr0_pos_min + 25; }
    else if (limite >= 221 && limite < 229)  { limite_transf = tmr0_pos_min + 26; }
    else if (limite >= 229 && limite < 238)  { limite_transf = tmr0_pos_min + 27; }
    else if (limite >= 238 && limite < 246)  { limite_transf = tmr0_pos_min + 28; }
    else if (limite >= 246 && limite < 255)  { limite_transf = tmr0_pos_min + 29; }
    else { limite_transf = tmr0_pos_max; }
    
    return;
}

void map2CCPServoRange(void){
    
    if (ADC_in >= 0 && ADC_in < 2)         { ADC_map = ccp_pos_min; }
    else if (ADC_in >= 2 && ADC_in < 4)    { ADC_map = ccp_pos_min + 1; }
    else if (ADC_in >= 4 && ADC_in < 6)    { ADC_map = ccp_pos_min + 2; }
    else if (ADC_in >= 6 && ADC_in < 8)    { ADC_map = ccp_pos_min + 3; }
    else if (ADC_in >= 8 && ADC_in < 11)   { ADC_map = ccp_pos_min + 4; }
    else if (ADC_in >= 11 && ADC_in < 13)  { ADC_map = ccp_pos_min + 5; }
    else if (ADC_in >= 13 && ADC_in < 15)  { ADC_map = ccp_pos_min + 6; }
    else if (ADC_in >= 15 && ADC_in < 17)  { ADC_map = ccp_pos_min + 7; }
    else if (ADC_in >= 17 && ADC_in < 19)  { ADC_map = ccp_pos_min + 8; }
    else if (ADC_in >= 19 && ADC_in < 22)  { ADC_map = ccp_pos_min + 9; }
    else if (ADC_in >= 22 && ADC_in < 24)  { ADC_map = ccp_pos_min + 10; }
    else if (ADC_in >= 24 && ADC_in < 26)  { ADC_map = ccp_pos_min + 11; }
    else if (ADC_in >= 26 && ADC_in < 28)  { ADC_map = ccp_pos_min + 12; }
    else if (ADC_in >= 28 && ADC_in < 31)  { ADC_map = ccp_pos_min + 13; }
    else if (ADC_in >= 31 && ADC_in < 33)  { ADC_map = ccp_pos_min + 14; }
    else if (ADC_in >= 33 && ADC_in < 35)  { ADC_map = ccp_pos_min + 15; }
    else if (ADC_in >= 35 && ADC_in < 37)  { ADC_map = ccp_pos_min + 16; }
    else if (ADC_in >= 37 && ADC_in < 39)  { ADC_map = ccp_pos_min + 17; }
    else if (ADC_in >= 39 && ADC_in < 42)  { ADC_map = ccp_pos_min + 18; }
    else if (ADC_in >= 42 && ADC_in < 44)  { ADC_map = ccp_pos_min + 19; }
    else if (ADC_in >= 44 && ADC_in < 46)  { ADC_map = ccp_pos_min + 20; }
    else if (ADC_in >= 46 && ADC_in < 48)  { ADC_map = ccp_pos_min + 21; }
    else if (ADC_in >= 48 && ADC_in < 51)  { ADC_map = ccp_pos_min + 22; }
    else if (ADC_in >= 51 && ADC_in < 53)  { ADC_map = ccp_pos_min + 23; }
    else if (ADC_in >= 53 && ADC_in < 55)  { ADC_map = ccp_pos_min + 24; }
    else if (ADC_in >= 55 && ADC_in < 57)  { ADC_map = ccp_pos_min + 25; }
    else if (ADC_in >= 57 && ADC_in < 59)  { ADC_map = ccp_pos_min + 26; }
    else if (ADC_in >= 59 && ADC_in < 62)  { ADC_map = ccp_pos_min + 27; }
    else if (ADC_in >= 62 && ADC_in < 64)  { ADC_map = ccp_pos_min + 28; }
    else if (ADC_in >= 64 && ADC_in < 66)  { ADC_map = ccp_pos_min + 29; }
    else if (ADC_in >= 66 && ADC_in < 68)  { ADC_map = ccp_pos_min + 30; }
    else if (ADC_in >= 68 && ADC_in < 70)  { ADC_map = ccp_pos_min + 31; }
    else if (ADC_in >= 70 && ADC_in < 73)  { ADC_map = ccp_pos_min + 32; }
    else if (ADC_in >= 73 && ADC_in < 75)  { ADC_map = ccp_pos_min + 33; }
    else if (ADC_in >= 75 && ADC_in < 77)  { ADC_map = ccp_pos_min + 34; }
    else if (ADC_in >= 77 && ADC_in < 79)  { ADC_map = ccp_pos_min + 35; }
    else if (ADC_in >= 79 && ADC_in < 82)  { ADC_map = ccp_pos_min + 36; }
    else if (ADC_in >= 82 && ADC_in < 84)  { ADC_map = ccp_pos_min + 37; }
    else if (ADC_in >= 84 && ADC_in < 86)  { ADC_map = ccp_pos_min + 38; }
    else if (ADC_in >= 86 && ADC_in < 88)  { ADC_map = ccp_pos_min + 39; }
    else if (ADC_in >= 88 && ADC_in < 90)  { ADC_map = ccp_pos_min + 40; }
    else if (ADC_in >= 90 && ADC_in < 93)  { ADC_map = ccp_pos_min + 41; }
    else if (ADC_in >= 93 && ADC_in < 95)  { ADC_map = ccp_pos_min + 42; }
    else if (ADC_in >= 95 && ADC_in < 97)  { ADC_map = ccp_pos_min + 43; }
    else if (ADC_in >= 97 && ADC_in < 99)  { ADC_map = ccp_pos_min + 44; }
    else if (ADC_in >= 99 && ADC_in < 102)   { ADC_map = ccp_pos_min + 45; }
    else if (ADC_in >= 102 && ADC_in < 104)  { ADC_map = ccp_pos_min + 46; }
    else if (ADC_in >= 104 && ADC_in < 106)  { ADC_map = ccp_pos_min + 47; }
    else if (ADC_in >= 106 && ADC_in < 108)  { ADC_map = ccp_pos_min + 48; }
    else if (ADC_in >= 108 && ADC_in < 110)  { ADC_map = ccp_pos_min + 49; }
    else if (ADC_in >= 110 && ADC_in < 113)  { ADC_map = ccp_pos_min + 50; }
    else if (ADC_in >= 113 && ADC_in < 115)  { ADC_map = ccp_pos_min + 51; }
    else if (ADC_in >= 115 && ADC_in < 117)  { ADC_map = ccp_pos_min + 52; }
    else if (ADC_in >= 117 && ADC_in < 119)  { ADC_map = ccp_pos_min + 53; }
    else if (ADC_in >= 119 && ADC_in < 121)  { ADC_map = ccp_pos_min + 54; }
    else if (ADC_in >= 121 && ADC_in < 124)  { ADC_map = ccp_pos_min + 55; }
    else if (ADC_in >= 124 && ADC_in < 126)  { ADC_map = ccp_pos_min + 56; }
    else if (ADC_in >= 126 && ADC_in < 128)  { ADC_map = ccp_pos_min + 57; }
    else if (ADC_in >= 128 && ADC_in < 130)  { ADC_map = ccp_pos_min + 58; }
    else if (ADC_in >= 130 && ADC_in < 133)  { ADC_map = ccp_pos_min + 59; }
    else if (ADC_in >= 133 && ADC_in < 135)  { ADC_map = ccp_pos_min + 60; }
    else if (ADC_in >= 135 && ADC_in < 137)  { ADC_map = ccp_pos_min + 61; }
    else if (ADC_in >= 137 && ADC_in < 139)  { ADC_map = ccp_pos_min + 62; }
    else if (ADC_in >= 139 && ADC_in < 141)  { ADC_map = ccp_pos_min + 63; }
    else if (ADC_in >= 141 && ADC_in < 144)  { ADC_map = ccp_pos_min + 64; }
    else if (ADC_in >= 144 && ADC_in < 146)  { ADC_map = ccp_pos_min + 65; }
    else if (ADC_in >= 146 && ADC_in < 148)  { ADC_map = ccp_pos_min + 66; }
    else if (ADC_in >= 148 && ADC_in < 150)  { ADC_map = ccp_pos_min + 67; }
    else if (ADC_in >= 150 && ADC_in < 153)  { ADC_map = ccp_pos_min + 68; }
    else if (ADC_in >= 153 && ADC_in < 155)  { ADC_map = ccp_pos_min + 69; }
    else if (ADC_in >= 155 && ADC_in < 157)  { ADC_map = ccp_pos_min + 70; }
    else if (ADC_in >= 157 && ADC_in < 159)  { ADC_map = ccp_pos_min + 71; }
    else if (ADC_in >= 159 && ADC_in < 161)  { ADC_map = ccp_pos_min + 72; }
    else if (ADC_in >= 161 && ADC_in < 164)  { ADC_map = ccp_pos_min + 73; }
    else if (ADC_in >= 164 && ADC_in < 166)  { ADC_map = ccp_pos_min + 74; }
    else if (ADC_in >= 166 && ADC_in < 168)  { ADC_map = ccp_pos_min + 75; }
    else if (ADC_in >= 168 && ADC_in < 170)  { ADC_map = ccp_pos_min + 76; }
    else if (ADC_in >= 170 && ADC_in < 172)  { ADC_map = ccp_pos_min + 77; }
    else if (ADC_in >= 172 && ADC_in < 175)  { ADC_map = ccp_pos_min + 78; }
    else if (ADC_in >= 175 && ADC_in < 177)  { ADC_map = ccp_pos_min + 79; }
    else if (ADC_in >= 177 && ADC_in < 179)  { ADC_map = ccp_pos_min + 80; }
    else if (ADC_in >= 179 && ADC_in < 181)  { ADC_map = ccp_pos_min + 81; }
    else if (ADC_in >= 181 && ADC_in < 184)  { ADC_map = ccp_pos_min + 82; }
    else if (ADC_in >= 184 && ADC_in < 186)  { ADC_map = ccp_pos_min + 83; }
    else if (ADC_in >= 186 && ADC_in < 188)  { ADC_map = ccp_pos_min + 84; }
    else if (ADC_in >= 188 && ADC_in < 190)  { ADC_map = ccp_pos_min + 85; }
    else if (ADC_in >= 190 && ADC_in < 192)  { ADC_map = ccp_pos_min + 86; }
    else if (ADC_in >= 192 && ADC_in < 195)  { ADC_map = ccp_pos_min + 87; }
    else if (ADC_in >= 195 && ADC_in < 197)  { ADC_map = ccp_pos_min + 88; }
    else if (ADC_in >= 197 && ADC_in < 199)  { ADC_map = ccp_pos_min + 89; }
    else if (ADC_in >= 199 && ADC_in < 201)  { ADC_map = ccp_pos_min + 90; }
    else if (ADC_in >= 201 && ADC_in < 204)  { ADC_map = ccp_pos_min + 91; }
    else if (ADC_in >= 204 && ADC_in < 206)  { ADC_map = ccp_pos_min + 92; }
    else if (ADC_in >= 206 && ADC_in < 208)  { ADC_map = ccp_pos_min + 93; }
    else if (ADC_in >= 208 && ADC_in < 210)  { ADC_map = ccp_pos_min + 94; }
    else if (ADC_in >= 210 && ADC_in < 212)  { ADC_map = ccp_pos_min + 95; }
    else if (ADC_in >= 212 && ADC_in < 215)  { ADC_map = ccp_pos_min + 96; }
    else if (ADC_in >= 215 && ADC_in < 217)  { ADC_map = ccp_pos_min + 97; }
    else if (ADC_in >= 217 && ADC_in < 219)  { ADC_map = ccp_pos_min + 98; }
    else if (ADC_in >= 219 && ADC_in < 221)  { ADC_map = ccp_pos_min + 99; }
    else if (ADC_in >= 221 && ADC_in < 223)  { ADC_map = ccp_pos_min + 100; }
    else if (ADC_in >= 223 && ADC_in < 226)  { ADC_map = ccp_pos_min + 101; }
    else if (ADC_in >= 226 && ADC_in < 228)  { ADC_map = ccp_pos_min + 102; }
    else if (ADC_in >= 228 && ADC_in < 230)  { ADC_map = ccp_pos_min + 103; }
    else if (ADC_in >= 230 && ADC_in < 232)  { ADC_map = ccp_pos_min + 104; }
    else if (ADC_in >= 232 && ADC_in < 235)  { ADC_map = ccp_pos_min + 105; }
    else if (ADC_in >= 235 && ADC_in < 237)  { ADC_map = ccp_pos_min + 106; }
    else if (ADC_in >= 237 && ADC_in < 239)  { ADC_map = ccp_pos_min + 107; }
    else if (ADC_in >= 239 && ADC_in < 241)  { ADC_map = ccp_pos_min + 108; }
    else if (ADC_in >= 241 && ADC_in < 243)  { ADC_map = ccp_pos_min + 109; }
    else if (ADC_in >= 243 && ADC_in < 246)  { ADC_map = ccp_pos_min + 110; }
    else if (ADC_in >= 246 && ADC_in < 248)  { ADC_map = ccp_pos_min + 111; }
    else if (ADC_in >= 248 && ADC_in < 250)  { ADC_map = ccp_pos_min + 112; }
    else if (ADC_in >= 250 && ADC_in < 252)  { ADC_map = ccp_pos_min + 113; }
    else if (ADC_in >= 252 && ADC_in < 255)  { ADC_map = ccp_pos_min + 114; }
    else { ADC_map = ccp_pos_max; }
    
    return;
}