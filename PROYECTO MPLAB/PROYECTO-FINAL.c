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
#include <stdlib.h>

//-----------------Definición de frecuencia de cristal---------------
#define _XTAL_FREQ 4000000

//-----------------------Constantes----------------------------------

//#define  valor_tmr0 248                         // valor_tmr0 = 156 (0.05 ms)

//-----------------------Variables------------------------------------

// Variables de ADC para TMR0
int servoTMR0_in = 0;
float servoTMR0_map = 0;
int valor_tmr0 = 0;

// Variables de ADC para CCP
int servoCCP_in = 0;
float servoCCP_map = 0;
int servoCCP1_pos = 35, servoCCP2_pos = 35;         // Se inicializan en 35 para evitar problemas post mapeo

// Variables de UART
char serial_in = 0;
int servoNo = 99;
int nums_recibidos = 0;
int aio_servoPos = 0;

// Modo de control de servo
// 0: Manual
// 1: Adafruit
// 2: EEPROM
int modo_control_servo = 1;

//------------Funciones sin retorno de variables----------------------
void setup(void);                               // Función de setup
void tmr0(void);                                // Función para reiniciar TMR0

// Mapeo de valores
void map2TMR0ServoRange(void);                  // Función para mapear de un rango de 0-255 a 7-37
void map2CCPServoRange(void);                   // Función para mapear de un rango de 0-255 a 35-150
char readSerialInput(void);
void updateStatusLEDS(void);

uint8_t readFromEEPROM(uint8_t address);
void writeToEEPROM(uint8_t data, uint8_t address);

//----------------------Interrupciones--------------------------------
void __interrupt() isr(void){
    
    // ====================
    // PORTB INTERRUPT
    // ====================
    
    if (INTCONbits.RBIF == 1){
        
        // Selección de modo con botones
        if (PORTBbits.RB5 == 1 && PORTBbits.RB4 == 0 && PORTBbits.RB3 == 0){
            modo_control_servo = 2;
            CCPR1L = readFromEEPROM(0x04);
            CCPR2L = readFromEEPROM(0x08);
        }
        else if (PORTBbits.RB5 == 0 && PORTBbits.RB4 == 1 && PORTBbits.RB3 == 0){
            modo_control_servo = 1;
        }
        else if (PORTBbits.RB5 == 0 && PORTBbits.RB4 == 0 && PORTBbits.RB3 == 1){
            modo_control_servo = 0;
        }
        
        // Guardar posiciones en modo manual
        if (PORTBbits.RB2 == 1 && modo_control_servo == 0){
            writeToEEPROM(servoCCP1_pos, 0x04);
            writeToEEPROM(servoCCP2_pos, 0x08);
        }
        
        // Se actualizan las LEDS de status
        updateStatusLEDS();
        
        // Se debe limpiar en software la bandera de interrupción
        INTCONbits.RBIF = 0;
    }
    
    // ====================
    // ADC INTERRUPT
    // ====================
    if(PIR1bits.ADIF && modo_control_servo == 0){
        if(ADCON0bits.CHS == 1){                    // Si el channel es 1 (puerto AN1)
            
            servoCCP_in = ADRESH;
            map2CCPServoRange();
            CCPR2L = (int) servoCCP_map;
            
            // Se almacena el mapeo en una variable específica al servo
            servoCCP1_pos = (int) servoCCP_map;
            
            CCP2CONbits.DC2B1 = ADRESH & 0b01;  
            CCP2CONbits.DC2B0 = (ADRESL>>7);
            
        }
        else if (ADCON0bits.CHS == 0){              // Si input channel = 0 (puerto AN0)
            servoCCP_in = ADRESH;
            map2CCPServoRange();
            CCPR1L = (int) servoCCP_map;
            
            // Se almacena el mapeo en una variable específica al servo
            servoCCP2_pos = (int) servoCCP_map;
            
            CCP1CONbits.DC1B1 = ADRESH & 0b01;  
            CCP1CONbits.DC1B0 = (ADRESL>>7);
        } 
        else if (ADCON0bits.CHS == 2){
            servoTMR0_in = ADRESH;                        // Lee el ADC
            map2TMR0ServoRange();                   // Mapea del rango del ADC al del Servo
            
        }
        PIR1bits.ADIF = 0;                      // Limpiar bander de interrupción ADC
    }
    
    // ====================
    // TIMER 0 INTERRUPT
    // ====================
    if(T0IF){
        
        if(PORTCbits.RC3 == 1){
           valor_tmr0 = (int) servoTMR0_map;
           PORTCbits.RC3 = 0;
        }
        else{
            valor_tmr0 = (255 - (int) servoTMR0_map);
            PORTCbits.RC3 = 1;
        }
        
        tmr0();                                 // Reiniciar TMR0
    }
    
    // ====================
    // EUSART INTERRUPT
    // ====================
    if(PIR1bits.RCIF && modo_control_servo == 1){
        
        aio_servoPos = 0;
        servoNo = 99;
        
        serial_in = RCREG;
        PIR1bits.RCIF = 0;
        
        // SI:
        // - Se detecta un comando de servo ("S")
        // - No hay errores de framing
        // - No hay errores de overrun
        //if (serial_in == 'S' && RCSTAbits.FERR == 0 && RCSTAbits.OERR == 0) {
        if (serial_in == 'S') {
            
            // Se lee el número luego de la "S"
            // (Se convierte a número al restarle el valor de '0' en ASCII)
            servoNo = readSerialInput() - '0';
            
            // Leer: Primer dígito de número
            serial_in = readSerialInput();
            
            while (serial_in != 'e'){
                
                // Leer: Todos los dígitos del número
                switch (nums_recibidos){
                    case 0:
                        aio_servoPos += serial_in - '0';
                        break;
                    case 1:
                        aio_servoPos += (serial_in - '0') * 10;
                        break;
                    case 2:
                        aio_servoPos += (serial_in - '0') * 100;
                        break;
                }
                
                // Se cuenta cada dígito recibido
                nums_recibidos ++;
                
                // Si se reciben más de tres dígitos o una "e", se finaliza la lectura
                if (nums_recibidos > 2){
                    serial_in = 'e';
                }
                else {
                    serial_in = readSerialInput();  
                }
            }
            
            // Se envía a transformar en el MAIN_LOOP la posición obtenida de Adafruit
            // Se resetea el número de dígitos recibidos
            servoCCP_in = aio_servoPos;
            nums_recibidos = 0;
        }
        
        // Se limpia la flag de recepción
        PIR1bits.RCIF = 0;
  
    }
}

//----------------------Main Loop--------------------------------
void main(void) {
    
    setup();                                    // Subrutina de setup
    ADCON0bits.GO = 1;                          // Comenzar conversión ADC 
    
    while(1){
        
        // Rutinas a ejecutar según modo
        switch (modo_control_servo){
            
            // ==============================
            // MODO CONTROL: POTENCIÓMETROS
            // ==============================
            case 0:
                // Interrupciones
                PIE1bits.ADIE = 1;                              // Interrupción ADC = enabled
                
                if(ADCON0bits.GO == 0){                         // Si bit GO = 0
                    if(ADCON0bits.CHS == 0){                    // Si Input Channel = AN1
                        ADCON0bits.CHS = 1;                     // Asignar input Channel = AN0
                        __delay_us(50);                         // Delay de 50 us
                    }
                    else if (ADCON0bits.CHS == 1){              // Si Input Channel = AN0
                        ADCON0bits.CHS = 2;                     // Asignar Input Channel = AN1
                        __delay_us(50);                         // Delay de 50 us
                    }
                    else if (ADCON0bits.CHS == 2){
                        ADCON0bits.CHS = 0;
                        __delay_us(50);
                    }
                    
                    __delay_us(50);
                    ADCON0bits.GO = 1;                      // Asignar bit GO = 1
                }
                break;
            
            // ==============================
            // MODO CONTROL: ADAFRUIT
            // ==============================
            case 1:
                
                // Se mapea el valor para los servos
                map2CCPServoRange();
                
                if (servoNo == 1){
                    CCPR1L = (int) servoCCP_map;
                    CCP1CONbits.DC1B1 = aio_servoPos & 0b01;  
                    CCP1CONbits.DC1B0 = 0;  
                }
                else if (servoNo == 2){
                    CCPR2L = (int) servoCCP_map;
                    CCP2CONbits.DC2B1 = aio_servoPos & 0b01;  
                    CCP2CONbits.DC2B0 = 0;
                }
                break;
            
            // ==============================
            // MODO CONTROL: EEPROM
            // ==============================
            case 2:
                NOP();
                
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
    TRISB = 0b00111100;                         // PORTB, bit 7-6 OUT, bit 5-2 IN, bit 1-0 OUT
    TRISD = 0;                                  // PORTD como salida                           
    TRISE = 0;                                  // PORTE como salida
    
    PORTD = 0;                                  // Limpiar PORTD
    PORTB = 0;                                  // Limpiar PORTB
    PORTC = 0;                                  // Limpiar PORTC
    PORTE = 0;                                  // Limpiar PORTE
    
    // Interrupt on-change de PORT B
    IOCB = 0;
    IOCBbits.IOCB5 = 1;
    IOCBbits.IOCB4 = 1;
    IOCBbits.IOCB3 = 1;
    IOCBbits.IOCB2 = 1;
    
    // Configuración de oscilador
    OSCCONbits.IRCF = 0b0110;                   // Oscilador a 4 MHz = 111
    OSCCONbits.SCS = 1;
    
    // Configuración de UART
    TXSTAbits.SYNC = 0;                         // Transmisión asíncrona
    TXSTAbits.BRGH = 0;                         // Baud rate a velocidad baja
    BAUDCTLbits.BRG16 = 1;                      // Baud rate de 16 bits
    
    SPBRG = 25;                                 // Baudrate de 9600 con reloj de 4MHz. Ver tabla de pág 165 del manual
    SPBRGH = 0;
    
    RCSTAbits.SPEN = 1;                         // Puertos seriales habilitados
    RCSTAbits.RX9 = 0;                          // Recepción de datos de 8 bits
    TXSTAbits.TX9 = 0;                          // Envío de datos de 8 bits
    RCSTAbits.CREN = 1;                         // Recepción continua = ON
    TXSTAbits.TXEN = 1;                         // Transmisión continua = ON
    
    
    // Configuración de TMR0
    OPTION_REGbits.T0CS = 0;                    // bit 5  TMR0 Clock Source Select bit...0 = Internal Clock (CLKO) 1 = Transition on T0CKI pin
    OPTION_REGbits.T0SE = 0;                    // bit 4 TMR0 Source Edge Select bit 0 = low/high 1 = high/low
    OPTION_REGbits.PSA = 0;                     // bit 3  Prescaler Assignment bit...0 = Prescaler is assigned to Timer0 module
    OPTION_REGbits.PS2 = 1;                     // bits 2-0  PS2:PS0: Prescaler Rate Select bits
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 1;
    TMR0 = valor_tmr0;                          // preset for timer register
    
    // Configuración del ADC
    ADCON1bits.ADFM = 0;                        // Resultado justificado a la izquierda
    ADCON1bits.VCFG0 = 0;                       // Voltaje 0 de referencia = VSS
    ADCON1bits.VCFG1 = 0;                       // Voltaje 1 de referencia = VDD
    
    ADCON0bits.ADCS = 0b10;                     // Conversión ADC generada con FOSC/32
    ADCON0bits.CHS = 0;                         // Input Channel = AN0
    ADCON0bits.ADON = 1;                        // ADC = enabled
    __delay_us(200);
    
    // Configuración de interrupciones
    INTCONbits.T0IF = 0;                        // Habilitada la bandera de TIMER 0      
    INTCONbits.T0IE = 1;                        // Habilitar las interrupciones de TIMER 0
    INTCONbits.GIE = 1;                         // Habilitar interrupciones globales
    PIR1bits.ADIF = 0;                          // Limpiar bandera de interrupción del ADC
    PIE1bits.ADIE = 1;                          // Interrupción ADC = enabled
    INTCONbits.PEIE = 1;                        // Interrupciones periféricas activadas
    
    PIR1bits.RCIF = 0;                          // Limpiar bandera de interrupción de EUSART
    PIE1bits.RCIE = 1;                          // Interrupción por recepción por EUSART
    PIE1bits.TXIE = 0;                          // Interrupción por envío OFF (No muy utilizado)
    
    // Configuración de PWM (CCP)
    // Periodo para servos (Tpwm) = 2 ms
    TRISCbits.TRISC2 = 1;                       // RC2/CCP1 como entrada
    TRISCbits.TRISC1 = 1;                       // RC1/CCP2 como entrada
    CCP1CONbits.P1M = 0b00;                     // Solo una salida en CCP1
    CCP1CONbits.CCP1M = 0b1100;                 // Modo de PWM para CCP1
    CCP2CONbits.CCP2M = 0b1100;                 // Modo de PWM para CCP2
    
    //CCPR1H = 0;
    CCPR1L = 150;                               // Duty cicle inicial del PWM en CCP1 y CCP2
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
    
    // Actualización de LEDS de Status
    updateStatusLEDS();
    
    // Se colocan dos posiciones iniciales en la EEPROM
    writeToEEPROM(servoCCP1_pos, 0x04);
    writeToEEPROM(servoCCP2_pos, 0x08);
    
    return;
}

//----------------------------Subrutinas----------------------------------------
void tmr0(void){
    INTCONbits.T0IF = 0;                        // Limpiar bandera de TIMER 0
    TMR0 = valor_tmr0;                          // TMR0 = 255
    return;
}

void map2TMR0ServoRange(void){
    
    // Output Range: 7 - 37
    // Input Range: 0 - 255
    // Formula: output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    servoTMR0_map = 7 + (0.1451) * (servoTMR0_in);
    
    return;
}

void map2CCPServoRange(void){
    
    // Output Range: 35 - 150
    // Input Range: 0 - 255
    // Formula: output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start)
    servoCCP_map = 35 + (0.45098) * (servoCCP_in);
    
    return;
}

char readSerialInput(void){
    
    int intentos = 0;
    int max_intentos = 100;
    char serial_input = ' ';
    
    while (PIR1bits.RCIF != 1){
        intentos ++;
        if (intentos > max_intentos){ 
            break;
        }
    }
    
    serial_input = RCREG;
    PIR1bits.RCIF = 0;
    return(serial_input);
    
}

void updateStatusLEDS(void){
    
    if (modo_control_servo == 2){
        PORTCbits.RC5 = 1;
        PORTCbits.RC4 = 0;
        PORTDbits.RD3 = 0;
    }
    else if (modo_control_servo == 1){
        PORTCbits.RC5 = 0;
        PORTCbits.RC4 = 1;
        PORTDbits.RD3 = 0;
    }
    else if (modo_control_servo == 0){
        PORTCbits.RC5 = 0;
        PORTCbits.RC4 = 0;
        PORTDbits.RD3 = 1;
    }
    
    return;
}

uint8_t readFromEEPROM(uint8_t address){
    EEADR = address;                        // Dirección a leer
    EECON1bits.EEPGD = 0;                   // Memoria de datos
    EECON1bits.RD = 1;                      // Hace una lectura
    uint8_t data = EEDAT;                   // Guarda el dato leído de la EEPROM
    return data;
}

// writeToEEPROM(variable, dir1EEPROM)
void writeToEEPROM(uint8_t data, uint8_t address){
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD = 0;                   // Escribir a memoria de datos
    EECON1bits.WREN = 1;                    // Habilitar escritura a EEPROM (datos)
    
    INTCONbits.GIE = 0;                     // Deshabilitar interrupciones
    EECON2 = 0x55;                          // Secuencia obligatoria
    EECON2 = 0xAA;                          
    EECON1bits.WR = 1;                      // Escribir / Write
    
    while (PIR2bits.EEIF == 0);
    PIR2bits.EEIF = 0;
    
    INTCONbits.GIE = 1;                     // Habilitar interrupciones
    EECON1bits.WREN = 0;                    // Deshabilitar escritura de EEPROM
    
    return;
}