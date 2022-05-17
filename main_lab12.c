/* 
 * File:   main.c
 * Author: Christopher Chiroy
 *
 * Potenciometro en AN0, ADRESH -> PORTC, ADRESL -> PORTD
 * 
 * Created on 5 april 2022, 20:12
 */

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

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 4000000
#define address_POT 0

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
uint8_t bandera_write=0, bandera_sleep=0, val_POT=0;
/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.ADIF){              // Fue interrupción del ADC?
        if(ADCON0bits.CHS == 0){    // Verificamos sea AN0 el canal seleccionado
            val_POT = ADRESH;       // El ADRESH es nuestro val POT 
            PORTC = val_POT;        // Lo mostramos en el puerto C
        }
        PIR1bits.ADIF = 0;          // Limpiamos bandera de interrupción
    }
    else if (INTCONbits.RBIF){      // Interrupcion del IOCB
        if (PORTBbits.RB0 == 0){    // Fue el boton 0 - dormir
            if (bandera_sleep == 0){ // Si esta despierto
                bandera_sleep = 1;  // Dormir
            }
        }
        if (PORTBbits.RB1 == 0){    // Boton 1 - despertar
            if (bandera_sleep == 1){ // Esta dormido
                bandera_sleep = 0;  // Despertar
            }
        }
        if (PORTBbits.RB2 == 0){    // Boton 2
            bandera_sleep = 0;      // Despertamos
            bandera_write = 1;      // Guardamos
        }
        INTCONbits.RBIF = 0;        // Limpiamos bandera
    }
    
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){
        if(ADCON0bits.GO == 0){             // No hay proceso de conversion
            ADCON0bits.GO = 1;              // Iniciamos proceso de conversión
        }
        if(bandera_sleep == 1){         // Si mandamos a dormir el PIC        
            PIE1bits.ADIE = 0;          // Desactivar interrupcion ADC
            SLEEP();                    // Pasar al modo sleep
        }
        else if(bandera_sleep == 0){    // Si despertamos al PIC       
            PIE1bits.ADIE = 1;          // Habilitamos la interrupcion (ADC puede desperarlo)
        }
        if(bandera_write == 1){                 // Guardar el valor
            write_EEPROM(address_POT,val_POT);  // Guardar el valor del POT
            __delay_ms(10);                     // delay
            bandera_write = 0;                  // Dejar de escribir
        }
        PORTD = read_EEPROM(address_POT);       // Lee constantemente
        PORTE = 0b0001;                         // Enciende y apaga un led
        __delay_ms(250);
        PORTE = 0b0000;
        __delay_ms(250);
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0b00000001; // AN0 como entrada analógica
    ANSELH = 0;         // I/O digitales)
    
    
    TRISA = 0b00000001; // AN0 como entrada
    PORTA = 0; 
    
    TRISC = 0;
    PORTC = 0;
    TRISD = 0;
    PORTD = 0;
    TRISE = 0;
    PORTE = 0;
    
    TRISB = 0xFF;
    OPTION_REGbits.nRBPU = 0;
    WPUB = 0b00000111;
    IOCB = 0b00000111;
    
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b0110;   // 4MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(40);             // Sample time
    
    // Configuracion interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    
}

uint8_t read_EEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;       // Lectura a la EEPROM
    EECON1bits.RD = 1;          // Obtenemos dato de la EEPROM
    return EEDAT;               // Regresamos dato 
}

void write_EEPROM(uint8_t address, uint8_t data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;       // Escritura a la EEPROM
    EECON1bits.WREN = 1;        // Habilitamos escritura en la EEPROM
    
    INTCONbits.GIE = 0;         // Deshabilitamos interrupciones
    EECON2 = 0x55;      
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;          // Iniciamos escritura
    
    EECON1bits.WREN = 0;        // Deshabilitamos escritura en la EEPROM
    //INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;         // Habilitamos interrupciones
}