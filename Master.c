/* 
 * File:   Proyecto2.c
 * Author: Marian López
 *
 * Created on 28 de mayo de 2022, 05:01 PM
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
#define _XTAL_FREQ 250000

// VARIABLES
unsigned short servo1 = 0;
unsigned short servo2 = 0;
unsigned short servo3 = 0;
unsigned short servo4 = 0;
unsigned short lectura_s1 = 0;
unsigned short lectura_s2 = 0;
unsigned short lectura_s3 = 0;
unsigned short lectura_s4 = 0;
char mensaje;

// POTOTIPOS DE FUNCIONES
void setup(void);
void chanel_ADC(void);
unsigned short map(int val, int i_min, int i_max, short o_min, short o_max);
void spi_write(unsigned char dato);
int write_EEPROM(int address, int data);
int guardar_posiciones(int s1, int s2, int s3, int s4);
void leer_posiciones();
void modificar_CCP1 (int posicion);
void modificar_CCP2 (int posicion);

// INTERRUPCIONES
void __interrupt() isr(void){
    
    if (PIR1bits.ADIF && PORTE == 1){                 // Fue int. ADC y es modo 1?
        if (ADCON0bits.CHS == 0){                     // Fue AN0?
            servo1 = map(ADRESH, 0, 255, 15, 32);     // Interpolación de datos
            modificar_CCP1(servo1);                   // Actualizar servo 1
        }
        if (ADCON0bits.CHS == 1){                     // Fue AN1?
            servo2 = map(ADRESH, 0, 255, 15, 32);     // Interpolación de datos
            modificar_CCP2(servo2);                   // Actualizar servo 2
        }
        if (ADCON0bits.CHS == 2){       // Fue AN2?
            servo3 = ADRESH;            // Guardar valor
        }
        if (ADCON0bits.CHS == 3){       // Fue AN3?
            servo4 = ADRESH;            // Guardar valor
        }
        PIR1bits.ADIF = 0;              // Apagar bandera int. ADC
    }
    
    if (INTCONbits.RBIF){           // Fue int. PORTB?
        while(!PORTBbits.RB0);      // Antirrebote
        switch (PORTE){             // Cambio de modo
            case (1): {
                PORTE = 2;
                break;
            }
            case (2): {
                PORTE = 4; 
                break;
            }
            case (4): {
                PORTE = 1;
                break;
            }
        }
        INTCONbits.RBIF = 0;        // Apagar bandera int. PORTB
    }
    
    if (PIR1bits.RCIF){         // Fue UART?
        mensaje = RCREG;        // Recibir mensaje
    }
    
    return;
}


// CICLO PRINCIPAL
void main(void){
    setup();
    while(1){
            
        // Modo control manual y grabado de posiciones
        if(PORTEbits.RE0){
            chanel_ADC();                       // Cambio de canal ADC
            spi_write(servo3);                  // Enviar valores para servo 3
            spi_write(servo4);                  // Enviar valores para servo 4
            guardar_posiciones(servo1, servo2, servo3, servo4); // Guardar en EEPROM según botón presionado
        }
        
        // Modo reproducción de posiciones
        if(PORTEbits.RE1){
            leer_posiciones();      // Leer EEPROM y enviar a Servos según botón presionado
        }
        
        // Modo control de PC    
    
    }
        

    
    
    return;
}

// CONFIGURACIONES
void setup(void){
    ANSEL = 0b1111;             // AN0, AN1, AN2 y AN3 como analógicos
    ANSELH = 0;
    TRISA = 0b1111;             // AN0, AN1, AN2 y AN3 como entradas
    PORTA = 0;
    
    OPTION_REGbits.nRBPU = 0;   // Pull ups encendidas individualmente
    WPUB = 0b11111;             // Pull ups encendidas
    IOCB = 0b1;                 // Int. por cambio de estado encendida
    
    TRISE = 0;
    PORTE = 0b0001;
    
    TRISD = 0;
    PORTD = 0;
    // Configuración reloj interno
    OSCCONbits.IRCF = 0b010;    // 250 kHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    // Configuración ADC
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    ADCON0bits.CHS = 0;         // Seleccionar AN0
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON0bits.ADON = 1;        // Habilitar modulo ADC
    __delay_us(40);   
    
    // Configación PWM (según manual para no dañar servos)
    TRISCbits.TRISC2 = 1;           // CCP1 como entrada
    TRISCbits.TRISC1 = 1;           // CCP2 como entrada
    CCP1CON = 0;                    // CCP apagado
    CCP1CONbits.P1M = 0;            // Modo salida simple
    CCP1CONbits.CCP1M = 0b1100;     // PWM
    CCP2CONbits.CCP2M = 0b1100;     // PWM
    CCPR1L = 31>>2;                 // Valor inicial 2 ms ancho de pulso
    CCP1CONbits.DC1B = 31 & 0b11;
    CCPR2L = 31>>2;                 // Valor inicial 2 ms ancho de pulso
    CCP2CONbits.DC2B1 = 31 & 0b10;
    CCP2CONbits.DC2B0 = 31 & 0b01;
    
    // Configuración TMR2
    PIR1bits.TMR2IF = 0;            // Bandera int. TMR2 apagada
    T2CONbits.T2CKPS = 0b11;        // Prescaler 1:16
    T2CONbits.TMR2ON = 1;           // TMR2 ON
    PR2 = 77;                       // Período de 20 ms
    while(!PIR1bits.TMR2IF);        // Esperar un cliclo del TMR2
    PIR1bits.TMR2IF = 0;            // Bandera int. TMR2 encendida
    TRISCbits.TRISC2 = 0;           // CCP1 como salida
    TRISCbits.TRISC1 = 0;           // CCP2 como salida
    
    // Configuracón serial SPI
    TRISC = 0b00010000;         // SDI entrada, SCK y SD0 salida
    PORTC = 0;
    SSPCONbits.SSPM = 0b0000;   // SPI master, reloj = Fosc/4
    SSPCONbits.CKP = 0;         // Polaridad = Reloj inactivo en 0
    SSPCONbits.SSPEN = 1;       // Habilitar pines de SPI
    SSPSTATbits.CKE = 1;        // Dato enviado a cada flanco positivo del reloj
    SSPSTATbits.SMP = 1;        // Se lee al final del periodo del reloj
    SSPBUF = 0;
      
    //Configuración serial UART
    TXSTAbits.SYNC = 0;         // Modo asíncrono
    TXSTAbits.BRGH = 1;         // Usar BAUD con alta velocidad
    BAUDCTLbits.BRG16 = 1;      // Usar el registro en 16 bits
    SPBRG = 25;                 // BAUD rate 9600 aprox
    SPBRGH = 0;
    RCSTAbits.SPEN = 1;         // Habilitar comunicación
    TXSTAbits.TX9 = 0;          // Utilizar 8 bits
    TXSTAbits.TXEN = 1;         // Habilitar transmisión
    RCSTAbits.CREN = 1;         // Habilitar recepción
    
    // Configuración interrupciones
    PIE1bits.ADIE = 1;          // Int. ADC
    PIR1bits.ADIF = 0;          // Bandera int. ADC apagada
    INTCONbits.RBIE = 1;        // Int. PORTB
    INTCONbits.RBIF = 0;        // Bandera int. PORTB apagada
    PIE1bits.RCIE = 1;          // Int. recepción UART
    PIR1bits.RCIF = 0;          // Bandera int. recepciones UART
    INTCONbits.GIE = 1;         // Int. globales
    INTCONbits.PEIE = 1;        // Int. periféricos
    return;
}

// Interpolación de datos
unsigned short map(int x, int x0, int x1, short y0, short y1){
    return (unsigned short)(y0 + ((float)(y1-y0)/(x1-x0)) * (x-x0));    // Ecuación para la interpolación de datos
}

// Enivar el dato al buffer
void spi_write(unsigned char dato){
    SSPBUF = dato;                  // Cargar dato
    while(SSPSTATbits.BF == 0);     // Esperar a que se envíe
    while(PIR1bits.SSPIF == 0);     // Se ha producido una transmisión
    PIR1bits.SSPIF = 0;             // Limpiar bit
}

// Escribir en la EEPROM
int write_EEPROM(int address, int data){
    EEADR = address;
    EEDAT = data;
    EECON1bits.EEPGD = 0;   // Escribir la EEPROM
    EECON1bits.WREN = 1;    // Habilitar escritura en la EEPROM
    
    INTCONbits.GIE = 0;     // Apagar int. globales para no interrumpir escritura
    EECON2 = 0x55;          // Preparar memoria para recibir datos
    EECON2 = 0xAA;  
    
    EECON1bits.WR = 1;      // Iniciar escritura
    EECON1bits.WREN = 0;    // Deshabilitar para que no pase nada del uC a la EEPROM
    while(!PIR2bits.EEIF);  // Esperar a que se termine la escritura
    PIR2bits.EEIF = 0;      // Limpiar bit
    INTCONbits.RBIF = 0;    // Limpiar bandera int. PORTB
    INTCONbits.GIE = 1;     // Habilitar interrupciones globales
}

// Leer de la EEPROM
int read_EEPROM(int address){
    EEADR = address;        // Escribir la dirección
    EECON1bits.EEPGD = 0;   // Acceder a la EEPROM
    EECON1bits.RD = 1;      // Iniciar la lectura
    return EEDAT;           // Regresar dato leido
}

void chanel_ADC(void){
    if(ADCON0bits.GO == 0){             // No hay proceso de conversion?
        if(ADCON0bits.CHS == 0)         // Cambia a canal 1
            ADCON0bits.CHS = 1;
        else if(ADCON0bits.CHS == 1)    // Cambia a canal 2
            ADCON0bits.CHS = 2;
        else if(ADCON0bits.CHS == 2)    // Cambia a canal 3
            ADCON0bits.CHS = 3;
        else if(ADCON0bits.CHS == 3)    // Cambia a canal 0
            ADCON0bits.CHS = 0;
        __delay_us(50);                 // Estabilización del circuito           
        ADCON0bits.GO = 1;              // Iniciar la conversión    
    }
    return;
}

// Guardar posiciones de servos en EEPROM según botón presionado
int guardar_posiciones(int s1, int s2, int s3, int s4){
    if (!PORTBbits.RB1){            // RB1 presionado
        while(!PORTBbits.RB1);      // Antirrebote
        write_EEPROM(1, s1);        // Guardar valor servo 1
    }
    if (!PORTBbits.RB2){            // RB2 presionado
        while(!PORTBbits.RB2);      // Antirrebote
        write_EEPROM(2, s2);        //Guardar valor servo 2
    }
    if (!PORTBbits.RB3){            // RB3 presionado
        while(!PORTBbits.RB3);      // Antirrebote
        write_EEPROM(3, s3);        // Guardar valor servo 3
    }
    if (!PORTBbits.RB4){            // RB4 presionado
        while(!PORTBbits.RB4);      // Antirrebote
        write_EEPROM(4, s4);        // Guardar valor servo 4
    }
}

// Leer posiciones de EEPROM y envias a servos según botón presionado
void leer_posiciones(void){
    if (!PORTBbits.RB1){                // RB1 presionado
        while(!PORTBbits.RB1);          // Antirrebote
        lectura_s1 = read_EEPROM(1);    // Guardar valor servo 1
        modificar_CCP1(lectura_s1);
    }
    if (!PORTBbits.RB2){                // RB2 presionado
        while(!PORTBbits.RB2);          // Antirrebote
        lectura_s2 = read_EEPROM(2);    // Guardar valor servo 2
        modificar_CCP2(lectura_s2);
    }
    if (!PORTBbits.RB3){                // RB3 presionado
        while(!PORTBbits.RB3);          // Antirrebote
        lectura_s3 = read_EEPROM(3);    // Guardar valor servo 3
        spi_write(lectura_s3);          // Enviar valores para servo 3
    }
    if (!PORTBbits.RB4){                // RB4 presionado
        while(!PORTBbits.RB4);          // Antirrebote
        lectura_s4 = read_EEPROM(4);    // Guardar valor servo 4
        spi_write(lectura_s4);          // Enviar valores para servo 4
    }
}

// Actualizar CCP1 del maestro
void modificar_CCP1 (int posicion){
    CCPR1L = (uint8_t)(posicion>>2);            // 6 bits a reg CCPR1L
    CCP1CONbits.DC1B = posicion & 0b11;         // 2 bits a DC1B
}

// Actualizar CCP2 del maestro
void modificar_CCP2 (int posicion){
    CCPR2L = (uint8_t)(posicion>>2);            // 6 bits a reg CCPR2L
    CCP2CONbits.DC2B1 = (posicion & 0b10)>>1;   // 2 bits a DC2B
    CCP2CONbits.DC2B0 = posicion & 0b01;
}