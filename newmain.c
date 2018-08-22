
// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = XT_XT     // Oscillator Selection bits (XT oscillator (XT))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = ON      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define led PORTCbits.RC0 // led saída

void inicializa_com(void) {
    //INICIALIZA PERIFÉRICO DE COMUNICAÇÃO
    TXSTA = 0b00100100; //MODO 8BITS, ASSÍNCRONO, HIGH SPEED
    RCSTA = 0b10010000; //habilita serial, 8 bits, habilitar recepcao
    BAUDCON = 0b01000000; //operando com divisor baud rate generator em modo 8bits
    SPBRG = 25; // veloc = 4.10e6 / (16 * (25 + 1 )) = 9615bps

}

void delay_ms(int tempo) {
    while (tempo > 0) {
        T0CS = 0;
        PSA = 0;
        T0PS0 = 1;
        T0PS1 = 0;
        T0PS2 = 0;
        TMR0IF = 0;
        T08BIT = 1;
        TMR0L = 0;
        TMR0ON = 1;
        while (TMR0IF == 0);
        --tempo;
    }
}

void tx_byte(char dado) {
    //rotina de transmissão
    TXREG = dado;
    while (TXSTAbits.TRMT == 0);
}

char piscar_led = 0; // se for 1, a rotina rx_byte devera piscar o led

char rx_byte(void) {
    //rotina de recepção
    char dado;
    //rotina de rx
    while (PIR1bits.RCIF == 00) {
        if (piscar_led == 1) {
            led = 1;
            delay_ms(500);
            led = 0;
            delay_ms(500);
        }
    }
    dado = RCREG;
    return (dado);
}

void main() {
    char r;
    char endereco = 100;
    int n;
    n = 0;
    inicializa_com();
    TRISCbits.TRISC0 = 0; //saída
    TRISCbits.TRISC6 = 0; //saída
    TRISCbits.TRISC7 = 1; //saída
    TRISA = 0b00001111; // pinos ra0, ra1, ra2 e ra3 sao entradas
    ADCON1 = 0b00001111; // pinos da PORTA como entradas digitais
    led = 0;
    while (1) {
        r = rx_byte();
        switch (r) {
            case 'a': endereco = 0;
                break;
            case 'b': endereco = 1;
                break;
            case 'c': endereco = 2;
                break;
            case '0': if (endereco == PORTA) {
                    led = 0;
                    piscar_led = 0;
                }
                break;
            case '1': if (endereco == PORTA) {
                    led = 1;
                    piscar_led = 0;
                }

                break;
            case '2': if (endereco == PORTA) {
                    piscar_led = 1;
                }

                break;
        }
    }
}

