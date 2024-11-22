#define LED1 LATCbits.LATC14
#define LED2 LATDbits.LATD0
#define LED3 LATDbits.LATD1
#define LED4 LATCbits.LATC13
#define IR1 PORTCbits.RC2
#define IR2 PORTCbits.RC1
#define IR3 PORTCbits.RC4
#define IR4 PORTGbits.RG6

   TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC3 = 1;
    TRISGbits.TRISG6 = 1;
    ANSELGbits.ANSG6 = 0;

void CheckLightDetection() {
    // Check LED1 and lED1
    if (IR1 == 1) {
        LATA = 0x1 // Example: Turn on indicator LED for LED1
    } else {
        LATA = 0; // Turn off indicator for LED1
    }

    // // Check LED2 and lED2
    // if (IR2 == 1) {
    //     LATA = 0x1 // Example: Turn on indicator LED for LED2
    // } else {
    //     LATBbits.LATB1 = 0; // Turn off indicator for LED2
    // }

    // // Check LED3 and lED3
    // if (IR3 == 1) {
    //     LATBbits.LATB2 = 1; // Example: Turn on indicator LED for LED3
    // } else {
    //     LATBbits.LATB2 = 0; // Turn off indicator for LED3
    // }

    // // Check LED4 and lED4
    // if (IR1 == 1) {
    //     LATBbits.LATB3 = 1; // Example: Turn on indicator LED for LED4
    // } else {
    //     LATBbits.LATB3 = 0; // Turn off indicator for LED4
    // }
}
