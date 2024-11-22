#define LED1 LATCbits.LATC14
#define LED2 LATDbits.LATD0
#define LED3 LATDbits.LATD1
#define LED4 LATCbits.LATC13
#define lED1 PORTDbits.RD9
#define lED2 PORTDbits.RD11
#define lED3 PORTDbits.RD10
#define lED4 PORTDbits.RD8



void CheckLightDetection() {
    // Check LED1 and lED1
    if (LED1 == 1 && lED1 == 1) {
        LATBbits.LATB0 = 1; // Example: Turn on indicator LED for LED1
    } else {
        LATBbits.LATB0 = 0; // Turn off indicator for LED1
    }

    // Check LED2 and lED2
    if (LED2 == 1 && lED2 == 1) {
        LATBbits.LATB1 = 1; // Example: Turn on indicator LED for LED2
    } else {
        LATBbits.LATB1 = 0; // Turn off indicator for LED2
    }

    // Check LED3 and lED3
    if (LED3 == 1 && lED3 == 1) {
        LATBbits.LATB2 = 1; // Example: Turn on indicator LED for LED3
    } else {
        LATBbits.LATB2 = 0; // Turn off indicator for LED3
    }

    // Check LED4 and lED4
    if (LED4 == 1 && lED4 == 1) {
        LATBbits.LATB3 = 1; // Example: Turn on indicator LED for LED4
    } else {
        LATBbits.LATB3 = 0; // Turn off indicator for LED4
    }
}
