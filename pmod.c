****
THIS GO ON THE TOP AND REMEMBER TO PRO
****   

#define LED1 LATCbits.LATC14
#define LED2 LATDbits.LATD0
#define LED3 LATDbits.LATD1
#define LED4 LATCbits.LATC13

#PUT THIS PART INSIDE CONFIGURE FUNCTION
TRISCbits.TRISC14 = 1;
TRISDbits.TRISD0 = 1;
TRISDbits.TRISD1 = 1;
TRISCbits.TRISC13 = 1;
ANSELDbits.ANSD1 = 0;

void CheckLightDetection() {
    // Check LED1 and lED1
    if (LED1 == 1) {
        LATCbits.LATC14 = 1; // Example: Turn on indicator LED for LED1
    } else {
        LATCbits.LATC14 = 0; // Turn off indicator for LED1
    }

    // Check LED2 and lED2
    if (LED2 == 1 ) {
        LATDbits.LATD0 = 1; // Example: Turn on indicator LED for LED2
    } else {
        LATDbits.LATD0 = 0; // Turn off indicator for LED2
    }

    // Check LED3 and lED3
    if (LED3 == 1 ) {
        LATDbits.LATD1 = 1; // Example: Turn on indicator LED for LED3
    } else {
        LATDbits.LATD1 = 0; // Turn off indicator for LED3
    }

    // Check LED4 and lED4
    if (LED4 == 1 ) {
        LATCbits.LATC13 = 1; // Example: Turn on indicator LED for LED4
    } else {
        LATCbits.LATC13 = 0; // Turn off indicator for LED4
    }
}
