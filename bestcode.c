    /*===================================CPEG222====================================
     * Program:      Project 4 Template
     * Authors:     Robert Freeman
     * Date:        11/07/2024
     * Description: This project leverages the Basys MX3 microcontroller to power a robot designed to race against multiple competitors.
     *              It incorporates new configurations and functions to optimize control, time tracking, and motor activation, contributing 
     *              to a competitive robot capable of racing against others in the challenge.

     * 
    ==============================================================================*/
    /*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART ----------*/
    #ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning during compiling
    #define _SUPPRESS_PLIB_WARNING
    #endif
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
    #pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
    #pragma config FPLLODIV = DIV_1         // System PLL Output Clock Divider (PLL Divide by 1)
    #pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
    #pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
    #pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
    #pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
    /*----------------------------------------------------------------------------*/
    #define SYS_FREQ (80000000L) // 80MHz system clock
    #define _80Mhz_ (80000000L)
    #define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
    #define LOOPS_NEEDED_TO_DELAY_ONE_MS (LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))

    #define TRUE 1
    #define FALSE 0


    // Libraries
    #include <string.h>
    #include <stdlib.h>
    #include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
    #include <stdio.h>  // need this for sprintf
    #include <sys/attribs.h>
    #include "config.h" // Basys MX3 configuration header
    #include "led.h"
    #include "ssd.h"
    #include "lcd.h"
    #include "swt.h"
    #include "srv.h"
    #include "acl.h"            //Accelerometer
    #include "adc.h"            //Analog-to-Digital Converter
    #include "aic.h"     
    #include "rgbled.h"         //RGB LED


    #define LED1 PORTCbits.RC14
    #define LED2 PORTDbits.RD0
    #define LED3 PORTDbits.RD1
    #define LED4 PORTCbits.RC13




    #include <stdlib.h>




    #include "utils.h"

    int micVal, micNum = 0;           // value read from the microphone input; greatest value of a given number of micVal values
    int clapcount, claptimer = 0;     // clapcount keeps track of how many claps the system has heard. After 11 ticks of claptimer
                                      // @100ms/tick courtesy of the core timer, resets the clapcount to 0.
    // Function Declarations
    void intializePorts();
    void pwmConfig();
    void activateServo();
    void timer2_init();
    void timer3_init();
    void CNConfig();
    void MIC_Stuff(); 

    //#define LED1 ADC_AnalogRead(17)
    //#define LED2  PORTCbits.RC3
    //#define LED3  ADC_AnalogRead(18)
    //#define LED4  ADC_AnalogRead(19)
    int main(void) {

        intializePorts();
        pwmConfig();
        LCD_WriteStringAtPos("Group 16 JERRY",0,0);
            while (TRUE){ // ready mode, looknig for claps.
                LED_SetGroupValue(LED1 | LED2 << 1 | LED3 << 2 | LED4 << 3);
                
            micVal = MIC_Val();
            MIC_Stuff();
            if (micNum > 0x3F) {

                clapcount += 1;
                micNum = 0;
                delay_ms(50);
            }

            if (clapcount  == 2  ){
                SSD();
                IncrementTime();
                activateServo();
            }     
    //            SRV_SetPulseMicroseconds0(300);
    //        SRV_SetPulseMicroseconds1(1500);
        }
    }






    // Initialize ports on board
    void intializePorts() {
        LED_Init();
        SRV_Init();
        LCD_Init();
        SSD_Init();
        SWT_Init();
        ACL_Init();
        ADC_Init();
        AIC_Init();
        MIC_Init();
        timer2_init();
        timer3_init();
        CNConfig();
        DDPCONbits.JTAGEN = 0; 
        TRISA &= 0xFF00;
        LATA &= 0xFF00; 
        // Initialize SW6
    TRISBbits.TRISB10 = 1; 
    ANSELBbits.ANSB10 = 0; 
    // Initialize SW7
    TRISBbits.TRISB9 = 1; 
    ANSELBbits.ANSB9 = 0; 
    TRISCbits.TRISC14 = 1;
    TRISDbits.TRISD0 = 1;
    TRISDbits.TRISD1 = 1;
    TRISCbits.TRISC13 = 1;
    ANSELDbits.ANSD1 = 0;
    }
    void LED(void) {
       //RIGHT WHEEL
        if (SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0) {
            LCD_WriteStringAtPos("STP",1,13);
            LATA &= ~0x0F; // Clear bits 0 to 3 (turn off LEDs 0, 1, 2, 3)
        }
        //This is supposed to turn on leds 2 and 3
         if (SWT_GetValue(0) == 1 && SWT_GetValue(1) == 0) {
             LCD_WriteStringAtPos("FWD",1,13);
            LATA |= (1 << 2) | (1 << 3); // Set bits 2 and 3 (turn on LEDs 2 and 3)
        }
        //this is supposed to turn on leds 0 and 1
         if (SWT_GetValue(1) == 1 && SWT_GetValue(0) == 0) {
             LCD_WriteStringAtPos("REV",1,13);
             LATA |= (1 << 0);  // Set bit 0 (RA0) to 1 to turn on LED 0
        LATA |= (1 << 1); 
      // turn off leds 3-0
        }
        if (SWT_GetValue(1) && SWT_GetValue(0)) {
             LCD_WriteStringAtPos("STP",1,13);
             LATA &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
        }


        //LEFT WHEEL
       if (SWT_GetValue(6) && SWT_GetValue(7)){
            LCD_WriteStringAtPos("STP",1,0);
            LATA &= ~(0xF0);    
        }
        else if (SWT_GetValue(7) && SWT_GetValue(6) == 0){

            LCD_WriteStringAtPos("REV",1,0);
            LATA |= (1 << 6);  // Set bit 6 (RA6) to 1 to turn on LED 6
        LATA |= (1 << 7);
        }
        else if (SWT_GetValue(6) && SWT_GetValue(7) == 0){
            LCD_WriteStringAtPos("FWD",1,0);
            LATA |= (1 << 4);    // Set bit 4 (RA4) to turn on LED 4
        LATA |= (1 << 5);

        }
        if (SWT_GetValue(6)==0 && SWT_GetValue(7)==0){
            LCD_WriteStringAtPos("STP",1,0);
            LATA &= ~(0xF0);    
        }
    }
    int clock_times[4] = {0,0,0,0};//array to store current time

    void IncrementTime() {

    //    if (clapcount < 2){
    //        clock_times[0] = clock_times[0];
    //        clock_times[1] = clock_times[1];
    //        clock_times[2] = clock_times[2];
    //        clock_times[3] = clock_times[3];
    //    }
    ////    }
    //    else if (clapcount == 2){
        clock_times[0]++; // Increment the rightmost digit (ones place)

        if (clock_times[0] > 9) {
            clock_times[0] = 0;
            clock_times[1]++; // Increment tens place
        }
        if (clock_times[1] > 9) {
            clock_times[1] = 0;
            clock_times[2]++; // Increment hundreds place
        }
        if (clock_times[2] > 9) {
            clock_times[2] = 0;
            clock_times[3]++; // Increment thousands place
        }
        if (clock_times[3] > 9) {
            clock_times[3] = 0; // Reset after 9999 (or any value you want)

        }
        }

    void SSD() {
        int time = ((clock_times[3] << 12) | (clock_times[2] << 8)
                   | (clock_times[1] << 4) | (clock_times[0]));
        SSD_WriteDigitsGrouped(time, 0x00);  // Assuming this function writes correctl
        delay_ms(40) ;// Adjust as needed


    }

    void delay_ms(int milliseconds){
        int i;
        for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++){

        }
    }
    volatile uint32_t counter = 0;  
    void ISR() {
        counter++;  // Increment the counter on every interrupt

        if (counter == 10) {  // This will occur after 1000 ms (100 ms * 10)
            counter = 0;  // Reset counter after 1000 ms (1 second)
        }
    }

    void pwmConfig() {

        // configure Timer X (select the timer, replace X with desired timer number)

        PR2 = 24999; //Period Register.
        T2CONbits.TCKPS = 3; //Timer Prescaler 
        T2CONbits.TGATE = 0; // not gated input (the default)
        T2CONbits.TCS = 0; // PBCLK input (the default)
        T2CONbits.ON = 1;  //Turn on Timer
        TMR2 = 0; // Set Timer X to 0

        IPC2bits.T2IP = 7;  //    priority
        IPC2bits.T2IS = 3;  //    subpriority
        IFS0bits.T2IF = 0; //    clear interrupt flag
        IEC0bits.T2IE = 1; //    enable interrupt


        // Configure Output Compare Module 4

        OC4CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
        OC4CONbits.OCTSEL = 0;   // Select the timer to use as a clock source
        OC4RS =  18750;//OC4RS is some fraction of the Period
        OC4R = OC4RS;
        OC4CONbits.ON = 1;       // Start the OC4 module

        //Do The same for OC5**************************
        OC5CONbits.OCM = 6;      // PWM mode on OC5; Fault pin is disabled
        OC5CONbits.OCTSEL = 0;   // Select the timer to use as a clock source
        OC5RS =  18750;//OC5RS is some fraction of the Period
        OC5R = OC5RS;
        OC5CONbits.ON = 1;       // Start the OC5 module


       TRISBbits.TRISB8 = 0; //set servo 0 as output
       TRISAbits.TRISA15 = 0; //set servo 1 as output
       ANSELBbits.ANSB8 = 0; //set servo 0 as digital

       RPB8R = 0x0B; // connect Servo 0 to OC5
       RPA15R = 0x0B;// connect Servo 1 to OC4


        //Set up additional timers here if necessary
    }
    void timer2_init() {
    /* Make sure vector interrupts is disabled prior to configuration */
        macro_disable_interrupts;
        T2CONbits.ON = 0; // Turn off Timer 2 during configuration
        T2CONbits.TCKPS = 0b111; // Prescaler 1:256
        T2CONbits.TCS = 0; // Use internal PBCLK
        TMR2 = 0; // Clear Timer 2 register
        // this pre register is getting the period register.
        // 10,000,000/256 = 39063 hz which then / period register makes 1hz which is
        // one second
        PR2 = (10000000 / (256*4)); // set at quater of a second
        // PRx=PBCLK/(timer prescaler*timer frequency)
        IPC2bits.T2IP = 4; // Interrupt priority
        IPC2bits.T2IS = 0; // Interrupt subpriority
        IFS0bits.T2IF = 0; // Clear interrupt flag
        IEC0bits.T2IE = 1; // Enable Timer 2 interrupt
        T2CONbits.ON = 1; // Turn on Timer 2
        macro_enable_interrupts(); // enable interrupts at CPU

    }

    void timer3_init() {
    /* Make sure vector interrupts is disabled prior to configuration */
        macro_disable_interrupts;
        T3CONbits.ON = 0; // turn off Timer3
        OC1CONbits.ON = 0; // Turn off OC1
        /* The following code sets up the alarm timer and interrupts */
        tris_A_OUT = 0;
        rp_A_OUT = 0x0C; // 1100 = OC1
        // disable analog (set pins as digital)
        ansel_A_OUT = 0;
        T3CONbits.TCKPS = 0b1; //1:1 prescale value
        T3CONbits.TGATE = 0; //not gated input (the default)
        T3CONbits.TCS = 0; //PCBLK input (the default)
        OC1CONbits.ON = 0; // Turn off OC1 while doing setup.
        OC1CONbits.OCM = 6; // PWM mode on OC1; Fault pin is disabled
        OC1CONbits.OCTSEL = 1; // Timer3 is the clock source for this Output Compare
        //module
        IPC3bits.T3IP = 7; // interrupt priority
        IPC3bits.T3IS = 3; // interrupt subpriority
        macro_enable_interrupts(); // enable interrupts at CPU
    }


    //ISR's are here is you need them. Don't forget to set them up!
    void __ISR(_TIMER_2_VECTOR) Timer2ISR(void) {
        IEC0bits.T2IE = 0; // disable interrupt

        IFS0bits.T2IF = 0; // clear interrupt flag
        IEC0bits.T2IE = 1; // enable interrupt
    }

    void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
        IEC0bits.T3IE = 0; // disable interrupt

        IFS0bits.T3IF = 0; // clear interrupt flag
        IEC0bits.T3IE = 1; // enable interrupt
    }

    void __ISR(_CHANGE_NOTICE_VECTOR) CN_Handler(void) {
    // 1. Disable CN interrupts
    IEC1bits.CNDIE = 0;
    // 2. Debounce keys for 10ms

    for (int a=0; a<1426; a++) {}
    if (SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0) {
            LCD_WriteStringAtPos("STP",1,13);
            LATA &= ~0x0F; // Clear bits 0 to 3 (turn off LEDs 0, 1, 2, 3)
        }
        //This is supposed to turn on leds 2 and 3
         if (SWT_GetValue(0) == 1 && SWT_GetValue(1) == 0) {
             LCD_WriteStringAtPos("FWD",1,13);
            LATA |= (1 << 2) | (1 << 3); // Set bits 2 and 3 (turn on LEDs 2 and 3)
        }
        //this is supposed to turn on leds 0 and 1
         if (SWT_GetValue(1) == 1 && SWT_GetValue(0) == 0) {
             LCD_WriteStringAtPos("REV",1,13);
            LATA |= 0x02; ;
      // turn off leds 3-0
        }
        if (SWT_GetValue(1) && SWT_GetValue(0)) {
             LCD_WriteStringAtPos("STP",1,13);
             LATA &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3));
        }


       if (SWT_GetValue(6) && SWT_GetValue(7)){
            LCD_WriteStringAtPos("STP",1,0);
            LATA |= (1 << 2) | (1 << 3);
        }
        else if (SWT_GetValue(7) && SWT_GetValue(6) == 0){
            LCD_WriteStringAtPos("REV",1,0);
        }
        else if (SWT_GetValue(6) && SWT_GetValue(7) == 0){
            LCD_WriteStringAtPos("FWD",1,0);
        }
        else{
            LCD_WriteStringAtPos("STP",1,0);

    IFS1bits.CNDIF = 0;
    // 6. Reenable CN interrupts
    IEC1bits.CNDIE = 1;

    }
    }
    void CNConfig() {
    /* Make sure vector interrupts is disabled prior to configuration */
        macro_disable_interrupts;
        // Complete the following configuration of CN interrupts, then uncomment them
        CNCONDbits.ON = 1; //all port D pins to trigger CN interrupts
        CNEND = 0x0F00; //configure PORTD pins 8-11 as CN pins
        CNPUD = 0x0F00; //enable pullups on PORTD pins 8-11
        IPC8bits.CNIP = 5; // set CN priority t`o 5
        IPC8bits.CNIS = 3; // set CN sub-priority to 3
        IFS1bits.CNDIF = 0; //Clear interrupt flag status bit
        IEC1bits.CNDIE = 1 ; //Enable CN interrupt on port D
        macro_enable_interrupts(); // re-enable interrupts
    }

    void activateServo(){
        //changed
        // fast forward
        if (LED1 == 0 && LED2 == 0 && LED3 == 0 && LED4 == 0){
            SRV_SetPulseMicroseconds0(100);
            SRV_SetPulseMicroseconds1(1500);
        }
        //changed 
        else if (LED1 == 1 && LED2 == 1 && LED3 == 1 && LED4 == 0){
            SRV_SetPulseMicroseconds0(100);
            SRV_SetPulseMicroseconds1(0);
        }
        // changed
        // sharp left
        else if (LED1 == 1 && LED2 == 1 && LED3 == 0 && LED4 == 0){
            SRV_SetPulseMicroseconds0(100);
            SRV_SetPulseMicroseconds1(0);
        }\
//90 DEGREE LEFT
        //changed
        else if (LED1 == 1 && LED2 == 0 && LED3 == 0 && LED4 == 0){
            SRV_SetPulseMicroseconds0(100);
            SRV_SetPulseMicroseconds1(0);
        }
//        else if (LED1 == 1 && LED2 == 0 && LED3 == 0 && LED4 == 1){
//            SRV_SetPulseMicroseconds0(1140);
//            SRV_SetPulseMicroseconds1(2400);
//        }
        // reverse
//        else if (LED1 == 1 && LED2 == 1 && LED3 == 1 && LED4 == 1){
//            SRV_SetPulseMicroseconds0(2400);
//            SRV_SetPulseMicroseconds1(1140);
//        }
//        //45 DEGREE right
//        else if (LED1 == 0 && LED2 == 1 && LED3 == 1 && LED4 == 1){
//            SRV_SetPulseMicroseconds0(1140);
//            SRV_SetPulseMicroseconds1(2500);
//        }
        

        //60 DEGREE RIGHT
        // changed
        else if (LED1 == 0 && LED2 == 0 && LED3 == 1 && LED4 == 1){
            SRV_SetPulseMicroseconds0(0);
            SRV_SetPulseMicroseconds1(1500);
        }
        //SHARP RIGHT
        //changed
        else if (LED1 == 0 && LED2 == 0 && LED3 == 0 && LED4 == 1){
            SRV_SetPulseMicroseconds0(0);
            SRV_SetPulseMicroseconds1(1500);
        }
        //changed
        else if (LED1 == 1 && LED2 == 0 && LED3 == 1 && LED4 == 1){
            SRV_SetPulseMicroseconds0(0);
            SRV_SetPulseMicroseconds1(1500);
    }
        // changed
        else if (LED1 == 1 && LED2 == 1 && LED3 == 0 && LED4 == 1){
            SRV_SetPulseMicroseconds0(100);
            SRV_SetPulseMicroseconds1(0);
        
        }}
    void MIC_Stuff() {            // Compares all values of micVal, finding the highest in a given list. Result stored in micNum
        if (micVal <= 470) {
            micNum = 0x00;
        }
        else if (micVal > 470 && micVal <= 540) {
            if (micNum < 0x01) {
                micNum = 0x01;
            }
        }
        else if(micVal > 540 && micVal <= 610) {
            if (micNum < 0x03) {
                micNum = 0x03;
            }
        }
        else if (micVal > 610 && micVal <= 680) {
            if (micNum < 0x07) {
                micNum = 0x07;
            }
        }
        else if (micVal > 680 && micVal <= 750) {
            if (micNum < 0x0F) {
                micNum = 0x0F;
            }
        }
        else if (micVal > 750 && micVal <= 820) {
            if (micNum < 0x1F) {
                micNum = 0x1F;
            }
        }
        else if (micVal > 820 && micVal <= 890) {
            if (micNum < 0x3F) {
                micNum = 0x3F;


            }
        } 
        else if (micVal > 890 && micVal <= 960) {
            if (micNum < 0x7F) {            
                micNum = 0x7F;
            }
        } 
        else if (micVal > 960) {
            if (micNum < 0xFF) {
                micNum = 0xFF;
            }
        }

    }
