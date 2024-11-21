/*===================================CPEG222====================================
 * Program:      Project 4 Template
 * Authors:     Tien and Justin
 * Date:        11/07/2024
 * This is a guide that you can use to write your project 4 code
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
#include <xc.h>   //Microchip XC processor header which links to the PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"
#include "ssd.h"
#include "lcd.h"
#include "swt.h"

// Function Declarations
void intializePorts();
void pwmConfig();
void activateServo();
int clock_times[4] = {0,0,0,0};//array to store current time


int main(void) {
    intializePorts();
    pwmConfig();
    LCD_WriteStringAtPos("Team: 16",0,0);
    
//    LCD_WriteStringAtPos("FWD          FWD",1,0);
    while (TRUE) {
        //PS.. It might be a good idea to put this function in a timer ISR later on.
       
       
            IncrementTime();
         SSD();
        LED();
        activateServo();
    }
}
void delay_ms(int milliseconds){
    int i;
    for (i = 0; i < milliseconds * LOOPS_NEEDED_TO_DELAY_ONE_MS; i++){
        
    }
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

    

// Initialize ports on board
void intializePorts() {
    DDPCONbits.JTAGEN = 0; 
    
    /* 
    The following line sets the tristate of Port A bits 0-7 to 0. The LEDs are 
    connected to those pins. When the tristate of a pin is set low, the pin is 
    configured as a digital output. Notice an &= is used in conjunction with
    leading 1s (the FF) in order to keep the other bits of Port A (8-15) in
    their current state. 
    */
    TRISA &= 0xFF00;
    LED_Init();
    LCD_Init();
    SSD_Init();
    SWT_Init();
    
}

void IncrementTime() {
    
    if ((SWT_GetValue(6) == 0 && SWT_GetValue(7) == 0 && 
         SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0) || 
        (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 1 && 
         SWT_GetValue(0) == 1 && SWT_GetValue(1) == 1) ||
        (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 1 && 
         SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0) || 
        (SWT_GetValue(6) == 0 && SWT_GetValue(7) == 0 && 
         SWT_GetValue(0) == 1 && SWT_GetValue(1) == 1)) {
    
       clock_times[0] = 0;
        clock_times[1] = 0;
        clock_times[2] = 0;
        clock_times[3] = 0;

    }
    else{
    clock_times[0]++; // Increment the rightmost digit (ones place)
    
    if (clock_times[0] > 9) {
        clock_times[0] = 0;
        clock_times[1]++; // Increment tens place
    }
    if (clock_times[1] > 5) {
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
}

void SSD() {
    int time = ((clock_times[3] << 12) | (clock_times[2] << 8)
               | (clock_times[1] << 4) | (clock_times[0]));
     if (SWT_GetValue(6) == 0 && SWT_GetValue(7) == 0 && SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0){
           time = 0; 
        }
        else if (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 1 && SWT_GetValue(0) == 1 && SWT_GetValue(1) == 1){
             time = 0;     
        }
        else if (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 1 && SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0){
             time = 0;    
        }
        else if (SWT_GetValue(6) == 0 && SWT_GetValue(7) == 0 && SWT_GetValue(0) == 1 && SWT_GetValue(1) == 1){
            time = 0;  
        }
    
    SSD_WriteDigitsGrouped(time, 0x00);  // Assuming this function writes correctl
    delay_ms(100); // Adjust as needed

}



void pwmConfig() {
    
    // configure Timer X (select the timer, replace X with desired timer number)

    PR2 = 9999; //Period Register.
    T2CONbits.TCKPS = 0; //Timer Prescaler 
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
    OC4RS =  PR2/2;//OC4RS is some fraction of the Period
    OC4R = OC4RS;
    OC4CONbits.ON = 1;       // Start the OC4 module
    
    //Do The same for OC5**************************
    OC5CONbits.OCM = 6;      // PWM mode on OC5; Fault pin is disabled
    OC5CONbits.OCTSEL = 0;   // Select the timer to use as a clock source
    OC5RS =  PR2/2;//OC5RS is some fraction of the Period
    OC5R = OC5RS;
    OC5CONbits.ON = 1;       // Start the OC5 module
   
   
   TRISBbits.TRISB8 = 0; //set servo 0 as output
   TRISAbits.TRISA15 = 0; //set servo 1 as output
   ANSELBbits.ANSB8 = 0; //set servo 0 as digital

   RPB8R = 0x0B; // connect Servo 0 to OC5
   RPA15R = 0x0B;// connect Servo 1 to OC4
    

    //Set up additional timers here if necessary
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


void activateServo(){
    //left
        if((SWT_GetValue(0) == 0 && SWT_GetValue(1) == 0) || (SWT_GetValue(0) == 1 && SWT_GetValue(1) == 1)){
            //stop right mot
            SRV_SetPulseMicroseconds0(0);
            LCD_WriteStringAtPos("STP",1,13);
            //right = 0x0;
        }
        else if (SWT_GetValue(0) == 1 && SWT_GetValue(1) == 0){
            //right mot forward
            SRV_SetPulseMicroseconds0(300);
            //right = 0b1100;
            LCD_WriteStringAtPos("FWD",1,13);
        }
        else if (SWT_GetValue(0) == 0 && SWT_GetValue(1) == 1){
            //right mot reverse
            SRV_SetPulseMicroseconds0(540);
            //right = 0b0011;
            LCD_WriteStringAtPos("REV",1,13);
        }
       
        if ((SWT_GetValue(6) == 0 && SWT_GetValue(7) == 0) || (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 1)){
            //stop left mot
            SRV_SetPulseMicroseconds1(0);
            LCD_WriteStringAtPos("STP",1,0);
            //left = 0x00;
        }    
        else if (SWT_GetValue(6) == 1 && SWT_GetValue(7) == 0){
            //Left mot forward
            SRV_SetPulseMicroseconds1(540);
            //left = 0b00110000;
            LCD_WriteStringAtPos("FWD",1,0);
        }
        else if (SWT_GetValue(6) == 0 && SWT_GetValue(7) == 1){
            //Left mot backwards
            SRV_SetPulseMicroseconds1(300);
            //left = 0b11000000;
            LCD_WriteStringAtPos("REV",1,0);
        }
        
    }
