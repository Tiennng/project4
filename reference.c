University of Delaware Mail	Le Tien Nguyen <tienng@udel.edu>
(no subject)
Ethan Wong <ethanjwo@udel.edu>	Thu, Nov 21, 2024 at 8:41 PM
To: Le Tien Nguyen <tienng@udel.edu>
/*===================================CPEG222====================================
 * Program:     Project 4 Template
 * Authors:     Robert Freeman
 * Date:        11/07/2024
 * This is a guide that you can use to write your project 4 code
==============================================================================*/
/*-------------- Board system settings. PLEASE DO NOT MODIFY THIS PART
----------*/
#ifndef _SUPPRESS_PLIB_WARNING          //suppress the plib warning
during compiling
#define _SUPPRESS_PLIB_WARNING
#endif
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1         // System PLL Output Clock
Divider (PLL Divide by 1)
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits
(Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable
(Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator
Configuration (XT osc mode)
#pragma config FPBDIV = DIV_8           // Peripheral Clock Divisor
(Pb_Clk is Sys_Clk/8)
/*----------------------------------------------------------------------------*/
#define SYS_FREQ (80000000L) // 80MHz system clock
#define _80Mhz_ (80000000L)
#define LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz 1426
#define LOOPS_NEEDED_TO_DELAY_ONE_MS
(LOOPS_NEEDED_TO_DELAY_ONE_MS_AT_80MHz * (SYS_FREQ / _80Mhz_))

#define TRUE 1
#define FALSE 0

#define TMR_TIME2    0.025 // 100 ms for each tick
#define TMR_TIME3    0.005 // 20ms for each tick
#define SW6 PORTBbits.RB10
#define SW7 PORTBbits.RB9
#define SW1 PORTFbits.RF5
#define SW0 PORTFbits.RF3
#define IR1 PORTCbits.RC2
#define IR2 PORTCbits.RC1
#define IR3 PORTCbits.RC4
#define IR4 PORTGbits.RG6


int val = 0;
int val2 =0;
int val3 = 0;
int val4 =0;
int servoCounter = 0;
char isOn1 = FALSE;
char isOn2 = FALSE;
char theClap = FALSE;
int clapTwo = 0;

// Libraries
#include <string.h>
#include <xc.h>   //Microchip XC processor header which links to the
PIC32MX370512L header
#include <stdio.h>  // need this for sprintf
#include <sys/attribs.h>
#include "config.h" // Basys MX3 configuration header
#include "led.h"
#include "ssd.h"
#include "lcd.h"
#include "swt.h"
#include "mic.h"
#include "adc.h"


// Function Declarations
void initializePorts();
void pwmConfig();
void activateServo();
void Timer2Setup();
void Timer3Setup();


int main(void) {
    initializePorts();
    pwmConfig();
    Timer2Setup();
    Timer3Setup();

    //LCD_WriteStringAtPos("Team:The Lemon      ", 0, 0); // line 0, position 0


    while (TRUE) {
        if (MIC_Val() > 730 && theClap == FALSE){
            theClap = TRUE;
            LCD_WriteStringAtPos("ONE", 0, 0); // line 0, position 0
        }
        if (theClap == TRUE ) {
            clapTwo++;
        }
        if (MIC_Val() > 730 && clapTwo > 50){
            LCD_WriteStringAtPos("TWO", 0, 0); // line 0, position 0
        }


    if (SW0 == 1 && SW1 == 0){
        LCD_WriteStringAtPos("             FWD", 1, 0); // line 0, position 0
        if ((SW6 == 1 && SW7 == 1) || (SW6 == 0 && SW7 == 0)){
            LCD_WriteStringAtPos("STP          FWD", 1, 0); // line 0,
position 0
            LATA |= 0xFF0C;
            LATA &= 0xFF0F;
        } else if (SW6 == 0 && SW7 == 1){
            LCD_WriteStringAtPos("REV          FWD", 1, 0); // line 0,
position 0
            LATA |= 0xFFCC;
            isOn1 = TRUE;
        } else {
            LCD_WriteStringAtPos("FWD          FWD", 1, 0);
            LATA |= 0xFF3C;
            isOn2 = TRUE;
        }
    }
    if (SW0 == 0 && SW1 == 1){
        LCD_WriteStringAtPos("             REV", 1, 0); // line 0, position 0
        if ((SW6 == 1 && SW7 == 1) || (SW6 == 0 && SW7 == 0)){
            LCD_WriteStringAtPos("STP          REV", 1, 0); // line 0,
position 0
            LATA |= 0xFF03;
            LATA &= 0xFF0F;
        } else if (SW6 == 0 && SW7 == 1){
            LCD_WriteStringAtPos("REV          REV", 1, 0); // line 0,
position 0
            LATA |= 0xFFC3;
            isOn1 = TRUE;
        } else {
            LCD_WriteStringAtPos("FWD          REV", 1, 0);
            LATA |= 0xFF33;
            isOn2 = TRUE;
        }
    }
    if (SW0 == 1 && SW1 == 1){
        LCD_WriteStringAtPos("             STP", 1, 0); // line 0, position 0
        if ((SW6 == 1 && SW7 == 1) || (SW6 == 0 && SW7 == 0)){
            LCD_WriteStringAtPos("STP          STP", 1, 0); // line 0,
position 0
            LATA &= 0xFF00;
        } else if (SW6 == 0 && SW7 == 1){
            LCD_WriteStringAtPos("REV          STP", 1, 0); // line 0,
position 0
            if (isOn1){
                LATA &= 0xFFC0;
                isOn1 = FALSE;
            } else if (!isOn1){
                LATA |= 0xFFC0;
            }
        } else {
            LCD_WriteStringAtPos("FWD          STP", 1, 0);
            if (isOn2){
                LATA &= 0xFF30;
                isOn2 = FALSE;
            } else if (!isOn2){
                LATA |= 0xFF30;
            }
        }
    }
    if (SW0 == 0 && SW1 == 0){
        LCD_WriteStringAtPos("             STP", 1, 0); // line 0, position 0
        if ((SW6 == 1 && SW7 == 1) || (SW6 == 0 && SW7 == 0)){
            LCD_WriteStringAtPos("STP          STP", 1, 0); // line 0,
position 0
            LATA &= 0xFF00;
        } else if (SW6 == 0 && SW7 == 1){
            LCD_WriteStringAtPos("REV          STP", 1, 0); // line 0,
position 0
            if (isOn1){
                LATA &= 0xFFC0;
                isOn1 = FALSE;
            } else if (!isOn1){
                LATA |= 0xFFC0;
            }
        } else {
            LCD_WriteStringAtPos("FWD          STP", 1, 0);
            if (isOn2){
                LATA &= 0xFF30;
                isOn2 = FALSE;
            } else if (!isOn2){
                LATA |= 0xFF30;
            }
        }
    }





        //PS.. It might be a good idea to put this function in a timer
ISR later on.
        activateServo();
    }
}




// Initialize ports on board
void initializePorts() {

    DDPCONbits.JTAGEN = 0; // Required to use Pin RA0 (connected to LED 0) as IO

    LED_Init();
    LCD_Init();
    SSD_Init();
    SWT_Init();
    MIC_Init();

    TRISBbits.TRISB10 = 1; // RB10 (SW6) configured as input
    ANSELBbits.ANSB10 = 0; // RB10 (SW6) disabled analog

    TRISBbits.TRISB9 = 1; // RB9 (SW7) configured as input
    ANSELBbits.ANSB9 = 0; // RB9 (SW7) disabled analog

    TRISFbits.TRISF3 = 1; // RF3 (SW0) configured as input
    TRISFbits.TRISF5 = 1; // RF5 (SW1) configured as input

    //PMOD
    TRISCbits.TRISC1 = 1;
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC3 = 1;
    TRISGbits.TRISG6 = 1;
    ANSELGbits.ANSG6 = 0;

    //MICROPHONER
    TRISBbits.TRISB4 = 1;
    ANSELBbits.ANSB4 = 1;


}

void pwmConfig() {

    // configure Timer X (select the timer, replace X with desired timer number)
    /*
    PRX = ; //Period Register.
    TXCONbits.TCKPS = ; //Timer Prescaler
    TXCONbits.TGATE = 0; // not gated input (the default)
    TXCONbits.TCS = 0; // PBCLK input (the default)
    TXCONbits.ON = 1;  //Turn on Timer
    TMRX = 0; // Set Timer X to 0

    IPCXbits.TXIP = ;  //    priority
    IPCXbits.TXIS = ;  //    subpriority
    IFS0bits.TXIF = 0; //    clear interrupt flag
    IEC0bits.TXIE = 1; //    enable interrupt
    */

    // Configure Output Compare Module 4

    OC4CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC4CONbits.OCTSEL = PR3;   // Select the timer to use as a clock source
    OC4RS = PR3*10/129;//OC4RS is some fraction of the Period
    OC4R = OC4RS;
    OC4CONbits.ON = 1;       // Start the OC4 module

    //Do The same for OC5**************************

    OC5CONbits.OCM = 6;      // PWM mode on OC4; Fault pin is disabled
    OC5CONbits.OCTSEL = PR3;   // Select the timer to use as a clock source
    OC5RS = PR3*10/131;           //OC4RS is some fraction of the Period
    OC5R = OC5RS;
    OC5CONbits.ON = 1;       // Start the OC4 module



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

    val++;
    if (val%10 == 0){
             //delay. At the end of this running, the Counter is
incremented to put it back in the delay loop.
        val2++;                                                 //In
regards to the wraparound functionality, it's the same as MODE1 and
MODE2 for setting clock and
        val=0;
//alarm, except now val2 and val3 have to change with each other,
which is accomplished by an if statement
        }                                                       //that
tests if val2 and val are equal to 0, but val2 isn't 0, and increments
val3
    if (val2%10 == 0 && val2!=0 && val%10==0){
         val3++;
    }
    if (val3%10 == 0 && val2%10 == 0 && val==0){
        val4++;
        val2 = 0;
    }

    SSD_WriteDigits(val%10,val2%10,val3%10,val4%10,0,1,0,0);





    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; // enable interrupt
}

void __ISR(_TIMER_3_VECTOR) Timer3ISR(void) {
    IEC0bits.T3IE = 0; // disable interrupt


    //RPB8R = 0x0B; // connect Servo 0 to OC5 LEFT
    //RPA15R = 0x0B;// connect Servo 1 to OC4
    // Left Motor
    /*
    if (SW6 == 1 && SW7 == 1) {
        OC5RS = PR3*10/131;
    }
    if (SW6 == 1 && SW7 == 0) {
        OC5RS = PR3/10;                     //FWD for left
    }
    if (SW6 == 0 && SW7 == 1) {
        OC5RS = PR3/20;                     //REV for left motor
    }
    if (SW6 == 0 && SW7 == 0) {
        OC5RS = PR3*10/131;
    }

    //Right Motor
    if (SW0 == 1 && SW1 == 1) {
        OC4RS = PR3*10/129;
    }
    if (SW0 == 1 && SW1 == 0) {
       OC4RS = PR3/20;                     //FWD for right
    }
    if (SW0 == 0 && SW1 == 1) {
        OC4RS = PR3/10;                     //REV for right motor
    }
    if (SW0 == 0 && SW1 == 0) {
        OC4RS = PR3*10/129;
    }

    */




    if (!SW6) {

    if (IR1 == 1 && IR2 == 1 && IR3 == 1 && IR4 == 1) {
        OC4RS = PR3/10;                     //REV for right motor
        OC5RS = PR3/20;                     //REV for left motor
    }

    if (IR1 == 0 && IR2 == 0 && IR3 == 0 && IR4 == 0) {
        OC4RS = PR3*10/129;                 //Stop
        OC5RS = PR3*10/131;                 //Stop
    }

    if (IR1 == 1 && IR2 == 0 && IR3 == 0 && IR4 == 0) {
        OC4RS = PR3/20;                     //FWD for right
        OC5RS = PR3/10;                     //FWD for left
    }
    if (IR1 == 0 && IR2 == 1 && IR3 == 1 && IR4 == 0) {
        OC4RS = PR3/20;                     //FWD for right
        OC5RS = PR3/10;                     //FWD for left
    }
    if (IR1 == 0 && IR2 == 0 && IR3 == 0 && IR4 == 1) {
        OC4RS = PR3/20;                     //FWD for right
        OC5RS = PR3/10;                     //FWD for left
    }
    if (IR1 == 0 && IR2 == 0 && IR3 == 0 && IR4 == 1) {
        OC4RS = PR3/20;                     //FWD for right
        OC5RS = PR3/20;                     //REV for left motor
    }
    if (IR1 == 1 && IR2 == 1 && IR3 == 1 && IR4 == 0) {
        OC4RS = PR3/20;                     //FWD for right
        OC5RS = PR3/20;                     //REV for left motor
    }
    if (IR1 == 0 && IR2 == 0 && IR3 == 1 && IR4 == 1) {
        OC4RS = PR3/10;                     //REV for right motor
        OC5RS = PR3/10;                     //FWD for left
    }
    if (IR1 == 0 && IR2 == 1 && IR3 == 1 && IR4 == 1) {
        OC4RS = PR3/10;                     //REV for right motor
        OC5RS = PR3/10;                     //FWD for left
    }
    }


    else {
        OC4RS = PR3*10/129;                 //Stop
        OC5RS = PR3*10/130;                 //Stop
    }

    IFS0bits.T3IF = 0; // clear interrupt flag
    IEC0bits.T3IE = 1; // enable interrupt
}


void Timer2Setup()
{
  PR2 = (int)(((float)(TMR_TIME2 * PB_FRQ) / 256) + 0.5); //set period
register, generates one interrupt every 1000 ms
  TMR2 = 0;                           //    initialize count to 0
  T2CONbits.TCKPS = 0b111;                //    1:256 prescale value
  T2CONbits.TGATE = 0;                //    not gated input (the default)
  T2CONbits.TCS = 0;                  //    PCBLK input (the default)
  T2CONbits.ON = 1;                   //    turn on Timer1
  IPC2bits.T2IP = 7;                  //    priority
  IPC2bits.T2IS = 3;                  //    subpriority
  IFS0bits.T2IF = 0;                  //    clear interrupt flag
  IEC0bits.T2IE = 1;                  //    enable interrupt
  macro_enable_interrupts();
}

void Timer3Setup()
{
  PR3 = (int)(((float)(TMR_TIME3 * PB_FRQ) / 256) + 0.5); //set period
register, generates one interrupt every 3 ms
  TMR3 = 0;                           //    initialize count to 0
  T3CONbits.TCKPS = 0b111;                //    1:64 prescale value
  T3CONbits.TGATE = 0;                //    not gated input (the default)
  T3CONbits.TCS = 0;                  //    PCBLK input (the default)
  T3CONbits.ON = 1;                   //    turn on Timer1
  IPC3bits.T3IP = 7;                  //    priority
  IPC3bits.T3IS = 3;                  //    subpriority
  IFS0bits.T3IF = 0;                  //    clear interrupt flag
  IEC0bits.T3IE = 1;                  //    enable interrupt
  macro_enable_interrupts();          //    enable interrupts at CPU
}

void activateServo(){
    if(SWT_GetValue(6)){
        //If Switch 6 is on, move servo...
        //Replace X with your Timer Number
        //OC4RS = (int) PRX / 25;
    }else{
        //If Switch 6 is off, stop moving servo...
        //Replace X with your Timer Number
        //OC4RS = (int) PRX / 13.8;
    }
}
