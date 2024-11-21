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
