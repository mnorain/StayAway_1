/*
 * File:   main.c
 * Author: Mohamed Norain
 * Created on July 13, 2014
 * Description:
 * This program is to control the speed and direction of a continues rotation
 * servo motor by using a potentiometer
 */

#include <plib.h>

// PIC32MX250F128D Configuration Bit Settings
#pragma config PMDL1WAY = OFF           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (4x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (16x Multiplier)
#pragma config UPLLIDIV = DIV_5         // USB PLL Input Divider (5x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = XT             // Primary Oscillator Configuration (XT osc mode)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WISZ_25      // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)


#define STOP_SPEED  53750   // this is the setting for the period of the timer
                            // that corresponds to zero servo motor speed (for
                            // the servo motor in this lab)

#define uS_10  25           // this is the value to be added (or deducted)from
                            // the timer period to result in an increment
                            // (or decrement)of 10 micro second of the pulse width

#define IR_Signal_PERIOD 264   // 26.4 us which will is about 37.9 Khz

#define IR_Signal_LOW_TIME 176   // 17.6 us is the low time in the IR 37.9 Khz signal

#define WAIT_TIMER  while(IFS0bits.T3IF==0){}

#define TIME_4_5_MS    45000     // to be loaded into the timer to give 4.5 ms
                                 // one timer step is 0.1uS

#define TIME_0_56_MS    5600    // to be loaded into the timer to give 0.56 ms
                                 // one timer step is 0.1uS

#define TIME_1_69_MS    16900    // to be loaded into the timer to give 1.69 ms
                                 // one timer step is 0.1uS


//==============================================================================
//delay function
void delay(){
        int count =200;

        while(count){
           count--;
        }
}


//==============================================================================
// this function is to stop the start the 37.8 Khz signal
void start_37khz(void){
    T2CONbits.ON=1;       //start timer two

    OC3CONbits.ON=1;       //start the output compare

}

void stop_37khz(void){
    T2CONbits.ON=0;       //start timer two

    OC3CONbits.ON=0;;       //start the output compare
    PORTBbits.RB9=0;        // ensure that the output compare physical
                            //pin is zero
}


void send_one_cycle(int OneTime, int ZeroTime){
   start_37khz();      // start the 37.9 signal
   PR3= OneTime;  // set timer period to 4.5 ms time
   TMR3=0;   // init timer to zero
   IFS0bits.T3IF=0;
   WAIT_TIMER

   stop_37khz();       // stop the 37.9 signal
   PR3= ZeroTime;    // set timer period to 4.5 ms time
   TMR3=0;             // init timer to zero
   IFS0bits.T3IF=0;    // make sure the interrupt flag is zero
   WAIT_TIMER          //  wait for the timer
}

void send_leader(void){
send_one_cycle(TIME_4_5_MS,TIME_4_5_MS);
}


void send_zero(void){
send_one_cycle(TIME_0_56_MS,TIME_0_56_MS);
}


void send_one(void){
send_one_cycle(TIME_0_56_MS,TIME_1_69_MS);
}


void send_end_bit(void){
send_one_cycle(TIME_0_56_MS,TIME_0_56_MS);
}


//==============================================================================
// The Main function
main() {
        // configure the required bits as digital outputs
        mPORTBSetPinsDigitalOut(BIT_9);

        // Mapping OC3 to bit 9 of PORTB
        RPB9Rbits.RPB9R  = 5;

        PORTBbits.RB9=0; // initialize the output to the servo motor to zero,
                         // When the output compare is still disabled the output
                         // to the servo motor is zero

        //============================================================
        // ADC configuration (refer to lab document for more info)
        mPORTBSetPinsAnalogIn(BIT_13);       // set RB13 as an analogue input

        AD1CON1=0;              // ensure that this register is zero before
                                // starting the configuration  
        AD1CON1bits.ASAM = 1;   // ASAM bit = 1 implies acquisition starts
                                // immediately after last conversion is done
        AD1CON2 = 0;            // Basic configuration in the second
                                // configuration register
        AD1CON3 = 0;            // ensure that this register is zero before
                                // starting the configuration
        AD1CON3bits.ADCS=2;     // the ADC clock is 6xTPB (TPB is the peripheral clock)
                                // TAD= TPB.2.(ADCS<7:0> + 1)= 6.TPB                        
        AD1CHS=0;               // ensure that all the bits in this register
                                // is zero before start the configuration
        AD1CHSbits.CH0SA=11;    // Selecting analouge input 11 (RB13) as the
                                // the input to the ADC
        AD1CSSL=0;              // clear this register for basic configuration
        AD1CON1bits.ON=0;       // Start the ADC

        //======================================================================
        // Timer 2 Configuration :
        // "we will do the bits one by one for better understanding"
        // this timer will be used to generate the 37.9 khz signal
        T2CON=0;              // Ensure that Timer 2 control register is zero
        T2CONbits.TCKPS=1;    // Prescale value is 1:2
        T2CONbits.T32=0;      // Timer 2 will be used as 16 bit stand a lone timer
                              // alternatively it can be combined with Timer 3 to
                              // to make one 32 bit timer
        T2CONbits.TCS=0;      // the timer clock source will be the peripheral
                              // clock (instead of the external input in pin T2CK)

        PR2=IR_Signal_PERIOD;           // this is the complete period for the freq requied by
                           // the IR protocol to generate the 37.8 Hz (264*0.1)
        
        T2CONbits.ON=0;       // Switch on timer 2


        //======================================================================
        // Timer 3 Configuration :
        // "we will do the bits one by one for better understanding"
        // this timer will be used to control the times for bits and start and stop bits
        T3CON=0;              // Ensure that Timer 2 control register is zero
        T3CONbits.TCKPS=1;    // Prescale value is 1:2
        //T3CONbits.T32=0;      // Timer 3 will be used as 16 bit stand a lone timer
                              // alternatively it can be combined with Timer 3 to
                              // to make one 32 bit timer
        T3CONbits.TCS=0;      // the timer clock source will be the peripheral
                              // clock (instead of the external input in pin T2CK)

        PR3=0xFFFF            // Period initialy set to maximum
        
        T3CONbits.ON=0;       // Switch on timer 2

        //======================================================================
        // The Output Compare configuration:
        // starting the output compare, using the 16 bit timer Timer 2, in the
        // continues pulse mode where the OC3 pin will be initialy as zero
        // then when the timer value match OC3R the OC3 pin value will switch
        //  to one and then it will be back to zero when Timer 2 value matches OC3RS

        OC3CON=0;            // ensure that all the bits in
                             // the Output compare control register is 0;
        
        OC3CONbits.OC32=0;   // The output compare is used for comparison with
                             // a 16 bit timer(Timer2)
        OC3CONbits.OCTSEL=0; // Output Compare timer select is 0 , so Timer 2 will
                             // will be used
        OC3CONbits.OCM=5;    // Output Compare Mode is "generate continuous output
                             // pulses on OCx pin"

        OC3R=IR_Signal_LOW_TIME;         // when the timer value matches this register the
                                         // OC3 pin will be set to one
                                         // (rfer to the lab document for more info)

        OC3RS=IR_Signal_PERIOD;        // when the timer value matches this register the
                                       // OC3 pin will be cleared (zero)

        OC3CONbits.ON=0;     // enable the output compare module
        //=====================================================================

        while (1){   // forever       
      
        }
}


