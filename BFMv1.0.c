//////////////////////////////////////////
// BAND FABRICATION MACHINE             //
// Program Version 1.0                  //
// Senior Design Team 10                //
//                                      //
// This Program interfaces all of the   //
// control elements of our machine:     //
// led screen, buttons, TubeDetSensor,  //
// CutSensor, relay, and switches.      //
//                                      //
// System Has 3 MODES:                  //
// Mode = 0; Manual Mode                //
// Mode = 1; Automatic Mode             //
// Mode = 2; Maintenance Mode           //
//                                      //
//////////////////////////////////////////


#include <msp430.h> 
#include <string.h>

unsigned int ADC_value=0, ADC_value2=0;

#include <driverlib.h> // Required for the LCD
#include "myGpio.h" // Required for the LCD
#include "myClocks.h" // Required for the LCD
#include "myLcd.h" // Required for the LCD

#define ADC12_SHT_16 0x0200 // 16 clock cycles for sample and hold
#define ADC12_ON 0x0010 // Used to turn ADC12 peripheral on
#define ADC12_SHT_SRC_SEL 0x0200 // Selects source for sample & hold
#define ADC12_12BIT 0x0020 // Selects 12-bits of resolution
#define ADC12_P92 0x000A // Use input P9.2 for analog input 00000100
#define ADC12_P96 0x000E // Use input P9.6 for analog input


#define RED_LED 0x0001 // P1.0 is the red LED FOR TESTING
#define RED_LED_OFF 0xFE // Used to turn-off the P1.0 LED

#define BUTTON11 0x02 // P1.1 is the button
#define BUTTON12 0x04 // P1.2 is the button
#define TubeSensor 0x40 // P1.0 is the TubeSensor

#define ACLK 0x0100 // Timer_A ACLK source
#define UP 0x0010 // Timer_A Up mode

#define ENABLE_PINS 0xFFFE // Required to use inputs and outputs

void ADC_SETUP(void);

int main(void)
{
    /////////////////
    // INITIAL SETUP
    /////////////////
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    int sysmode = 0;            //0 = Manual, 1 = Automatic 2 = Maintenence
    int count = 0;
    int B1,B2 = 0;
    int PassFlg = 0;
    int PassData = 0;
    int PassInit = 0;
    int PassT = 0;
    int PassChk[6] = {4,2,2,3,1,2};
    int PassEnter[6] = {0,0,0,0,0,0};
    int TSval = 0;
    int CSval = 0;
    int ADCct = 0;
    int ADCtrig = 0; //0 is tube sensor and 1 is cut sensor

    //SETUP LCD                 // Initializes General Purpose
    initGPIO();                 // Inputs and Outputs for LCD
    initClocks();               // Initialize clocks for LCD
    myLCD_init();               // Prepares LCD to receive commands

    //*needed* Button Setup
    P1OUT = BUTTON11; //enables buttons
    P1REN = BUTTON11;
    P1OUT = P1OUT | BUTTON12;
    P1REN = P1REN | BUTTON12;

    P1OUT = P1OUT | TubeSensor;
    P1REN = P1REN | TubeSensor;

    P1DIR = RED_LED;

    P9REN = BIT0; // Enables pull-up resistor for pin 9.0 00000100
    P9OUT = BIT0; //
    P2REN = BIT0; // Enables pull-up resistor for pin 9.1
    P2OUT = BIT0; //
    P9REN = BIT6; // Enables pull-up resistor for pin 9.1
    P9OUT = BIT6;



    //Timer Setup
    TA0CCR0 = 20000; // Sets value of Timer_0
    TA0CTL = ACLK + UP; // Set ACLK, UP MODE for Timer_0
    TA0CCTL0 = CCIE; // Enable interrupt for Timer_0
    TA1CCR0 = 60000;  // timing used for timer2
    TA1CTL = ACLK | UP; // Use ACLK, for UP mode

    _BIS_SR(GIE); // Activate interrupts previously enabled


    //*needed* RelayOutput Pin Setup (LED temporarily)
    PM5CTL0 = ENABLE_PINS; // Enable inputs and outputs

    ADC_SETUP(); // Sets up ADC peripheral
    ADC12CTL0 = ADC12CTL0 | ADC12ENC; // Enable conversion
    ADC12CTL0 = ADC12CTL0 | ADC12SC; // Start conversion

    //////////////////
    // PROGRAM CODE
    //////////////////
    while(1)
    {
        if(ADCtrig == 1) // test the Cut Sensor
        {
            if(ADC12MEM0 > 0x800) // Is TubeSensor ON?
            {
                CSval = 1;
            }
            else
            {
                CSval = 0;
            }

        }
        if(ADCtrig == 1) // test the TubeSensor
        {
            if(ADC12MEM1 > 0x900) // Is Cut Sensor ON?
            {
                TSval = 1;
            }
            else
            {
                TSval = 1; // should be 0 but 1 for demo
            }
        }

        if(ADCct < 200)
        {
            ADCct++;
        }
        else
        {
            if(ADCtrig == 0)
            {
                ADCtrig = 1;
                ADC12MCTL0 = ADC12_P96; // P9.6 is analog input
            }
            else
            {
                ADCtrig = 0;
                ADC12MCTL0 = ADC12_P92; // P9.2 is analog input
            }
            ADCct = 0;
            ADC12CTL0 = ADC12CTL0 | ADC12ENC; // Enable conversion
            ADC12CTL0 = ADC12CTL0 | ADC12SC; // Start conversion
        }

        if((BIT0 & P9IN) == 1) // Is CB0 button pushed?
        {
            PassFlg = 1;
            PassData = 1;
            unsigned long delay; // Wait for bouncing to end
             for(delay=0;delay<120000;delay=delay+1);
        }
        if((BIT0 & P2IN) == 1) // Is CB1 button pushed? (CB1 & P2IN) == 1 in if
        {
            PassFlg = 1;
            PassData = 2;
            unsigned long delay; // Wait for bouncing to end
            for(delay=0;delay<120000;delay=delay+1);
        }
        if((BUTTON11 & P1IN) == 0) // Is P11 button pushed?
        {
            PassFlg = 1;
            PassData = 3;
            unsigned long delay; // Wait for bouncing to end
            for(delay=0;delay<120000;delay=delay+1);
        }
        if((BUTTON12 & P1IN) == 0) // Is P12 button pushed?
        {
            PassFlg = 1;
            PassData = 4;
            unsigned long delay; // Wait for bouncing to end
            for(delay=0;delay<120000;delay=delay+1);
        }

        if(sysmode == 0) //When Machine is in Manual Mode (ON boot)
        {
            P1OUT = P1OUT & RED_LED_OFF; // Turn off the red LED
            if(PassFlg == 1) //when a password event has been triggered
            {
                if(PassInit == 0) //on initial startup
                {
                    PassT = 0;
                    PassData = 0;
                    PassInit = 1;
                    myLCD_showChar(' ', 4); // Display "H" in space PassT
                    myLCD_showChar(' ', 5); // Display "H" in space PassT
                    myLCD_showChar(' ', 6); // Display "H" in space PassT
                }
                if(TA0CTL & TAIFG) // If timer has finished counting
                {
                    if(PassData != 0)
                    {
                        PassEnter[PassT] = PassData;
                        char DispVal=PassData+'0';
                        PassData = 0;
                        myLCD_showChar(DispVal, PassT+1 ); // Display Dispval in space PassT+1
                        PassT++;
                    }
                }
                if(PassT == 6)
                {
                    PassFlg = 0;
                    PassT = 0;
                    int diff = 0;
                    if (PassEnter[0] != PassChk[0]) diff +=1;
                    if (PassEnter[1] != PassChk[1]) diff +=1;
                    if (PassEnter[2] != PassChk[2]) diff +=1;
                    if (PassEnter[3] != PassChk[3]) diff +=1;
                    if (PassEnter[4] != PassChk[4]) diff +=1;
                    if (PassEnter[5] != PassChk[5]) diff +=1;
                    if(diff == 0) sysmode = 1;
                    else myLCD_displayNumber(count);
                    PassInit = 0;
                }
            }
            if(TSval == 1) // Is TubeSensor ON?
            {
                if(CSval == 1) // Is cutting sensor on?
                {
                    B1 = 1;
                }
                if(B1 == 1)
                {
                    if(CSval == 0)
                    {

                            count++;
                            myLCD_displayNumber(count);
                            TA0CTL = TA0CTL & (~TAIFG); //resets timer
                            B1 = 0;

                    }
                }

            }


        }

        if(sysmode == 1) //When Machine is in Automatic Mode
        {
            P1OUT = P1OUT | RED_LED; // Turn on the RED LED
            if(PassFlg==1)
            {
                PassFlg = 0;
                sysmode = 0;
                myLCD_displayNumber(count);
            }

        }

        if(sysmode == 2) //When Machine is in Maintenence Mode
        {

        }






    }

}

//************************************************************************
// Timer0 Interrupt Service Routine
//************************************************************************
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_ISR (void)
{
    ADC12CTL0 = ADC12CTL0 | ADC12ENC; // Enable conversion
    ADC12CTL0 = ADC12CTL0 | ADC12SC; // Start conversion
}

//ADC setup function
void ADC_SETUP(void)
{
 ADC12CTL0 = ADC12_SHT_16 | ADC12_ON ; // Turn on, set sample & hold time
 ADC12CTL1 = ADC12_SHT_SRC_SEL; // Specify sample & hold clock source
 ADC12CTL2 = ADC12_12BIT; // 12-bit conversion results
 ADC12MCTL1 = ADC12_P96; // P9.6 is analog input
 ADC12MCTL0 = ADC12_P92; // P9.2 is analog input

}

