#include <stdio.h>
#include "NU32.h"  // Config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "currentsense.h"
#include "utilities.h"  // Declares global variable pic_mode

#define BUF_SIZE 200

static volatile int next_dc = 0;  // Global only in this file
mode_type pic_mode = IDLE; // Allocate memory for var

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void){
  switch (pic_mode){
    case IDLE:
    {
      OC1RS = 0;
      LATDbits.LATD6 = 0;
      break;
    }
    case PWM:
    {
      // Duty cycle and direction bit set according to -100 and 100
      if (next_dc < 0){     // - PWM
        OC1RS = -next_dc*(PR3+1)/100;
        LATDbits.LATD6 = 0;
      }
      else {                   // + or no PWM
        OC1RS = next_dc*(PR3+1)/100;
        LATDbits.LATD6 = 1;
      }
      break;
    }// End case PWM

    // case ITEST:
    // {
    //   break;
    // }
    // case HOLD:
    // {
    //   break;
    // }
    // case TRACK:
    // {
    //   break;
    // }
  }
  IFS0bits.T2IF = 0;  // Clear interrupt
}

int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // Cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // Turn off the LEDs
  NU32_LED2 = 1;

  __builtin_disable_interrupts();
  TRISDbits.TRISD0 = 0;    // Output pin
  TRISDbits.TRISD6 = 0;    // Motor digital output pin

  //ISR TIMER 2
  T2CONbits.TCKPS = 0;     // Timer2 with prescaler N=1
  PR2 = 15999;             // 5 kHz ISR
  TMR2 = 0;                // Initial TMR2 count is 0

  //PWM TIMER 3
  T3CONbits.TCKPS = 0;     // Timer3 with prescaler N=1
  PR3 = 3999;              // 20 kHz square wave
  TMR3 = 0;                // Initial TMR3 count is 0

  //OC USING TIMER 3
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin;
  OC1CONbits.OC32 = 0;     // Use a 16-bit timer
  OC1CONbits.OCTSEL = 1;   // Use Timer3
  OC1RS = 1000;            // Duty cycle = OC1RS/(PR3+1) = 25%
  OC1R = 1000;             // Read-only later, initialize before OC1

  //ON
  T2CONbits.ON = 1;        // Turn on Timer2
  T3CONbits.ON = 1;        // Turn on Timer 3
  OC1CONbits.ON = 1;       // Turn on OC1

  //OTHER ISR SETTINGS
  IPC2bits.T2IP = 5;       // Set priority lvl 5
  IFS0bits.T2IF = 0;       // Clear interrupt flag
  IEC0bits.T2IE = 1;       // Enable interrupt

  adc_init();      // Initializing B0
  encoder_init();  // Initializing SPI4
  __builtin_enable_interrupts();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // Expect next character to be menu command
    NU32_LED2 = 1;                   // Clear the error LED
    switch (buffer[0]) {
      case 'q':
      {
        // Handle q for quit.
        setmode(IDLE);
        break;
      }
      case 'a': // Read current sensor (ADC counts)
      {
        sprintf(buffer, "%d\r\n", read_adc());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'b': // Read current sensor (mA)
      {
        sprintf(buffer, "%.3f\r\n", convert_adc());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c': // Read encoder ticks
      {
        sprintf(buffer, "%d\r\n", encoder_ticks());
        NU32_WriteUART3(buffer);  // Send encoder ticks to client
        break;
      }
      case 'd': // Read encoder degrees x100 (divde by 100.0 in matlab)
      {         // Avoid float math for efficiency
        sprintf(buffer, "%d\r\n", encoder_degree());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e': // Reset encoder
      {
        encoder_reset();
        break;
      }
      case 'f': // Set PWM
      {
        setmode(PWM);
        NU32_ReadUART3(buffer, BUF_SIZE); // Read new duty cycle
        sscanf(buffer, "%d", &next_dc);

        sprintf(buffer, "%d\r\n", next_dc); // View input
        NU32_WriteUART3(buffer);
        break;
      }
      case 'p': // Unpower motor
      {
        setmode(IDLE);
        break;
      }
      case 'r': // Return pic32 mode type
      {
        sprintf(buffer, "%s\r\n", getmode(pic_mode));
        NU32_WriteUART3(buffer);
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // Turn on LED2 to indicate an error
        break;
      }
    }// End switch
  }// End while
  return 0;
}
