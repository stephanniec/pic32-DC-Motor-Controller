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
      else if (next_dc > 0){                   // + or no PWM
        OC1RS = next_dc*(PR3+1)/100;
        LATDbits.LATD6 = 1;
      }
      else {
        OC1RS = 0;
        LATDbits.LATD6 = 0;
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

  adc_init();      // Initializing B0
  encoder_init();  // Initializing SPI4
  isr_init();      // Setting up Timer2 and Timer3 for ISR & D0 PWM

  __builtin_enable_interrupts();

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // Expect next character to be menu command
    NU32_LED2 = 1;                   // Clear the error LED
    switch (buffer[0]) {
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
        NU32_ReadUART3(buffer, BUF_SIZE); // Read new duty cycle
        sscanf(buffer, "%d", &next_dc);
        setmode(PWM);
        break;
      }
      case 'p': // Unpower motor
      {
        setmode(IDLE);
        break;
      }
      case 'q':
      {
        // Handle q for quit.
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
