#include <stdio.h>
#include "NU32.h"  // Config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "currentsense.h"
#include "utilities.h"  // Declares global variable pic_mode

#define BUF_SIZE 200
#define PLOTPTS 100 // 100 points in ITEST
#define REF_mA 200  // Reference starts at 200mA

mode_type pic_mode = IDLE;          // Allocate memory for var
static volatile int next_dc = 0;    // New duty cycle
static volatile float kp_I = 0;     // Current gains
static volatile float ki_I = 0;
static volatile float ref_arr[PLOTPTS];  // Reference +/- 200mA sqr wave
static volatile float adc_arr[PLOTPTS];  // Measured current
static volatile int store_data = 0;      // Disable data storing

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void){
  switch (pic_mode){
    static int itest_count = 0;
    static float ref_current = REF_mA;
    static float adc_current = 0;
    static float e = 0;
    static float eint = 0; // Position error, integrater error
    static float u = 0;
    static float unew = 0; // Controller output

    case IDLE:
    {
      OC1RS = 0;
      LATDbits.LATD6 = 0;
      break;
    }
    case PWM:
    {
      // Duty cycle and direction bit set according to -100 and 100
      if (next_dc < 0){                        // (-) PWM
        OC1RS = -next_dc*(PR3+1)/100;
        LATDbits.LATD6 = 0;
      }
      else if (next_dc > 0){                   // (+) or no PWM
        OC1RS = next_dc*(PR3+1)/100;
        LATDbits.LATD6 = 1;
      }
      else {
        OC1RS = 0;
        LATDbits.LATD6 = 0;
      }
      break;
    }// End case PWM
    case ITEST:
    {
      if (0 <= itest_count && itest_count < 100){
          //Find reference value and append ref_arr[]
          if (itest_count> 0 && itest_count % 25 == 0){
              ref_current = -ref_current;
          }
          else {
              ref_current = ref_current;
          }

          //Read ADC value (mA) and append adc_arr[]
          adc_current = convert_adc();

          //PI control: I error = R - Y
          e = ref_current - adc_current;
          eint = eint + e;
          u = kp_I*e + ki_I*eint; // u is in mA

          // //Converting controller output u (mA) to A
          // unew = u/1000.0;

          // //Adjust for anti-windup during motor saturation @ +/-3.3V
          // unew = anti_windup(unew);

          //Calculate new pwm and set motor direction using pin D6
          new_pwm(u);

          if(store_data){ //Put mA values into global arrays
            ref_arr[itest_count] = ref_current;
            adc_arr[itest_count] = adc_current;
          }

          itest_count++;
      }//end if

      else{ //Reset counter, r(t), integral error, disable storing data
          itest_count = 0;
          ref_current = REF_mA;
          eint = 0;
          store_data = 0;
          setmode(IDLE);
      }
      break;
    }//end ITEST

    // case HOLD:
    // {
    //   break;
    // }
    // case TRACK:
    // {
    //   break;
    // }
  }//end switch

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
        sprintf(buffer, "%f\n", convert_adc());
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
      case 'g': // Get current gains from MATLAB
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &kp_I);        // kp current define
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &ki_I);        // ki current define
        break;
      }
      case 'h': // Send current gains to MATLAB
      {
        sprintf(buffer, "%f\r\n", kp_I);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", ki_I);
        NU32_WriteUART3(buffer);
        break;
      }
      case 'k': // Test current control gains
      {
        int i = 0;
        store_data = 1; // Enable data storing
        setmode(ITEST);
        sprintf(buffer, "%d\r\n", PLOTPTS); // Size of arrays
        NU32_WriteUART3(buffer);

        __builtin_disable_interrupts(); // Data transfer slow
        for (i = 0; i<PLOTPTS; i++){
          sprintf(buffer, "%d %d\r\n", (int)ref_arr[i], (int)adc_arr[i]);
          NU32_WriteUART3(buffer);
        }
        __builtin_enable_interrupts();

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
