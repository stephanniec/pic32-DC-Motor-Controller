#include <stdio.h>
#include "NU32.h"  // Config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"  // Declares global variable pic_mode
#include "currentsense.h"
#include "currentctrl.h"
#include "positionctrl.h"

#define BUF_SIZE 200
#define PLOTPTS 100 // 100 points in ITEST
#define REF_mA 200  // Reference starts at 200mA

mode_type pic_mode = IDLE;                // Allocate memory
static volatile int next_dc = 0;          // New duty cycle
static volatile float kp_I = 0, ki_I = 0; // Current PI gains
static volatile float ref_arr[PLOTPTS];   // Ref current +/- 200mA sqr wave
static volatile float adc_arr[PLOTPTS];   // Measured current
static volatile float kp_pos = 0;         // Position PID gains
static volatile float ki_pos = 0;
static volatile float kd_pos = 0;
static volatile float err_int = 0;        // Position integral error
static volatile float ref_deg = 0;        // Reference angle in deg
static volatile float holding_current = 0;
static volatile int store_data = 0;       // Disable data storing

void __ISR(_TIMER_2_VECTOR, IPL4SOFT) MasterController(void){
  // 5000 Hz

  switch (pic_mode){
    static int itest_count = 0;
    static float ref_current = REF_mA;
    static float adc_current = 0;
    float e = 0;
    static float eint = 0; // Position error, integrater error
    float u = 0, unew = 0; // Current controller output

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
          // Find reference value
          if (itest_count> 0 && itest_count % 25 == 0){
              ref_current = -ref_current;
          }
          else {
              ref_current = ref_current;
          }

          // Read ADC value (mA)
          adc_current = convert_adc();

          // PI control: I error (mA) = R - Y
          e = ref_current - adc_current;
          eint = eint + e;
          u = kp_I*e + ki_I*eint; // Gains convert mA to mV
          unew = ((u/1000)/3.3)*100; // Converts mV to DC = X%

          if (unew > 100){  // Actuator saturation
            unew = 100;
          }
          else if (unew < -100){
            unew = -100;
          }
          else {
            unew = unew;
          }

          // Calculate new pwm and set motor direction using pin D6
          new_pwm(unew);

          if(store_data){ // Put mA values into global arrays
            ref_arr[itest_count] = ref_current;
            adc_arr[itest_count] = adc_current;
          }

          itest_count++;
      }// end if

      else{ // Reset counter, r(t), int error, & disable storing data
          itest_count = 0;
          ref_current = REF_mA;
          eint = 0;
          store_data = 0;
          setmode(IDLE);
      }
      break;
    }//end ITEST
    case HOLD:
    {
      char buffer[BUF_SIZE];

      // Get desired current
      ref_current = holding_current;
      sprintf(buffer, "current from pcon = %f\r\n", ref_current);
      NU32_WriteUART3(buffer);

      // Read ADC value (mA)
      adc_current = convert_adc();
      sprintf(buffer, "adc read = %f\r\n", adc_current);
      NU32_WriteUART3(buffer);

      // PI control: I error (mA) = R - Y
      e = ref_current - adc_current;
      sprintf(buffer, "e = %f\r\n", e);
      NU32_WriteUART3(buffer);
      eint = eint + e;
      sprintf(buffer, "eint = %f\r\n", eint);
      NU32_WriteUART3(buffer);

      u = kp_I*e + ki_I*eint; // Gains convert mA to mV
      unew = ((u/1000)/3.3)*100; // Converts mV to DC = X%
      sprintf(buffer, "unew = %f\r\n", unew);
      NU32_WriteUART3(buffer);

      if (unew > 100.0){    // Actuator saturation
        unew = 100.0;
      }
      else if (unew < -100.0){
        unew = -100.0;
      }

      sprintf(buffer, "unew converted = %f\r\n", unew);
      NU32_WriteUART3(buffer);

      //Calculate new pwm
      new_pwm(unew);
      break;
    }
    default:
    {
      break;
    }

  }//end switch

  IFS0bits.T2IF = 0;  // Clear master interrupt
}

void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PositionController(void){
  //200Hz, priority lvl 5
  static float actual_deg = 0;
  static float err = 0, err_dot = 0, err_prev = 0;
  static float uhold = 0;

  switch (pic_mode){
    case HOLD:
    {
      char buffer[BUF_SIZE];
      actual_deg = (float) encoder_degree()/100.0;
      //encoder_degree returns int 100x of actual degree

      err = ref_deg - actual_deg;
      pos_dir(err);    // Set spin direction

      err_dot = err - err_prev;  //Assume finite differences
      err_int = err_int + err;

      float current_temp = kp_pos*err + ki_pos*err_int + kd_pos*err_dot;

      if (current_temp > 500){// Max reference current allowed
        holding_current = 500;
      }
      else if (current_temp < -500){
        holding_current = -500;
      }
      else{
        holding_current = current_temp;
      }

      err_prev = err; //Save for comparison later
      break;
    }
    case TRACK:
    {
      break;
    }
    default:
    {
      break;
    }
  }

  IFS0bits.T4IF = 0;  // Clear position interrupt
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
  position_init(); // Setting up Timer4 for position control

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
      case 'i': // Get position gains from MATLAB
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &kp_pos);     // define kp position
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &ki_pos);     // define ki positon
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &kd_pos);     // define kd position
        break;
      }
      case 'j': // Send position gains to MATLAB
      {
        sprintf(buffer, "%f\r\n", kp_pos);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", ki_pos);
        NU32_WriteUART3(buffer);
        sprintf(buffer, "%f\r\n", kd_pos);
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
      case 'l': // Go to angle (deg)
      {
        NU32_ReadUART3(buffer, BUF_SIZE);
        sscanf(buffer, "%f", &ref_deg);
        err_int = 0;  //Resetting for each new entry
        setmode(HOLD);
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
