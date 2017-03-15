#include "NU32.h"
#include "positionctrl.h"

void position_init(void){
  TRISDbits.TRISD6 = 0;    // Motor digital output pin

  //ISR TIMER 4
  T4CONbits.TCKPS = 0b011;     // Timer4 with prescaler N=8
  PR4 = 49999;                 // 200 Hz ISR
  TMR4 = 0;                    // Initial TMR4 count is 0

  //ON
  T4CONbits.ON = 1;        // Turn on Timer4

  //OTHER ISR SETTINGS
  IPC4bits.T4IP = 5;       // Set priority lvl 5
  IFS0bits.T4IF = 0;       // Clear interrupt flag
  IEC0bits.T4IE = 1;       // Enable interrupt
}

void pos_dir(float error){ //Returning abs(error)
    if (error < 0){
        LATDbits.LATD6 = 0; //cw
    }
    else {
        LATDbits.LATD6 = 1; //ccw
    }
}

/*--------------------------------------------------------------------
General notes:
error_position = ref_position - actual_position
if error_position > 0
actual angle too far cw
adjust by changing motor spin direction to ccw

else if error_position < 0
actual angle too far ccw
adjust by changing motor spin direction to cw
---------------------------------------------------------------------*/
