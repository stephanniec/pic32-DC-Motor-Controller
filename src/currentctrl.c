#include "NU32.h"
#include "currentctrl.h"

void isr_init(void){
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

}

float anti_windup(float u_volts){
  if (u_volts > 3.3){
    u_volts = 3.3;
  }
  else if (u_volts < -3.3){
    u_volts = -3.3;
  }
  return u_volts;
}

void new_pwm(float u_volts){
    if (u_volts < 0){
        LATDbits.LATD6 = 1; //ccw
        OC1RS = (unsigned int) ((-u_volts/3.3)*PR3);
    }
    else {
        LATDbits.LATD6 = 0; //cw
        OC1RS = (unsigned int) ((u_volts/3.3)*PR3);
    }
}
