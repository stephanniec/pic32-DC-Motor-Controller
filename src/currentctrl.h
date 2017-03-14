#ifndef CURRENTCONTROL__H__
#define CURRENTCONTROL__H__

void isr_init();            // Set up the ISR
float anti_windup(float);    // Integrater antiwindup when motor saturates
void new_pwm(float);      // Set new pwm and motor direction for ITEST

#endif
