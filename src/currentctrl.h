#ifndef CURRENTCTRL__H__
#define CURRENTCTRL__H__

void isr_init();            // Set up the ISR
float anti_windup(float);    // Integrater antiwindup when motor saturates
void new_pwm(float);      // Set new pwm and motor direction for ITEST
void new_pwm_pos(float);
#endif
