#include <stdio.h>
#include "utilities.h"
#include <xc.h>

void setmode(mode_type choice){
    pic_mode = choice;
}

char* getmode(mode_type pmode){
    if (pmode == IDLE){
        return "IDLE";
    }
    else if (pmode == PWM){
        return "PWM";
    }
    else if (pmode == ITEST){
        return "ITEST";
    }
    else if (pmode == HOLD){
        return "HOLD";
    }
    else if (pmode == TRACK){
        return "TRACK";
    }
}


