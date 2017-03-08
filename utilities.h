#ifndef UTILITIES__H__
#define UTILITIES__H__

typedef enum { IDLE, PWM, ITEST, HOLD, TRACK }mode_type;
extern mode_type pic_mode; //tell compiler mode_type var pic_mode exists

void setmode(mode_type);
char* getmode(mode_type);

#endif
