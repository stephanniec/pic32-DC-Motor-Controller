#ifndef CURRENTSENSE__H__
#define CURRENTSENSE__H__

#define SAMPLE_TIME 10

void adc_init();

int read_adc();

float convert_adc();

#endif
