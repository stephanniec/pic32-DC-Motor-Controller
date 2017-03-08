#include "currentsense.h"
#include <xc.h>

void adc_init(void){
    AD1PCFGbits.PCFG0 = 0;    //Pin B0 is analog input
    AD1CON3bits.ADCS = 2;     //ADC clock per: Tad = 2*(adcs+1)*Tpb
    AD1CON1bits.ADON = 1;     //Turn on A/D converter
}

int read_adc(void){
    unsigned int elapsed = 0, finish_time = 0;
    int adcval = 0, cycle = 0;

    AD1CHSbits.CH0SA = 0; //use Mux A and pin B0 for sampling

    for (cycle = 0; cycle < 5; cycle++){
        AD1CON1bits.SAMP = 1; //start sampling
        elapsed = _CP0_GET_COUNT();
        finish_time = elapsed + SAMPLE_TIME;
        while(_CP0_GET_COUNT() < finish_time){
          ; //sample for 250ns
        }
        AD1CON1bits.SAMP = 0; //stop sampling, start converting
        while(!AD1CON1bits.DONE){
          ; //wait for conversion process to finish
        }
        adcval = adcval + ADC1BUF0;
    }
    return adcval/5;
}

float convert_adc(void){
    int adc_raw = 0, adc_amps = 0;
    adc_raw = read_adc();
    adc_amps = (adc_raw - 447.483345)/(-0.580402);
    return adc_amps;
}
