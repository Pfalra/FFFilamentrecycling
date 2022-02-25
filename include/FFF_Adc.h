#ifndef FFF_ADC_H
#define FFF_ADC_H

void FFF_Adc_init();

double FFF_Adc_readVolt(uint8_t channel);

uint16_t FFF_Adc_readTempRaw(uint8_t channel);

#endif