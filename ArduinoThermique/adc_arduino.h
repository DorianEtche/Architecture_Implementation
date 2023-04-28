#ifndef _ADC_ARDUINO_H_
#define _ADC_ARDUINO_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void adcSetup(uint8_T nb_channels, uint8_T channels[],uint8_T gains_124[], uint8_T shift);
uint16_T readAdc(uint8_T num_channel);
#ifdef __cplusplus
}
#endif
#endif //_ADC_ARDUINO_H_
