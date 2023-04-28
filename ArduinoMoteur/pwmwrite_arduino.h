#ifndef _PWM_ARDUINO_H_
#define _PWM_ARDUINO_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void pwmSetup(uint8_T pinA, uint8_T useComplementaryPin, float period_us, float duty_us, uint8_T id);
void pwmWrite(float value,uint8_T id);
void pwmWriteDutyMicros(float duty_us, uint8_T id);
void pwmStop(uint8_T id);
#ifdef __cplusplus
}
#endif
#endif //_PWM_ARDUINO_H