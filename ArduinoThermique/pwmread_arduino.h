#ifndef _PWMREAD_ARDUINO_H_
#define _PWMREAD_ARDUINO_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

//void pwmreadISR1();
//void pwmreadISR0();
void pwmreadSetup(uint8_T pinA, uint8_T id);
int32_T myPwmreadValue(uint8_T id);
//int32_T myPwmreadCount(uint8_T id);
#ifdef __cplusplus
}
#endif
#endif //_PWMREAD_ARDUINO_H_