#ifndef _INTERRUPT_TIME_ARDUINO_H_
#define _INTERRUPT_TIME_ARDUINO_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

//void interrupt_timeISR1();
//void interrupt_timeISR0();
void interrupt_timeSetup(uint8_T pinA, uint8_T id,uint8_T mode);
int32_T myInterrupt_timeValue(uint8_T id);
//int32_T myInterrupt_timeCount(uint8_T id);
#ifdef __cplusplus
}
#endif
#endif //_INTERRUPT_TIME_ARDUINO_H_