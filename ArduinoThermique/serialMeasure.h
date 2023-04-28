#ifndef _SERIAL_MEASURE_H_
#define _SERIAL_MEASURE_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif
void setupSerialMeasure(uint8_T id, uint32_T baudRate);
void finishSerialMeasure(uint8_T id);
float getSerialMeasure(uint8_T id);


#ifdef __cplusplus
}
#endif
#endif //_SERIAL_MEASURE_H_
