#ifndef _SERIAL_READ_H_
#define _SERIAL_READ_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif
void setupSerialread(uint8_T id, uint32_T baudRate);
void finishSerialread(uint8_T id);
uint8_T getSerialread(uint8_T id,int forget);


#ifdef __cplusplus
}
#endif
#endif //_SERIAL_READ_H_
