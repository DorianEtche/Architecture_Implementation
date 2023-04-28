#ifndef _STUDENT_CONTROLLER_H_
#define _STUDENT_CONTROLLER_H_
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif
void matlab_setup_control(float sample_time_second,uint8_T id);
float matlab_update_control(float u1,float u2,uint8_T id);
void matlab_finish_control(uint8_T id);

#ifdef __cplusplus
}
#endif
#endif //_STUDENT_CONTROLLER_H_
