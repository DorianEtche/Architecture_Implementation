#include <Arduino.h>
#include "pwmread_arduino.h"

//#define TEST_WITH_VISUAL_STUDIO_CODE

#define DIRECT_PIN_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)

typedef struct
{
    volatile uint32_t *pinA_register;
    uint32_t pinA_bitmask;
	unsigned long riseTime,fallTime;
	long elapsedTime,count;
} pwmread_t;
#define MAX_NB_PWMREAD 7
volatile uint8_t pwmread_is_active[MAX_NB_PWMREAD] = {0, 0,0,0,0,0,0};
volatile pwmread_t pwmread[MAX_NB_PWMREAD];

//TC1 ch 0
void pwmread_interrupt_handler(int id)
{ // same handler for all pwmread
    static uint8_t s[MAX_NB_PWMREAD];
    if (!pwmread_is_active[id]) {
        return;
    }
    if (DIRECT_PIN_READ(pwmread[id].pinA_register, pwmread[id].pinA_bitmask)) {
       pwmread[id].riseTime=micros();	
	}else {
		pwmread[id].fallTime=micros();
		if (pwmread[id].fallTime>pwmread[id].riseTime) {
		  pwmread[id].elapsedTime=pwmread[id].fallTime-pwmread[id].riseTime;
		  pwmread[id].count++;
		}
	}
}
	
void interrupt_pwmread_0(){
    pwmread_interrupt_handler(0);
}
void interrupt_pwmread_1(){
    pwmread_interrupt_handler(1);
}
void interrupt_pwmread_2(){
    pwmread_interrupt_handler(2);
}
void interrupt_pwmread_3(){
    pwmread_interrupt_handler(3);
}
void interrupt_pwmread_4(){
    pwmread_interrupt_handler(4);
}
void interrupt_pwmread_5(){
    pwmread_interrupt_handler(5);
}
void interrupt_pwmread_6(){
    pwmread_interrupt_handler(6);
}
void interrupt_pwmread_7(){
    pwmread_interrupt_handler(7);
}







// Create an Encoder object and save it to the pointer
extern "C" void pwmreadSetup(uint8_T pinA, uint8_T id)
{
    pinMode(pinA, INPUT_PULLUP);
    switch (id)
    {
    case 0:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_0, CHANGE);
        pwmread_is_active[id] = 1;
        break;
    case 1:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_1, CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 2:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_2, CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 3:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_3 ,CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 4:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_4 ,CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 5:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_5 ,CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 6:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_6 ,CHANGE);
        pwmread_is_active[id] = 1;
        break;
        case 7:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_pwmread_7 ,CHANGE);
        pwmread_is_active[id] = 1;
        break;


		}

    pwmread[id].pinA_register = portInputRegister(digitalPinToPort(pinA));
    pwmread[id].pinA_bitmask = digitalPinToBitMask(pinA);
    pwmread[id].count = 0; //it is not obvious that first value is 0
	pwmread[id].riseTime=micros();

}

// Read the position relative to the old position
// and reset the position to zero
extern "C" int32_T myPwmreadValue(uint8_T id)
{
    return (int32_T) pwmread[id].elapsedTime;
}
/*extern "C" int32_t myPwmreadCount(uint8_T id)
{
    return (int32_t)pwmread[id].count;
}
*/
#ifdef TEST_WITH_VISUAL_STUDIO_CODE
// unused by matlab
void setup()
{
    Serial.begin(115200);
    pwmreadSetup(2, 0);
    // unused
}

void loop()
{
    while (1)
    {
        Serial.print("count:");
        Serial.print(pwmread[0].count);
        Serial.print(" delta t en us:");
        Serial.println(pwmread[0].elapsedTime);
        delay(100);
    }
    // unused
}
#endif