#include <Arduino.h>
#include "interrupt_time.h"

//#define TEST_WITH_VISUAL_STUDIO_CODE

#define DIRECT_PIN_READ_INTR(base, mask) (((*(base)) & (mask)) ? 1 : 0)
#define RISING_TO_RISING 0
#define FALLING_TO_FALLING 1
#define ANY_TO_ANY 2
#define RISING_TO_FALLING 3
#define FALLING_TO_RISING 4
typedef struct
{
    uint8_T mode; // 0<=> temps a 1, 1 <=> temps a 0, 2<=> temps entre 2 derniers basculements
    volatile uint32_t *pinA_register;
    uint32_t pinA_bitmask;
	unsigned long lastTime,lastTimeOn,lastTimeOff;
	long count;
    long elapsedTime,elapsedTimeOn,elapsedTimeOff;
    
    
} interrupt_time_t;
#define MAX_NB_INTERRUPT_TIME 7
volatile uint8_t interrupt_time_is_active[MAX_NB_INTERRUPT_TIME] = {0, 0,0,0,0,0,0};
volatile interrupt_time_t interrupt_time[MAX_NB_INTERRUPT_TIME];

//TC1 ch 0
void interrupt_time_interrupt_handler(int id)
{ // same handler for all interrupt_time
    static uint8_t s[MAX_NB_INTERRUPT_TIME];
    if (!interrupt_time_is_active[id]) {
        return;
    }
	interrupt_time[id].elapsedTime=micros()-interrupt_time[id].lastTime;
    interrupt_time[id].lastTime+=interrupt_time[id].elapsedTime;
	if (interrupt_time[id].mode<=ANY_TO_ANY) {
	// nothing to do	
	   return;
	}	
	if (DIRECT_PIN_READ_INTR(interrupt_time[id].pinA_register, interrupt_time[id].pinA_bitmask)){
    // rising edge
	    interrupt_time[id].lastTimeOn=interrupt_time[id].lastTime;
    	interrupt_time[id].elapsedTimeOff=interrupt_time[id].lastTimeOn-interrupt_time[id].lastTimeOff;
	} else {
    // falling edge
	    interrupt_time[id].lastTimeOff=interrupt_time[id].lastTime;
    	interrupt_time[id].elapsedTimeOn=interrupt_time[id].lastTimeOff-interrupt_time[id].lastTimeOn;
	}
    	
	interrupt_time[id].count++;    
}
	
void interrupt_interrupt_time_0(){
    interrupt_time_interrupt_handler(0);
}
void interrupt_interrupt_time_1(){
    interrupt_time_interrupt_handler(1);
}
void interrupt_interrupt_time_2(){
    interrupt_time_interrupt_handler(2);
}
void interrupt_interrupt_time_3(){
    interrupt_time_interrupt_handler(3);
}
void interrupt_interrupt_time_4(){
    interrupt_time_interrupt_handler(4);
}
void interrupt_interrupt_time_5(){
    interrupt_time_interrupt_handler(5);
}
void interrupt_interrupt_time_6(){
    interrupt_time_interrupt_handler(6);
}
void interrupt_interrupt_time_7(){
    interrupt_time_interrupt_handler(7);
}










// Create an Encoder object and save it to the pointer
extern "C" void interrupt_timeSetup(uint8_T pinA, uint8_T id, uint8_T mode)
{
    pinMode(pinA, INPUT_PULLUP);
	interrupt_time[id].pinA_register = portInputRegister(digitalPinToPort(pinA));
    interrupt_time[id].pinA_bitmask  = digitalPinToBitMask(pinA);
    interrupt_time[id].mode=mode;
	int intrType=CHANGE;
    if (mode==RISING_TO_RISING) {
       intrType=RISING;
    }
    else if (mode==FALLING_TO_FALLING) {
       intrType=FALLING;
    }
            
    switch (id)
    {
    case 0:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_0, intrType);
        interrupt_time_is_active[id] = 1;
        break;
    case 1:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_1, intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 2:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_2, intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 3:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_3 ,intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 4:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_4 ,intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 5:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_5 ,intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 6:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_6 ,intrType);
        interrupt_time_is_active[id] = 1;
        break;
        case 7:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_interrupt_time_7 ,intrType);
        interrupt_time_is_active[id] = 1;
        break;


		}
    interrupt_time[id].count = 0; //it is not obvious that first value is 0
	interrupt_time[id].lastTime=micros();
	interrupt_time[id].lastTimeOn=interrupt_time[id].lastTime;
	interrupt_time[id].lastTimeOff=interrupt_time[id].lastTime;
	
}

// Read the position relative to the old position
// and reset the position to zero
extern "C" int32_T myInterrupt_timeValue(uint8_T id)
{
	if (interrupt_time[id].mode <=ANY_TO_ANY ){
      return (int32_T) interrupt_time[id].elapsedTime;
	}
	if (interrupt_time[id].mode ==RISING_TO_FALLING ){
		return (int32_T) interrupt_time[id].elapsedTimeOn;
    }
	if (interrupt_time[id].mode ==FALLING_TO_RISING ){
		return (int32_T) interrupt_time[id].elapsedTimeOff;
    }
	
}
/*extern "C" int32_t myInterrupt_timeCount(uint8_T id)
{
    return (int32_t)interrupt_time[id].count;
}
*/
#ifdef TEST_WITH_VISUAL_STUDIO_CODE
// unused by matlab
void setup()
{
    Serial.begin(115200);
    interrupt_timeSetup(2, 0, 1);
    // unused
}

void loop()
{
    while (1)
    {
        Serial.print("count:");
        Serial.print(interrupt_time[0].count);
        Serial.print(" delta t en us:");
        Serial.println(interrupt_time[0].elapsedTime);
        delay(100);
    }
    // unused
}
#endif