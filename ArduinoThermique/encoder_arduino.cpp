#include <Arduino.h>
#include "encoder_arduino.h"

//#define TEST_WITH_VISUAL_STUDIO_CODE

#define DIRECT_PIN_READ(base, mask) (((*(base)) & (mask)) ? 1 : 0)

typedef struct
{
    volatile RoReg *pinA_register;
    volatile RoReg *pinB_register;
    uint32_T pinA_bitmask;
    uint32_T pinB_bitmask;
    uint8_T state;
    int position;
} Encoder_state_t;
#define MAX_NB_ENCODER 2
volatile uint8_T encoder_is_active[MAX_NB_ENCODER] = {0, 0};
volatile Encoder_state_t encoders[MAX_NB_ENCODER];

//TC1 ch 0
void codeur_interrupt_handler(int id)
{ // same handler for all encoders
    static uint8_T s[MAX_NB_ENCODER];
    if (!encoder_is_active[id]) {
        return;
    }
    s[id] = encoders[id].state & 3;
    if (DIRECT_PIN_READ(encoders[id].pinA_register, encoders[id].pinA_bitmask))
        s[id] |= 4;
    if (DIRECT_PIN_READ(encoders[id].pinB_register, encoders[id].pinB_bitmask))
        s[id] |= 8;
    switch (s[id])
    {
        //
    case 0:
    case 5:
    case 10:
    case 15: // 00 00 , 01:01 , 10 10 , 11 11
        break;
    case 1:
    case 7:
    case 8:
    case 14: // 00:01 , 01:11 , 10:00 , 11:10
        encoders[id].position++;
        break;
    case 2:
    case 4:
    case 11:
    case 13: // 00:10 , 01:00 , 10:11 , 11:01
        encoders[id].position--;
        break;
    case 3:
    case 12:
        encoders[id].position += 2;
        break; // 00:11 , 11:00
    default:
        encoders[id].position -= 2;
        break; // 01:10 , 10:01
    }
    encoders[id].state = (s[id] >> 2);
}
void interrupt_codeur_0(){
    codeur_interrupt_handler(0);
}
void interrupt_codeur_1(){
    codeur_interrupt_handler(1);
}
// Create an Encoder object and save it to the pointer
extern "C" void encoderSetup(uint8_T pinA, uint8_T pinB, uint8_T id)
{
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    switch (id)
    {
    case 0:
        encoder_is_active[id] = 1;
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_codeur_0, CHANGE);
        break;
    case 1:
        attachInterrupt(digitalPinToInterrupt(pinA), interrupt_codeur_1, CHANGE);
           encoder_is_active[id] = 1;
        break;
    }

    encoders[id].pinA_register = portInputRegister(digitalPinToPort(pinA));
    encoders[id].pinB_register = portInputRegister(digitalPinToPort(pinB));
    encoders[id].pinA_bitmask = digitalPinToBitMask(pinA);
    encoders[id].pinB_bitmask = digitalPinToBitMask(pinB);
    encoders[id].state = 0; //it is not obvious that first value is 0
    // better to correctly initialize state
    if (DIRECT_PIN_READ(encoders[id].pinA_register, encoders[id].pinA_bitmask))
        encoders[id].state |= 1;
    if (DIRECT_PIN_READ(encoders[id].pinB_register, encoders[id].pinB_bitmask))
        encoders[id].state |= 2;
    encoders[id].position = 0;

}

// Read the position relative to the old position
// and reset the position to zero
extern "C" int32_T readEncoder(uint8_T id)
{
    int32_T currentPosition = encoders[id].position;
    //encoders[id].position = 0; // why ?... program has to integrate encoder to get position
    return currentPosition;
}
#ifdef TEST_WITH_VISUAL_STUDIO_CODE
// unused by matlab
void setup()
{
    Serial.begin(115200);
    encoderSetup(2, 3, 0);
    // unused
}

void loop()
{
    while (1)
    {
        Serial.print("state:");
        Serial.print(encoders[0].state);
        Serial.print(",pos:");
        Serial.println(encoders[0].position);
        delay(100);
    }
    // unused
}
#endif