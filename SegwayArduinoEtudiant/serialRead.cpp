#include <Arduino.h>
#include "rtwtypes.h"
//#define TEST_SERIAL_READ
#define SERIAL_COMMAND_MASK 0x3F
#define SERIAL_CONTROL_BYTE_DATA 0x80
#define SERIAL_READ_DATA 4
#define SERIAL_IDLE 0
union byte4_serial {
  // unsigned
  uint8_t ui_8[4];
  uint16_t ui_16[2];
  uint32_t ui_32;
  // signed
  int8_t i_8[4];
  int16_t i_16[2];
  int32_t i_32;
  // float
  float f;
};
typedef struct
{
  uint8_t id;
  uint8_t last_measure;
  int32_t count_measure;
  int32_t count; 
  uint8_t nbSerialData;  
  int serialState;
  byte4_serial data;
  uint8_t current;
} structSerial;
#define NB_MAX_SERIAL 4

structSerial serials[NB_MAX_SERIAL];
extern "C" void setupSerialread(uint8_T id, uint32_T baudRate)
{
  if (id >= NB_MAX_SERIAL)
  {
    return;
  }
  serials[id].id = id; // unusable
  serials[id].last_measure = 0.0;
  serials[id].serialState = SERIAL_IDLE;
  serials[id].count=-1;
  serials[id].count_measure=-1;
  
  switch (id)
  {
  case 0:
    Serial.begin(baudRate);
    break;
  case 1:
    Serial1.begin(baudRate);
    break;
  case 2:
    Serial2.begin(baudRate);
    break;
  case 3:
    Serial3.begin(baudRate);
    break;
  }
}
extern "C" void finishSerialread(uint8_T id){
  // nothing to do for instance
}
extern "C" uint8_T getSerialread(uint8_T id,uint8_T forget)
{
  structSerial *s = &(serials[id]);
  s->count++;
  uint8_t inChar, bits;
  int value;
  switch (s->id)
  { // read next value
  case 0:
    value = Serial.read();
    break;
  case 1:
    value = Serial1.read();
    break;
  case 2:
    value = Serial2.read();
    break;
  case 3:
    value = Serial3.read();
    break;
    default :value=-1;
  }                  // while (value > = 0)
  if (value <0) {
    return 255; // indicate no value
  }
  s->last_measure=forget; // forget last measure
  while (value >= 0) // data is ok if value >=0
  {
    if (value !=forget) {
    s->last_measure = lowByte(value);
    }
    switch (s->id)
    {
    case 0:
      value = Serial.read();
      break;
    case 1:
      value = Serial1.read();
      break;
    case 2:
      value = Serial2.read();
      break;
    case 3:
      value = Serial3.read();
      break;
    }
  } // while (value > = 0)
  return (s->last_measure);
}

#ifdef TEST_SERIAL_READ
void setup()
{
  setupSerialread(2, 115200);
  Serial.begin(115200);

}

void loop()
{
  int32_t last_count=0 ;
  while (1)
  {
    getSerialread(2);
    if (serials[2].count_measure!=last_count) {
      last_count=serials[2].count_measure;
      Serial.print("count:");Serial.print(serials[2].count_measure);
      Serial.print("value:");Serial.println(serials[2].last_measure);
    } 
    delay(10);
  }
}
#endif