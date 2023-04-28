// Alt+Shift+F To AutoFormat Document
// Arduino libs under :
// C:\Users\UFSDI\AppData\Local\Arduino15
// 1- Arduino : Initialize
// 2- Arduino : Board Manager ( SAM Arm Cortex M3 ) indispensable for IntelliSense
// 3- Arduino : Board Config -> arduino due programming port
// 4-> Open Folder ArduinoChauffe

// define good chauffe

#include "rtwtypes.h"
static float Tech = 0.05; // periode d'echantillonnage en s
static int32_T TechUs = 1e6 * Tech;
static bool showSerialData = false;
#include "adc_arduino.h"
#include "pwmwrite_arduino.h"
void emergencyStop(const char *cause);
/*-------------------------------------------------------------
mesure T from adcValue
 //N =   1.0e+02 *   2.772589381189320   1.494419112912484   0.041831127368204  -0.000115186992265
  // D =   1.000000000000000   1.344424424278247   0.105119432905542   0.000634706463461
 
----------------------------------------------------------------*/
static float adcScale,R2Kohm;
static float numT[4]= {2.772589381189320e2  ,1.494419112912484e2,   0.041831127368204e2,  -0.000115186992265e2};
static float denT[4]= {1.000000000000000  , 1.344424424278247 ,  0.105119432905542  , 0.000634706463461};

void initMesureTDegresC()
{
  uint8_T nb_channels=1;
  uint8_T channels[1]={0};
  uint8_T gains_124[]={1};
  uint8_T shift=9;
  float vRefAdc=3.3;
  float vAlimPont=3.3;
  float fullScaleAdc=4095.0;
  R2Kohm=4.7;
  adcSetup(nb_channels, channels,gains_124, shift);
  adcScale=vRefAdc/(fullScaleAdc*gains_124[0])/vAlimPont; 

 
}
void oneStepMesureTDegresC( float &TDegC)
{
  float mesADC=(float)readAdc(0); // read channel 0;
  float a=adcScale*mesADC; // a = R2/(R1+R2)  , where R1 is thermistor
  float R1Kohm=(1-a)/a *R2Kohm; // estimate R1 kOhm from a
  // 4th order fractional approx of T(°C) = fct( R1 Kohm), S. Ygorra
  float nT=0,dT=0,x=1;
  for (int i=0;i<4;i++) {
     nT+=numT[i]*x;
     dT+=denT[i]*x;
     x*=R1Kohm; 
  }
  TDegC=nT/dT;
}

/*-----------------------------------------------------------
chauffes part
-----------------------------------------------------------*/

static uint8_T chauffePwmPin = 34, chauffeId = 0;
static float chauffeUAlimVolt = 12.0,chauffeUmaxVolt =3.0,chauffeUminVolt=0;
static float chauffePwmScale ,chauffePwmOffset , chauffePwm;
static float chauffePwmPeriodUs = 500.0; // 200 us => 5 Khz to ear chauffes, choose 50 us => 20Khz to obtain silent work
/*-----------------------------------------------------------
 control part
-----------------------------------------------------------*/
static float reftempDeg, tempDeg = 0;
void initChauffeControl( float Te)
{
    // parametres d'un regulateur a temps continu convenable pour le controle du moteur
    // determiner le type de regulateur, et en deduire la pulsation au gain unite probable pour cette loi de commande
    // inventer des experiences permettant de trouver le modèle du systeme et de recalculer ce regulateur
     // initialiser les variables et parametres en z^-1 du controleur left-right
}
void oneStepChauffeControl(float &u)
{
  // a modifier, calculer u = commande = fct(reference : reftempDeg , mesure :tempDeg)
}
static int nbPwm=0;
static float refU=0, stepU= 1;

void initChauffe()
{
    float pwmMin=-1, pwmMax=1 , uMax=chauffeUAlimVolt, uMin=0;
    chauffePwmScale=(pwmMax-pwmMin)/(chauffeUAlimVolt-chauffeUminVolt);
    chauffePwmOffset = pwmMin;
  uint8_t useComplementaryPin=0;
  pwmSetup(chauffePwmPin, useComplementaryPin, chauffePwmPeriodUs, chauffePwmPeriodUs / 2, chauffeId);
  pwmWrite(chauffePwmOffset, chauffeId);
  nbPwm = 1;
}

void applyChauffe(float &u)
{
  u = min(u, chauffeUmaxVolt);
  u = max(u, chauffeUminVolt);
  chauffePwm = u * chauffePwmScale+chauffePwmOffset;
  pwmWrite(chauffePwm, chauffeId);
}

/*-----------------------------------------------------------
Serial part
-----------------------------------------------------------*/
#define SER_TOGGLE_SHOW 'T'
#define SER_INCREASE_U '+'
#define SER_DECREASE_U '-'

void initSerial()
{
  Serial.begin(115200);
  // show list of instructions on Serial
  Serial.println("%%TOGGLE SHOW:T");
  Serial.println("%%INCREASE/DECREASE U:+/-");
  
}
void oneStepSerialRead()
{
  uint8_T v = Serial.read();
  switch (v)
  {
  case SER_TOGGLE_SHOW:
    if (showSerialData) {
      Serial.println("];%%end of data");      
    } else{
      Serial.println("data=[");      
    }
    showSerialData=!showSerialData;
    return;
   case SER_INCREASE_U:
    refU+=stepU;
    refU=min(refU, chauffeUmaxVolt);
    return;
   case SER_DECREASE_U:
    refU-=stepU;
    refU=max(refU, chauffeUminVolt);
    return;

  }
  
}
/*---------------------------------------------------------------
WHOLE SYSTEM PART
----------------------------------------------------------------*/

void setup()
{
  initSerial();
  initMesureTDegresC(); 
  initChauffe(); 
  initChauffeControl(Tech);
}
static float THB_MAX_RD = 45.0 * M_PI / 180.0;
void loop()
{
  char msg[100];
  int count = 0;
  float u=0,tempDeg,adcValue;
  
  while (1)
  {
    uint32_t t_us = micros();
    oneStepSerialRead();
    oneStepMesureTDegresC(tempDeg);
    oneStepChauffeControl(u); // do nothing for instance
    float uTotal=u+refU;
    applyChauffe(uTotal);
    if (showSerialData)
    {
      // affichage data
      sprintf(msg, "%.2f,%.2f", (float)tempDeg, (float)uTotal);
      Serial.println(msg);
    }
    // wait next sample Time
    int32_t deltaTus = t_us + TechUs - micros();
    if (deltaTus > 1000)
    {
      Sleep(deltaTus / 1000);
    }
    else if (deltaTus < 0)
    {
      char msg[100];
      sprintf(msg, "%% OVERRUN:+%f ms", (float)-deltaTus * 0.001);
      emergencyStop(msg);
    }

    count++;
  }

}

void emergencyStop(const char *cause)
{

  for (int id = 0; id < nbPwm; id++)
  {
    pwmStop(id);
  }
  Serial.print("Stop:");
  Serial.println(cause);

  while (1)

    ; // never go out of this function
}
