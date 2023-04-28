// Alt+Shift+F To AutoFormat Document
//________________________________________________
// pour Intelllisense : Palette de commandes preferences ->afficher les parametres, ou open settings  (en json )
// et selectionner : "C_Cpp.intelliSenseEngine": "default",
//______________________________________________
// Arduino libs under :
// C:\Users\UFSDI\AppData\Local\Arduino15
// 1- Arduino : Initialize
// 2- Arduino : Board Manager ( SAM Arm Cortex M3 ) indispensable for IntelliSense
// 3- Arduino : Board Config -> arduino due programming port
// 4-> Open Folder ArduinoMotor

// define good motor
#define LEFT_MOTOR_SEGWAY
//#define MOTOR_SALLE_AUTOM

#include "rtwtypes.h"

static float Tech = 0.01; // periode d'echantillonnage en s
static int32_T TechUs = 1e6 * Tech;
static bool showSerialData = false;
#include "encoder_arduino.h"
#include "pwmwrite_arduino.h"
void emergencyStop(const char *cause);
/*-------------------------------------------------------------
Derivative filter part
----------------------------------------------------------------*/
static float dv_tau; // variables globales derivateur filtre
void initDerivativeFilter(float tauSec, float TECH)
{
  dv_tau = tauSec;
  // expression du filtre en p : p /( tau . p+1), a traduit en z^-1
}

static float thm_2 = 0;
void oneStepDerivativeFilter(float thm, float &vthm)
{
  vthm = vthm - 0.965*thm_2 + thm * 1.035;
  thm_2 = thm;
}

/*-----------------------------------------------------------
motors part
-----------------------------------------------------------*/

#ifdef LEFT_MOTOR_SEGWAY
static uint8_T motorPwmPin = 34, motorEncoderPinA = 6, motorEncoderPinB = 7, motorId = 0;
static float motorUAlimVolt = 5.0,motorUmaxVolt =5.0;
#endif
#ifdef MOTOR_SALLE_AUTOM
static uint8_T motorPwmPin = 34, motorEncoderPinA = 10, motorEncoderPinB = 11, motorId = 0;
static float motorUAlimVolt = 7.5,motorUmaxVolt =6.0;
#endif
static float motorPwmScale = +1 / motorUAlimVolt,   motorPwm;
static float motorPwmPeriodUs = 50.0; // 200 us => 5 Khz to ear motors, choose 50 us => 20Khz to obtain silent work
static float motorEncoderScale = 0.5 * (M_PI * 2.0) / (120.0 * 32.0);
/*-----------------------------------------------------------
Motor control part
-----------------------------------------------------------*/
static float refthm = 5, thm = 0, y_1 = 0,y_2 = 0, x_1 = 0, x_2 = 0;
void initMotorControl( float Te)
{
    // parametres d'un regulateur a temps continu convenable pour le controle du moteur
    // determiner le type de regulateur, et en deduire la pulsation au gain unite probable pour cette loi de commande
    // inventer des experiences permettant de trouver le modèle du systeme et de recalculer ce regulateur
     // initialiser les variables et parametres en z^-1 du controleur left-right
}
void oneStepMotorControl(float &u)
{
  x_1 = (refthm - thm)*9;
  y_1 = y_2 + x_1 * 77 - x_2 * 77; 
  u = 60 * y_1*1.035-y_2*0.965+u;
  y_2 = y_1;
  x_2 = x_1;
}
static int nbPwm=0;
void initMotor()
{

  encoderSetup(motorEncoderPinA, motorEncoderPinB, motorId);
  pwmSetup(motorPwmPin, 1, motorPwmPeriodUs, motorPwmPeriodUs / 2, motorId);
  pwmWrite(0.0, motorId);
  nbPwm = 1;
}

void applyControl(float &u)
{
  u = min(u, motorUmaxVolt);
  u = max(u, -motorUmaxVolt);
  motorPwm = u * motorPwmScale;
  pwmWrite(motorPwm, motorId);
}

/*-----------------------------------------------------------
Serial part
-----------------------------------------------------------*/
#define SER_TOGGLE_SHOW 'T'
void initSerial()
{
  Serial.begin(115200);
  // show list of instructions on Serial
  Serial.println("%%TOGGLE SHOW:T");
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
  }
}
/*---------------------------------------------------------------
WHOLE SYSTEM PART
----------------------------------------------------------------*/

void setup()
{
  initSerial();

  initDerivativeFilter(0.01, Tech); // juste pour la partie identification
  // initNotchFilter(1 / 70.0, 0.4, 1.2, Tech); // 70 rd/s notch filter, csi num=0.2,csi den =0.6;
  initMotor(); 
  initMotorControl(Tech);
}
static float THB_MAX_RD = 45.0 * M_PI / 180.0;
void loop()
{
  char msg[100];
  int count = 0;
  float u,vthm;
  
  while (1)
  {
    uint32_t t_us = micros();
    oneStepSerialRead();
    int32_T motorEncoder = readEncoder(motorId);
    thm=motorEncoder*motorEncoderScale; // position angulaire en radians
    oneStepDerivativeFilter(thm, vthm); // estime la vitesse du moteur depuis la position
    // application de u passant alternativement de 1 a 2 volts
    //if ((count & 255) < 128)
    //{
    //  u = 1;
    //}
    //else
    //{
    //  u = 2;
    //}
    oneStepMotorControl(u); // do nothing for instance
    

    applyControl(u);
    if (showSerialData)
    {
      // affichage data
      sprintf(msg, "%.4f,%.2f,%.4f", (float)thm, (float)u,(float)vthm);
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
