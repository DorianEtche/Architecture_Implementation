// Alt+Shift+F To AutoFormat Document
// Arduino libs under :
// C:\Users\UFSDI\AppData\Local\Arduino15
// 1- Arduino : Initialize
// 2- Arduino : Board Manager ( SAM Arm Cortex M3 )
// 3- Arduino : Board Config -> arduino due programming port
// 4-> Open Folder Segway Arduino Etudiant 
#define CONTROL_SEGWAY
#ifdef CONTROL_SEGWAY
#define TANGENTIAL_CONTROL_FAST 2
#define TANGENTIAL_CONTROL_SLOW 1
#define TANGENTIAL_CONTROL_NONE 0
#define LR_CONTROL_ON 2
#define LR_CONTROL_OFF 1
#define NOTCH_FILTER_ON 2
#define NOTCH_FILTER_OFF 1

#include "rtwtypes.h"
static float Tech = 0.01; // periode d'echantillonnage en s
static int32_T TechUs = 1e6 * Tech;
static bool showSerialData = true;
#define USE_MPU_6050
#include "encoder_arduino.h"
#include "serialRead.h"
#include "pwmwrite_arduino.h"
#include <Wire.h>
#ifdef USE_MPU_6050
const int MPU_addr = 0x68;
static float scaleGyro, scaleAcc;
static bool restartMPU6050 = false;
void initMPU6050()
{

#ifdef SHOW_INIT_MUP6050_TIME
  uint32_t t0 = micros();
#endif
  scaleGyro = -(250.0 / 32768.0) * (M_PI / 180.0);
  scaleAcc = -2.0 / 32768.0;

  if (restartMPU6050)
  {
    Wire.end();
  }
  Wire.begin();
  restartMPU6050 = true; // after first call, restart is always true
  Wire.setClock(400000UL);

  // reset MPU
  /*0x6B : Clock Sel and power Managment
*  Register : 6B          CLK_SEL [2:0]       Type :  Read/Write
     *
     *  Descrption :
     *    Sets the clock Speed and PLL Referrence.
     *
     *           CLK_SEL    |     Clock Selection
     *          -------------------------------------
     *              0           internal 8 Mhz Oscillator
     *              1           PLL with X Gyro Refference
     *              2           PLL with Y Gyro Refference
     *              3           PLL with Z Gyro Refference
     *              4           PLL with external 32.78 kHz Refference
     *              5           PLL with external 19.2 MHz Refference
     *              6           Reserved
     *              7           Stops and puts to sleep.
     *
     *    SLEEP     [bit 6]    -   set '1' to put MPU 6050 to Sleep.
     *    CYCLE     [bit 5]    -   set '1' and Sleep is disabled.
     *    TEMP_DIS  [bit 3]    -   set '1' to disable Temp. Sensor
     *    DEV_RESET [bit 7]    -   set to '1' to reset the internal Register.

*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  /*
  set gyro full scale : registre 1B
     *              0         +/- 250 degree/s
     *              1         +/- 500 degree/s
     *              2         +/- 1000 degree/s
     *              3         +/- 2000 degree/s
  */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);
  Wire.write(0);
  Wire.endTransmission(true);
  /*
   * Accelerometer config: 0x1C
   *           AFS_SEL    |     Range
   *          -----------------------------
   *              0            +/- 2 g
   *              1            +/- 4 g
   *              2            +/- 8 g
   *              3            +/- 16 g
   */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0);
  Wire.endTransmission(true);
  /*
   *  Register : 19          SMPLTR_DIV [7:0]       Type :  Read/Write
   *
   *  Description :
   *    Sample Rate = (Gyro Output Rate)/(1 + SMPLRT_DIV)
   * */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x19);
  Wire.write(0);
  Wire.endTransmission(true);

  /*
   * filter configuration, DLPF_CFG : 0x26
   *         |    BandWidth  |   Acc (fs)  |  Gyro (fs)
   *    ----------------------------------------------------
   *        0             260 Hz          1 kHz       8 kHz
   *        1             184             1           1
   *        2              94             1           1
   *        3              44             1           1
   *        4              21             1           1
   *        5              10             1           1
   *        6               5             1           1
   */
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x26);
  Wire.write(2);
  Wire.endTransmission(true);

#ifdef SHOW_INIT_MUP6050_TIME
  uint32_t t1 = micros();
  Serial.print(" reset MPU6050 time us =");
  Serial.println(t1 - t0);
#endif
}

/*
REGISTRES DANS MPU6050.h
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
*/

float readGyroXRadSec()
{

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_addr, 2, true);
  int16_T gyroX = Wire.read() << 8 | Wire.read();
  // Serial.print("gyro raw=");
  // Serial.print(gyroX);
  return gyroX * scaleGyro;
}
float readAccZG()
{

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 2, true);
  int16_T accZ = Wire.read() << 8 | Wire.read();
  return accZ * scaleAcc;
}

#endif // USE_MPU_6050

//------------------------------------
// global variables
//------------------------------------
int nbPwm = 0;
void emergencyStop(const char *);

/*-----------------------------------------------------------
Complementary filter part
-----------------------------------------------------------*/
static float cf_tau; // variables globales du filtre complementaire
void initComplementaryFilter(float tauSec, float thb_initRd, float TECH)
{
  // approx de tustin de 1/( 1+tau.p )
}
void oneStepComplementaryFilter(float gyroRds, float accZG, float &thB)
{
  float un = cf_tau * gyroRds + accZG;
  // thB = Un filtree avec le filtre
}
/*-------------------------------------------------------------
notch filter part (filtre anti resonnant )
notch Filter applique sur vthm :
-9db a wn=70rd/s, csi_num=0.2, csi_den=0.6
----------------------------------------------------------------*/
static int16_t typeNotchFilter; // ne pas modifier, sert au dialogue blue tooth
void initNotchFilter(float tauSec, float csin, float csid, float TECH)
{
  typeNotchFilter = NOTCH_FILTER_OFF; // ne pas modifier, sert au dialogue blue tooth
}
void oneStepNotchFilter(float in, float &out)
{
  if (typeNotchFilter == NOTCH_FILTER_OFF)
  {
    out = in;
    return;
  }
  // a modifier pour obtenir out = in filtree avec notch filter
  out = in;
}

/*-------------------------------------------------------------
Derivative filter part
----------------------------------------------------------------*/
static float dv_tau; // variables globales derivateur filtre
void initDerivativeFilter(float tauSec, float TECH)
{
  dv_tau = tauSec;
  // expression du filtre en p : p /( tau . p+1), a traduit en z^-1
}
void oneStepDerivativeFilter(float thm, float &vthm)
{
  vthm = 0;
}

/*-----------------------------------------------------------
motors part
-----------------------------------------------------------*/
static float motorUmaxVolt = 5.0;
static float motorLeftPwmScale = -1 / motorUmaxVolt, motorRightPwmScale = 1.0 / motorUmaxVolt, motorLeftU, motorLeftPwm, motorRightU, motorRightPwm;
static uint8_T motorLeftPwmPin = 34, motoLeftEncoderPinA = 7, motorLeftEncoderPinB = 6, motorLeftId = 0;
static uint8_T motorRightPwmPin = 38, motoRightEncoderPinA = 3, motorRightEncoderPinB = 4, motorRightId = 1;
static float motorPwmPeriodUs = 50.0; // 200 us => 5 Khz to ear motors, choose 50 us => 20Khz to obtain silent work
static float motorEncoderScale = 0.5 * (M_PI * 2.0) / (120.0 * 32.0);
/*-----------------------------------------------------------
tangential control part
-----------------------------------------------------------*/
static float tg_thb, tg_vthb, tg_thm, tg_vthm, tg_Kthb, tg_Kthm, tg_Kvthb, tg_Kvthm, tg_refthm, tg_ref_vthmTe = 0;
static float tg_refu, tg_stepU, tg_UMax ;
static int16_t typeTgControl;
void initTangentialControl(int typeControl, bool firstTime)
{
  switch (typeControl)
  {
  case TANGENTIAL_CONTROL_SLOW:
    /* version 1 => lente*/
    // K=[-24.3471   -0.7092   -4.0633   -1.1249],
    // poles =[-2,-6,-8,-4], fonctionne meme en absence de notch
    tg_Kthb = -25;
    tg_Kthm = -0.8;
    tg_Kvthb = -4.2;
    tg_Kvthm = -1.2;
    typeTgControl = TANGENTIAL_CONTROL_SLOW;
    break;
  case TANGENTIAL_CONTROL_FAST:
    /* version 2 => plus rapide*/
    // K=[-35.3358   -2.8367   -5.9544   -1.9227],
    // poles =[-4+1i*4,-6,-8,-4-1i*4], necessite notch pour fonctionner correctement
    tg_Kthb = -35;
    tg_Kthm = -2.8;
    tg_Kvthb = -5.95;
    tg_Kvthm = -1.92;
    typeTgControl = TANGENTIAL_CONTROL_FAST;
    break;
  case TANGENTIAL_CONTROL_NONE:
    /* version 3 => pas de controle tangentiel*/
    // K=[0 0 0 0],
    tg_Kthb = 0;
    tg_Kthm = 0;
    tg_Kvthb = 0;
    tg_Kvthm = 0;
    tg_refu = 0;
    tg_stepU = 0.5;
    tg_UMax = 4;
    typeTgControl = TANGENTIAL_CONTROL_NONE;
    break;
  }
  if (firstTime)
  {
    tg_refthm = 0;
  }
}
void oneStepTangentialControl(float &u_tg)
{
  if (typeTgControl == TANGENTIAL_CONTROL_NONE)
  {
    u_tg = tg_refu;
    return;
  }
  tg_refthm += tg_ref_vthmTe;
  // signe moins a cause de la contre reaction
  u_tg = -tg_Kthb * tg_thb;
  u_tg -= tg_Kthm * (tg_thm - tg_refthm);
  u_tg -= tg_Kvthb * tg_vthb;
  u_tg -= tg_Kvthm * tg_vthm;
}

/*-----------------------------------------------------------
leftRight control part
tf( [1.36e2,2.36e3],[1,7.86e1])
-----------------------------------------------------------*/
static float lr_refthm, lr_thm = 0, lr_stepRef, lr_refu, lr_stepU, lr_UMax;
static float lr_n0z_1, lr_n1z_1, lr_d1z_1, lr_xn;
static int16_T typeLRControl;
void initLeftRightControl(int typeControl, float Te)
{

  if (typeControl == LR_CONTROL_ON)
  {
    // parametres d'un regulateur a temps continu convenable pour le controle left-right
    // determiner le type de regulateur, et en deduire la pulsation au gain unite probable pour cette loi de commande
    // inventer des experiences permettant de trouver le modèle du systeme et de recalculer ce regulateur
    float n1p = 1.36e2;
    float n0p = 2.36e3;
    float d1p = 1;
    float d0p = 7.86e1;
    lr_refthm = lr_thm;
    typeLRControl = typeControl;
    // initialiser les variables et parametres en z^-1 du controleur left-right
    return;
  }

  typeLRControl = LR_CONTROL_OFF;
  lr_refu = 0;
}
void oneStepLeftRightControl(float &u_lr)
{
  if (typeLRControl == LR_CONTROL_ON)
  {

    u_lr = 0; // a modifier, u_lr = commande = fct(reference : lr_refthm , mesure :lr_thm)
  }
  else
  {
    u_lr = lr_refu;
  }
}

void initMotors()
{

  encoderSetup(motoLeftEncoderPinA, motorLeftEncoderPinB, motorLeftId);
  encoderSetup(motoRightEncoderPinA, motorRightEncoderPinB, motorRightId);
  nbPwm = 0;

  pwmSetup(motorLeftPwmPin, 1, motorPwmPeriodUs, motorPwmPeriodUs / 2, motorLeftId);
  pwmSetup(motorRightPwmPin, 1, motorPwmPeriodUs, motorPwmPeriodUs / 2, motorRightId);
  pwmWrite(0.0, motorLeftId);
  pwmWrite(0.0, motorRightId);
  nbPwm = 2;
}

void applyControl(float u_tg, float u_lr)
{
  motorLeftU = u_tg + u_lr;
  motorRightU = u_tg - u_lr;
  motorLeftU = min(motorLeftU, motorUmaxVolt);
  motorLeftU = max(motorLeftU, -motorUmaxVolt);
  motorRightU = min(motorRightU, motorUmaxVolt);
  motorRightU = max(motorRightU, -motorUmaxVolt);
  motorLeftPwm = motorLeftU * motorLeftPwmScale;
  motorRightPwm = motorRightU * motorRightPwmScale;

  pwmWrite(motorLeftPwm, motorLeftId);
  pwmWrite(motorRightPwm, motorRightId);
}

/*-----------------------------------------------------------
Bluetooth Serial part
-----------------------------------------------------------*/
static float bt_tg_max_speed_rdTe, bt_tg_step_speed_rdTe = 0;
void initBluetooth(float lr_step_rds, float tg_speed_step_rds, float tg_max_speed_rds, float Te)
{
  setupSerialread(3, 9600); // bluetooth adapter connected to serial 3, with speed 9600 bauds

  bt_tg_max_speed_rdTe = Te * tg_max_speed_rds;
  bt_tg_step_speed_rdTe = Te * tg_speed_step_rds;
  lr_stepRef = lr_step_rds; // 5° /s speed for left right ref
  lr_stepU = 0.25;          // 0.25V  for left right U
  lr_UMax = 2;
  // show list of instructions on Serial 3
  Serial3.println("LEFT:B, RIGHT :F");
  Serial3.println("FORWARD:L, BACKWARD:R");
  Serial3.println("STOP:X");
  Serial3.println("TOGGLE NOTCH FILTER:N");
  Serial3.println("TOGGLE TANG CONTROL( SLOW,FAST) :T");
  Serial3.println("TOGGLE LEFT-RIGHT CONTROL(ON,OFF) :V");
}

void oneStepBluetooth()
{
#define BT_NOTHING 83
#define BT_RIGHT 'F'
#define BT_LEFT 'B'
#define BT_BACKWARD 'R'
#define BT_FORWARD 'L'
#define BT_STOP 'X'
#define BT_INV_NOTCH 'N'
#define BT_INV_TG 'T'
#define BT_INV_LR 'V'
  uint8_T v = getSerialread(3, BT_NOTHING);
  switch (v)
  {
  case 255: // nothing to do
  case BT_NOTHING:
    return;
  case BT_INV_LR:
    if (typeLRControl == LR_CONTROL_OFF)
    {
      initLeftRightControl(LR_CONTROL_ON, Tech);
    }
    else
    {
      initLeftRightControl(LR_CONTROL_OFF, Tech);
    }
    return;
  case BT_INV_TG:
    switch (typeTgControl)
    {
    case TANGENTIAL_CONTROL_FAST:
      initTangentialControl(TANGENTIAL_CONTROL_SLOW, false);
    case TANGENTIAL_CONTROL_SLOW:
      initTangentialControl(TANGENTIAL_CONTROL_FAST, false);
    }
    return;
  case BT_INV_NOTCH:
    if (typeNotchFilter == NOTCH_FILTER_ON)
    {
      typeNotchFilter = NOTCH_FILTER_OFF;
    }
    else
    {
      typeNotchFilter = NOTCH_FILTER_ON;
    }
    break;

  case BT_STOP:
    emergencyStop("bluetooth Stop");
    break;
  case BT_RIGHT:
    switch (typeLRControl)
    {
    case LR_CONTROL_ON:
      lr_refthm += lr_stepRef;
      break;
    case LR_CONTROL_OFF:
      lr_refu += lr_stepU;
      if (lr_refu > lr_UMax)
      {
        lr_refu = lr_UMax;
      }
      break;
    }
    break;
  case BT_LEFT:
    switch (typeLRControl)
    {
    case LR_CONTROL_ON:
      lr_refthm -= lr_stepRef;
      break;
    case LR_CONTROL_OFF:
      lr_refu -= lr_stepU;
      if (lr_refu < -lr_UMax)
      {
        lr_refu = -lr_UMax;
      }
      break;
    }
    break;
  case BT_FORWARD:
    if (typeTgControl == TANGENTIAL_CONTROL_NONE)
    {
      tg_refu += tg_stepU;
      if (tg_refu > tg_UMax)
      {
        tg_refu = tg_UMax;
      }
      return;
    }
    tg_ref_vthmTe += bt_tg_step_speed_rdTe;
    if (tg_ref_vthmTe > bt_tg_max_speed_rdTe)
    {
      tg_ref_vthmTe = bt_tg_max_speed_rdTe;
    }
    break;
  case BT_BACKWARD:
    if (typeTgControl == TANGENTIAL_CONTROL_NONE)
    {
      tg_refu -= tg_stepU;
      if (tg_refu <-tg_UMax)
      {
        tg_refu = -tg_UMax;
      }
      return;
    }
    tg_ref_vthmTe -= bt_tg_step_speed_rdTe;
    if (tg_ref_vthmTe < -bt_tg_max_speed_rdTe)
    {
      tg_ref_vthmTe = -bt_tg_max_speed_rdTe;
    }
    break;
  }
  // Serial.println(v);
}
/*---------------------------------------------------------------
WHOLE SYSTEM PART
----------------------------------------------------------------*/

void setup()
{
  Serial.begin(115200);
  float wheel_radius = 0.07, essieu_length = 0.2, rotate_step_rd = 5.0 * M_PI / 180.0, tangent_speed_step_ms = 0.05, tangent_max_speed_ms = 0.5;

  initBluetooth(rotate_step_rd * essieu_length / wheel_radius, tangent_speed_step_ms / wheel_radius, tangent_max_speed_ms / wheel_radius, Tech);
  initComplementaryFilter(2, 0.0, Tech);
  initDerivativeFilter(0.01, Tech);
  initTangentialControl(TANGENTIAL_CONTROL_NONE, true); // a changer avec TANGENTIAL_CONTROL_FAST APRES MISE AU POINT DERIV ET FLT COMPLEMENTAIRE
  initNotchFilter(1 / 70.0, 0.2, 0.6, Tech);            // 70 rd/s notch filter, csi num=0.2,csi den =0.6;
  // initNotchFilter(1 / 70.0, 0.4, 1.2, Tech); // 70 rd/s notch filter, csi num=0.2,csi den =0.6;
  initMotors(); // attention, effet de bord sur nbPwm, et forcement avant initLeftRightControl
  initLeftRightControl(LR_CONTROL_OFF, Tech);

#ifdef USE_MPU_6050
  initMPU6050(); // brute force reinit at each sample step

#endif
  // wait 2 seconds before starting
  Sleep(2000);
// watchdog part at the end of setUP
#ifdef USE_WATCHDOG
  uint8_t watchdogpwmId = 2;
  uint8_t watchdogpwmPin = 36;
  int watchdogpwmTimeUs = (int)(TechUs * 1.2);

  initWatchdog(watchdogpwmPin, watchdogpwmTimeUs, watchdogpwmId);
  nbPwm = 3;
#endif
}
static float THB_MAX_RD = 45.0 * M_PI / 180.0;
void loop()
{
  char msg[100];
  int count = 0;
  float u_tg, u_lr;
  float accZGinit, gyroXinit;
  if (showSerialData)
  {
    Serial.println("\n% tg_thm * 180.0 / M_PI, lr_thm * 180.0 / M_PI, tg_thb * 180.0 / M_PI, gyroX * 180 / M_PI, tg_vthb * 180.0 / M_PI, 100 * typeNotchFilter + 10 * typeTgControl + 1 * typeLRControl");
    Serial.print("data=[");
  }
  while (1)
  {

    uint32_t t_us = micros();
    oneStepBluetooth();
    // estimate vthB,thB
    initMPU6050(); // brute force reinit of mpu, take 400us
    float gyroX = readGyroXRadSec();
    float accZG = readAccZG();
    if (count == 0)
    {
      accZGinit = accZG;
      gyroXinit = gyroX;
    }
    gyroX -= gyroXinit;
    accZG -= accZGinit;
    oneStepNotchFilter(gyroX, tg_vthb);
    oneStepComplementaryFilter(tg_vthb, accZG, tg_thb);

    if (typeTgControl != TANGENTIAL_CONTROL_NONE)
    {
      if (abs(tg_thb) > THB_MAX_RD)
      {
        char msg[100];
        sprintf(msg, "%% too big angle :+%f dg", (float)tg_thb * 180.0 / M_PI);
        emergencyStop(msg);
      }
    }
    // estimate thm,vthm
    int32_T motorLeftEncoder = readEncoder(motorLeftId);
    int32_T motorRightEncoder = readEncoder(motorRightId);
    tg_thm = motorEncoderScale * (motorLeftEncoder + motorRightEncoder); // thm=thmLeft/2+ thmRight/2
    lr_thm = motorEncoderScale * (motorLeftEncoder - motorRightEncoder); // thmLeftRight=thmLeft/2- thmRight/2
    oneStepDerivativeFilter(tg_thm, tg_vthm);                            // estimation de la vitesse du moteur
    // calcule u_tg
    oneStepTangentialControl(u_tg);
    oneStepLeftRightControl(u_lr); // calcul commande left_right = fct lr_refthm,lr_thm (rd)
    applyControl(u_tg, u_lr);
    if (showSerialData)
    {
      // affichage data
      sprintf(msg, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", u_tg, u_lr, tg_thm * 180.0 / M_PI, lr_thm * 180.0 / M_PI, tg_thb * 180.0 / M_PI, gyroX * 180 / M_PI, tg_vthb * 180.0 / M_PI, 100 * typeNotchFilter + 10 * typeTgControl + 1 * typeLRControl);
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
#ifdef USE_WATCHDOG
    watchdogVar = 1;
#endif
    count++;
  }

  emergencyStop("normal end ");

  // unused
}

void emergencyStop(const char *cause)
{

  for (int id = 0; id < nbPwm; id++)
  {
    pwmStop(id);
  }
  Serial3.print("Stop:");
  Serial3.println(cause);

  while (1)

    ; // never go out of this function
}
#endif