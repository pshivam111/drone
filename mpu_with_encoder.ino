//****************************************************//Used for 4.1 and 4.2 //*************************************************
/*
      0.0007912 m per pulse
*/

#define dir1L 23
#define dir2L 25
#define dir1R 27
#define dir2R 29
#define LENC1 4
#define LENC2 3
#define RENC1 2
#define RENC2 14
#define pwmL 5
#define pwmR 6

#define em1 53              //  ELECTROMAGNET
#define em2 51
#define buzz 45
#define b 39
#define G 37
#define C 35
#define R 33

int t, ss, vel;
int s;
double kiT = 0.466  ,  kT = 192.0,    kdT = 0*50 ; //mpu pitch
double kiXL = 0 ,   kXL = 0  ,    kdXL = 0; // encoder left
double kiXR = 0 ,   kXR = 0 ,    kdXR = 0  ; // encoder right
double kiFi = 0.0010*0,   kFi = 2.5,    kdFi = 0; //mpu yaw
double kiU = 0,   kU = 9 ,    kdU = 400.0*0; //mpu tern //
int cm;
double kkL, kkR, preT = 0, preXL, preXR, preFi;
double iT, T, dT;
double iXL, dXL;
double iXR, dXR;
double iFi, Fi, dFi;
volatile int xR, xL;
//..................
int term=0;
int readValue = 0;
int ry_state=0,prev_ry_state=0;
int prev_ry = 0;
int count = 0;
int lx_state=0,prev_lx_state=1;
int lx1 = 0, lx2 = 0, lx = 500;
int ly1 = 0, ly2 = 0, ly = 500;
int rx1 = 0, rx2 = 0, rx = 500;
int ry1 = 0, ry2 = 0, ry = 500;
byte DL = 0;
byte DH = 0;
bool ls = 0;
bool rs = 0;

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw1 = 0, pitch1 = 0, roll1 = 0; //set point
float yaw2 = 0, pitch2 = 0, roll2 = 0; //new reading
char buff[10];
int l=0,m=1023;
#define SW_Reset A0
bool MPU_flag = 0;
float roll = 0, yaw = 0, pitch = 0; //to be sent
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void setup() {
  //############################################################
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //############################################################
  Serial.begin(2000000);
  Serial2.begin(115200);
  mpu.initialize();

  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(SW_Reset, INPUT_PULLUP);
  pinMode(12, OUTPUT);

  pinMode(dir1L, OUTPUT);                   //  Configuring the direction, PWM and electromagnet pins of Arduino Mega as OUTPUT pins
  pinMode(dir2L, OUTPUT);                   //
  pinMode(dir1R, OUTPUT);                   //
  pinMode(dir2R, OUTPUT);                   //
  pinMode(pwmL, OUTPUT);                    //
  pinMode(pwmR, OUTPUT);                    //
  pinMode(LENC1, INPUT_PULLUP);
  pinMode(LENC2, INPUT_PULLUP);
  pinMode(RENC1, INPUT_PULLUP);
  pinMode(RENC2, INPUT_PULLUP);
  pinMode(em1, OUTPUT);              
  pinMode(em2, OUTPUT);
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(C, OUTPUT);
  pinMode(buzz, OUTPUT);

  digitalWrite(C, 0);

  attachInterrupt(digitalPinToInterrupt(LENC2), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(RENC1), ai1, RISING);

  //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(54);
  mpu.setYGyroOffset(91);
  mpu.setZGyroOffset(-7);
  mpu.setZAccelOffset(777); // 1688 factory default for my test chip

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
  }

  pinMode(LED_PIN, OUTPUT);
  //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$



}


char c = 'p', w = 'o', temp = 'o';
const char *msg;


void loop() {
  t = micros();


  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
    if (mpuInterrupt && fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
    if (!digitalRead(SW_Reset) || MPU_flag == 0)
    {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      yaw1 = ypr[0] * 180 / M_PI;
      pitch1 = ypr[1] * 180 / M_PI;
      roll1 = ypr[2] * 180 / M_PI;
      pitch1 = pitch2;
      roll1 = roll2;
      MPU_flag = 1;
      xL = 0;
      xR = 0;
      iT = 0;
      iXL = 0; iXR = 0;
      iFi = 0;
      term=0;

    }
    //154.93

    //    Serial.flush();
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    yaw2 = ypr[0] * 180 / M_PI;
    yaw = yaw2 - yaw1;
    if (yaw == 0.00)
    {
      yaw = yaw + 0.01;
    }
//            Serial.print(yaw);Serial.print("      ");
    dtostrf(yaw, 4, 2, buff);

    pitch2 = ypr[1] * 180 / M_PI;
    pitch = pitch2 - pitch1;
    if (pitch == 0.00)
    {
      pitch = pitch + 0.01;
    }
    dtostrf(pitch, 4, 2, buff);
    //        Serial.print(pitch1);Serial.print("\t");


    roll2 = ypr[2] * 180 / M_PI;
    roll = roll2 - roll1;
    if (roll == 0.00)
    {
      roll = roll + 0.01;
    }
    //          Serial.println(roll);
    dtostrf(roll, 4, 2, buff);

  
    T = roll;
    Fi = yaw;
    ///////////////////////////////////////////////////////////////////

    if (Serial2.available() > 29)
    {
      if (Serial2.read() == 0x7E) {
        for (int i = 1; i <= 18; i++)
        {
          byte disc = Serial2.read();
        }
        int DH = Serial2.read();

        DL = Serial2.read();
        lx1 = Serial2.read();
        lx2 = Serial2.read();
        ly1 = Serial2.read();
        ly2 = Serial2.read();
        rx1 = Serial2.read();
        rx2 = Serial2.read();
        ry1 = Serial2.read();
        ry2 = Serial2.read();
      }

      ry = lx2 + (256 * (lx1 & 3));
      rx = ly2 + (256 * (ly1 & 3));
      ly = rx2 + (256 * (rx1 & 3));
      lx = ry2 + (256 * (ry1 & 3));

      ls = (DL & 16);
      rs = (DH & 4);
}
      if(ry<400) ry_state=-1;
      else if(ry<600) ry_state=0;
      else ry_state=1;
      
      if(lx<400) lx_state=-1;
      else if(lx<600) lx_state=0;
      else lx_state=1;

      
      if((ry_state!=0) || (lx_state!=0 ))
      {
        xL=0;
        xR=0;
      }
    
      if(prev_lx_state==0)
      {
        term -= 30*lx_state;
      }
      

      
      if(prev_ry==ry && (ry>1000 || ry<10)){ count++; }
      else count=0;
      T += ((ry<500)||(ry>520)) *( ((ry-511)/512.0) * (0.6 + (count*0.0004*(count>10))));     //aage pichhe
      
      
      Fi += term;//((lx<500)||(lx>520))*((lx/1020.0)*180 -90);   //daen baen

//  Serial.print(term); 
//  Serial.print("      ");    
//  Serial.print(Fi); 
//  Serial.print("      "); 
   

    ///////////////////////////////////////////////////////////////////////////////
  while(!(Fi<180 && Fi > -180))
  {
      if(Fi>180) Fi-=360;
      if(Fi<-180) Fi+=360;
  }
// Serial.println(Fi); 
    iT += (T);      dT = T - preT;
    iXL += xL;      dXL = xL - preXL;
    iXR += xR;      dXR = xR - preXR;
    iFi += Fi;      dFi = Fi - preFi;

    if (!T) {
      iT = 0;
    }
    if (!xR) {
      iXR = 0;
    }
    if (!xL) {
      iXL = 0;
    }
    if (iT * kiT > 10000)
      s = 10000;
    else
      s = iT * kiT;
    

    cm =  (270 * 4.5 ) / (6 * PI); //distance in cm for bot to oscillate without jerk |-----------B-----------|
    //    Serial.print(T); Ser/ial.println(" ");
kkL = (!lx_state)*((s) + (T * kT) - (dT * kdT) - (kXL * xL) - kiXL * iXL + kdXL * dXL + ((xL > cm) && (T > 0) * 35) - ((xL < -cm) && (T < 0) * 35) + kiFi * iFi + kFi * Fi - kdFi * dFi) + abs(lx_state)*(kiU * iFi + kU * Fi - kdU * dFi);
kkR = (!lx_state)*((s) + (T * kT) - (dT * kdT) - (kXR * xR) - kiXR * iXR + kdXR * dXR + ((xR > cm) && (T > 0) * 35) - ((xR < -cm) && (T < 0) * 35) - kiFi * iFi - kFi * Fi + kdFi * dFi) + abs(lx_state)*(-kiU * iFi - kU * Fi + kdU * dFi);


    preT = T;
    preXL = xL;
    preXR = xR;
    preFi = Fi;
    LMotor(kkL);
    RMotor(kkR);
    prev_ry_state = ry_state;
    prev_lx_state = lx_state;
    prev_ry = ry;

  }


  //##################################################################################################
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
  {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();

  }
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif
  }
  //##################################################################################################
  vel = micros() - t;
  //  Serial.println(vel);
}



void LMotor(int inputpwm)
{
  digitalWrite(dir1L, inputpwm <= 0);
  digitalWrite(dir2L, inputpwm > 0);
  if (abs(inputpwm) > 170)
    inputpwm = 170;

  analogWrite(pwmL, abs(inputpwm));
}
void RMotor(int inputpwm)
{
  digitalWrite(dir1R, inputpwm <= 0);
  digitalWrite(dir2R, inputpwm > 0);
  if (abs(inputpwm) > 170)
    inputpwm = 170;

  analogWrite(pwmR, abs(inputpwm));
}

void ai0()
{
  if (digitalRead(LENC1) == LOW)
    xL++;
  else if (digitalRead(LENC1) == HIGH)
    xL--;
}

void ai1()
{
  if (digitalRead(RENC2) == LOW)
    xR--;
  else if (digitalRead(RENC2) == HIGH)
    xR++;
}

void caliberate(int y)
{
  if ((y > 0) && (y < yaw1))
  {
    yaw = ((y - yaw1) + 360);
  }
  else if ((y >= yaw1) && (y <= 360))
  {
    yaw = (y - yaw1);
  }
}
