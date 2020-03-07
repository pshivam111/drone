

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
            Serial.print(yaw);Serial.print("      ");
    dtostrf(yaw, 4, 2, buff);

    pitch2 = ypr[1] * 180 / M_PI;
    pitch = pitch2 - pitch1;
    if (pitch == 0.00)
    {
      pitch = pitch + 0.01;
    }
    dtostrf(pitch, 4, 2, buff);
            Serial.print(pitch1);Serial.print("\t");


    roll2 = ypr[2] * 180 / M_PI;
    roll = roll2 - roll1;
    if (roll == 0.00)
    {
      roll = roll + 0.01;
    }
              Serial.println(roll);
    dtostrf(roll, 4, 2, buff);

  
    T = roll;
    Fi = yaw;
    ///////////////////////////////////////////////////////////////////

 

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
