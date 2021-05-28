#include "I2Cdev.h"
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if  I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu1(0x68);
MPU6050 mpu2(0x69);

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT1_PIN 2
#define INTERRUPT2_PIN 3

#define LED_PIN 13

#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6

#define ADO_HIGH_PIN 7

bool blinkState = false;

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000; // 60 seconds, 60000 milliseconds

bool dmp1Ready = false;
uint8_t mpu1IntStatus;
uint8_t dev1Status;
uint16_t packet1Size;
uint16_t fifo1Count;
uint8_t fifo1Buffer[64];
int mpu1Counter = 0;

bool dmp2Ready = false;
uint8_t mpu2IntStatus;
uint8_t dev2Status;
uint16_t packet2Size;
uint16_t fifo2Count;
uint8_t fifo2Buffer[64];
int mpu2Counter = 0;

Servo servo1;
Servo servo2;
Servo servo3;

Quaternion q1;
VectorInt16 aa1;
VectorInt16 aa1Real;
VectorInt16 aa1World;
VectorFloat gravity1;
float euler1[3];
float ypr1[3];

Quaternion q2;
VectorInt16 aa2;
VectorInt16 aa2Real;
VectorInt16 aa2World;
VectorFloat gravity2;
float euler2[3];
float ypr2[3];

float servo1init = 85;
float servo2init = 95;
float servo3init = 95;
float ang = 60;
int servoInitCounter = 0;

uint8_t teapot1Packet[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
uint8_t teapot2Packet[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int relay = 9;
bool launched = false;
bool unsafe = false;
bool completed = false;

volatile bool mpu1Interrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmp1DataReady() {
    mpu1Interrupt = true;
}

volatile bool mpu2Interrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmp2DataReady() {
    mpu2Interrupt = true;
}

void mpu1Calibrate() {
  Serial.println(F("Initializing MPU1 DMP..."));
  dev1Status = mpu1.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu1.setXGyroOffset(220);
  mpu1.setYGyroOffset(76);
  mpu1.setZGyroOffset(-85);
  //mpu1.setZAccelOffset(1788); // 1688 factory default for my test chip


  if (dev1Status == 0) {
    Serial.println("HI");
    mpu1.CalibrateAccel(6);
    mpu1.CalibrateGyro(6);
    mpu1.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT1_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT1_PIN), dmp1DataReady, RISING);
    mpu1IntStatus = mpu1.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmp1Ready = true;
    packet1Size = mpu1.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(dev1Status);
      Serial.println(F(")"));
  }
}

void mpu2Calibrate() {
  Serial.println(F("Initializing MPU2 DMP..."));
  dev2Status = mpu2.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu2.setXGyroOffset(220);
  mpu2.setYGyroOffset(76);
  mpu2.setZGyroOffset(-85);
  mpu2.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  if (dev2Status == 0) {
    
    mpu2.CalibrateAccel(6);
    Serial.println("HI");
    mpu2.CalibrateGyro(6);
    
    mpu2.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu2.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT2_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT2_PIN), dmp2DataReady, RISING);
    mpu2IntStatus = mpu2.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmp2Ready = true;
    packet2Size = mpu2.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(dev2Status);
      Serial.println(F(")"));
  }
}

void setup() {
  // put your setup code here, to run once:
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin(); //BUG MAY BE HIRE IF MULTIPLE MPUS
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  
  //digitalWrite(ADO_HIGH_PIN, HIGH);
  
  mpu1.initialize();
  pinMode(INTERRUPT1_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU1 connection successful") : F("MPU1 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  mpu1Calibrate();
  
  /*
  Serial.println(F("Initializing MPU1 DMP..."));
  dev1Status = mpu1.dmpInitialize();

  if (dev1Status == 0) {
    mpu1.CalibrateAccel(6);
    mpu1.CalibrateGyro(6);
    mpu1.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu1.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT1_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT1_PIN), dmp1DataReady, RISING);
    mpu1IntStatus = mpu1.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmp1Ready = true;
    packet1Size = mpu1.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(dev1Status);
      Serial.println(F(")"));
  }
  */
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  pinMode(relay, OUTPUT);
  setServos();
  
  mpu2.initialize();
  pinMode(INTERRUPT2_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu2.testConnection() ? F("MPU2 connection successful") : F("MPU2 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //mpu2Calibrate();
  
  waitForLaunch();
  startMillis = millis();
  Serial.println("START TIME: " + startMillis);
}

void setServos() {
  digitalWrite(relay, HIGH);
  servo1.write(servo1init);
  servo2.write(servo2init);
  servo3.write(servo3init);
  delay(250);
  digitalWrite(relay, LOW);
}

void waitForLaunch() {
  while(!launched) {
    if (mpu1Counter >= 1000) {
      mpu1Calibrate();
      mpu1Counter = 0;
      if (servoInitCounter >= 10) {
        setServos();
        mpu2Calibrate();
        servoInitCounter = 0;
      } else {
        servoInitCounter++;
      }
    }
    if (!dmp1Ready) return;
    while (!mpu1Interrupt && fifo1Count < packet1Size) {
        if (mpu1Interrupt && fifo1Count < packet1Size) {
          fifo1Count = mpu1.getFIFOCount();
        }
    }
    mpu1Interrupt = false;
    mpu1IntStatus = mpu1.getIntStatus();
    fifo1Count = mpu1.getFIFOCount();
    if(fifo1Count < packet1Size){
            //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
        // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    else if ((mpu1IntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifo1Count >= 1024) {
      // reset so we can continue cleanly
      mpu1.resetFIFO();
      Serial.println(F("FIFO overflow!"));
    } else if (mpu1IntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
        while(fifo1Count >= packet1Size) {
          mpu1.getFIFOBytes(fifo1Buffer, packet1Size);
          fifo1Count -= packet1Size;
        }
        mpu1.dmpGetQuaternion(&q1, fifo1Buffer);
        mpu1.dmpGetGravity(&gravity1, &q1);
        mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
        Serial.print("ypr\t");
        Serial.print(ypr1[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr1[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr1[2] * 180/M_PI);
        Serial.print("\t");
        Serial.print(mpu1Counter);
        Serial.print("\t");
        Serial.println(servoInitCounter);
        
        mpu1.dmpGetQuaternion(&q1, fifo1Buffer);
        mpu1.dmpGetAccel(&aa1, fifo1Buffer);
        mpu1.dmpGetGravity(&gravity1, &q1);
        mpu1.dmpGetLinearAccel(&aa1Real, &aa1, &gravity1);
        if (abs(aa1Real.z) > 8000) { // Gravity acceleration ~ 8192
          Serial.print(aa1Real.z);
          if (abs(aa1Real.z) > 15000) {
            Serial.println("LAUNCHED!");
            launched = true;
            digitalWrite(relay, HIGH);
            return;
          } else {
            Serial.println("not launched");
          }
        }
    }
    mpu1Counter++;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  currentMillis = millis();
  Serial.println("TIME SINCE LAUNCH: " + (currentMillis - startMillis));
  */
  // if programming failed, don't try to do anything
  
  if (!dmp1Ready) return;
  //if (!dmp2Ready) return;
  while (!mpu1Interrupt && fifo1Count < packet1Size) {
      if (mpu1Interrupt && fifo1Count < packet1Size) {
        fifo1Count = mpu1.getFIFOCount();
      }
  }
  mpu1Interrupt = false;
  mpu1IntStatus = mpu1.getIntStatus();
  fifo1Count = mpu1.getFIFOCount();
  if(fifo1Count < packet1Size){
          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  }
  else if ((mpu1IntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifo1Count >= 1024) {
    // reset so we can continue cleanly
    mpu1.resetFIFO();
    //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
    Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpu1IntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
      while(fifo1Count >= packet1Size) { // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu1.getFIFOBytes(fifo1Buffer, packet1Size);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifo1Count -= packet1Size;
      }
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu1.dmpGetQuaternion(&q1, fifo1Buffer);
          mpu1.dmpGetGravity(&gravity1, &q1);
          mpu1.dmpGetYawPitchRoll(ypr1, &q1, &gravity1);
          //Serial.print("ypr\t");
          //Serial.print(ypr[0] * 180/M_PI);
          servo1.write(servo1init+ypr1[0]*180/M_PI);
          //Serial.print("\t");
          //Serial.print(ypr[1] * 180/M_PI);
          servo2.write(servo2init-ypr1[1]*180/M_PI);
          //Serial.print("\t");
          //Serial.println(ypr1[2] * 180/M_PI);
          servo3.write(servo3init-ypr1[2]*180/M_PI);
      #endif
     
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
}
