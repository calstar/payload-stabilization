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
#define LED_PIN 13
#define SERVO1_PIN 4
#define SERVO2_PIN 5
#define SERVO3_PIN 6

bool blinkState = false;

bool dmp1Ready = false;
uint8_t mpu1IntStatus;
uint8_t dev1Status;
uint16_t packet1Size;
uint16_t fifo1Count;
uint8_t fifo1Buffer[64];

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
float servo1init = 85;
float servo2init = 95;
float servo3init = 95;
float ang = 60;

uint8_t teapot1Packet[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int relay = 9;
bool launched = false;
bool unsafe = false;
bool completed = false;

volatile bool mpu1Interrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmp1DataReady() {
    mpu1Interrupt = true;
}




void setup() {
  // put your setup code here, to run once:
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  mpu1.initialize();
  pinMode(INTERRUPT1_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu1.testConnection() ? F("MPU1 connection successful") : F("MPU1 connection failed"));

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));

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
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo1.write(servo1init);
  servo2.write(servo2init);
  servo3.write(servo3init);
  delay(100);
  //digitalWrite(relay, LOW);
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // if programming failed, don't try to do anything
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

      #ifdef OUTPUT_READABLE_REALACCEL
          // display real acceleration, adjusted to remove gravity
          mpu1.dmpGetQuaternion(&q1, fifo1Buffer);
          mpu1.dmpGetAccel(&aa1, fifo1Buffer);
          mpu1.dmpGetGravity(&gravity1, &q1);
          mpu1.dmpGetLinearAccel(&aa1Real, &aa1, &gravity1);
          /*
          Serial.print("areal\t");
          
          Serial.print(aa1Real.x);
          Serial.print("\t");
          Serial.print(aa1Real.y);
          Serial.print("\t");
          Serial.print(aa1Real.z);
          Serial.print("\t");
          //Serial.println(aa1Real.z);
          Serial.print(aa1Real.x*aa1Real.x);
          Serial.print("\t");
          Serial.print(aa1Real.y*aa1Real.y);
          Serial.print("\t");
          Serial.print(aa1Real.z*aa1Real.z);
          Serial.print("\t");
          Serial.println((aa1Real.x*aa1Real.x)+(aa1Real.y*aa1Real.y)+(aa1Real.z*aa1Real.z));
          Serial.println(sqrt((aa1Real.x*aa1Real.x)+(aa1Real.y*aa1Real.y)+(aa1Real.z*aa1Real.z)));
          */
          if (abs(aa1Real.z) > 8000) {
            Serial.println(aa1Real.z);
          }
          
      #endif
      /*
      #ifdef OUTPUT_READABLE_WORLDACCEL
          // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          mpu1.dmpGetQuaternion(&q1, fifo1Buffer);
          mpu1.dmpGetAccel(&aa1, fifo1Buffer);
          mpu1.dmpGetGravity(&gravity1, &q1);
          mpu1.dmpGetLinearAccel(&aa1Real, &aa1, &gravity1);
          mpu1.dmpGetLinearAccelInWorld(&aa1World, &aa1Real, &q1);
          Serial.print("aworld\t");
          Serial.print(aa1World.x);
          Serial.print("\t");
          Serial.print(aa1World.y);
          Serial.print("\t");
          Serial.println(aa1World.z);
      #endif
      */
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
}
