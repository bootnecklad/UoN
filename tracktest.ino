#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, 12, 13, A2, A1, A0);
SoftwareSerial carController(8, 9); // RX, TX


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

const int YAW = 0;
const int PITCH = 1;
const int ROLL = 2;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void move_forward(int motorSpeed)
{
  carController.write("#Bbff0");
  carController.print(lowByte(motorSpeed));
  carController.write(",0");
  carController.print(lowByte(motorSpeed));
}

void move_reverse(int motorSpeed)
{
  carController.write("#Bbrr0");
  carController.print(lowByte(motorSpeed));
  carController.write(",0");
  carController.print(lowByte(motorSpeed));
}

void rotate_left(int motorSpeed)
{
  carController.write("#Bbfr0");
  carController.print(lowByte(motorSpeed));
  carController.write(",0");
  carController.print(lowByte(motorSpeed));
}

void rotate_right(int motorSpeed)
{
  carController.write("#Bbrf0");
  carController.print(lowByte(motorSpeed));
  carController.write(",0");
  carController.print(lowByte(motorSpeed));
}

void halt_motors()
{
  carController.write("#Hb");
}

void setup() {

  lcd.begin(16, 2);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    Serial.begin(57600);
    carController.begin(57600);
    while (!Serial);
    while(!carController);

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-46);
    mpu.setYGyroOffset(-25);
    mpu.setZGyroOffset(25);
    mpu.setXAccelOffset(1640);
    mpu.setYAccelOffset(-108);
    mpu.setZAccelOffset(880); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void read_sensor_values()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
       
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
//        #ifdef OUTPUT_READABLE_YAWPITCHROLL
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
//        #endif

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    }
}

void stop_at_angle(int principalAxisAngle, float initialAngle, int stopAngle)
{
  float currentAngle;

  currentAngle = ypr[principalAxisAngle] * 180/M_PI;
  
  while(abs((int)(currentAngle - initialAngle)) < (initialAngle+stopAngle))
  {
      read_sensor_values();
      currentAngle = ypr[principalAxisAngle] * 180/M_PI;
      lcd.setCursor(0,0);
      lcd.print(initialAngle);
      lcd.setCursor(0,1);
      lcd.print(currentAngle);
      Serial.println(initialAngle);
      Serial.print(" ");
      Serial.println(currentAngle);
      Serial.print("\n");
  }

  halt_motors();
}


void findState(float refAngleYAW ,float refAnglePITCH,float refAngleROLL, int *state, int angleAccuracy)
{
  read_sensor_values();
  float currentAnglePITCH = ypr[PITCH] * 180/M_PI;
  float currentAngleYAW = ypr[YAW] * 180/M_PI;
  float currentAngleROLL = ypr[ROLL] * 180/M_PI;
  if (abs((int)currentAnglePITCH - refAnglePITCH) < angleAccuracy) 
  {
   //Serial.print("flat");
   read_sensor_values();
   currentAnglePITCH = ypr[PITCH] * 180/M_PI;
   state[0] = 0;
  }
  
  else if (((int)currentAnglePITCH - refAnglePITCH) > angleAccuracy) 
  {
   //Serial.print("going up");
   read_sensor_values();
   currentAnglePITCH = ypr[PITCH] * 180/M_PI;
   state[0] = 1;
  }
    else if (((int)currentAnglePITCH - refAnglePITCH) < -angleAccuracy) 
  {
   //Serial.print("going down");
   read_sensor_values();
   currentAnglePITCH = ypr[PITCH] * 180/M_PI;
   state[0] = 2;
  }
//  if (abs((int)currentAngleROLL - refAngleROLL) < angleAccuracy)
//  {
//   //Serial.print("Straight");
//   read_sensor_values();
//   currentAngleROLL = ypr[ROLL] * 180/M_PI;
//   state[1] = 0;
//  }
//   else if ((int)(currentAngleROLL - refAngleROLL) > angleAccuracy)
//  {
//   //Serial.print("RIGHT");
//   read_sensor_values();
//   currentAngleROLL = ypr[ROLL] * 180/M_PI;
//   state[1] = 1;
//  }
//   else if (((int)currentAngleROLL - refAngleROLL) < -angleAccuracy)
//  {
//   //Serial.print("LEFT");
//   read_sensor_values();
//   currentAngleROLL = ypr[ROLL] * 180/M_PI;
//   state[1] = 2;
//  }

if (abs((int)currentAngleYAW - refAngleYAW) < angleAccuracy)
  {
   //Serial.print("Straight");
   read_sensor_values();
   currentAngleYAW = ypr[YAW] * 180/M_PI;
   state[1] = 0;
  }
   else if ((int)(currentAngleYAW - refAngleYAW) > angleAccuracy)
  {
   //Serial.print("RIGHT");
   read_sensor_values();
   currentAngleYAW = ypr[YAW] * 180/M_PI;
   state[1] = 1;
  }
   else if (((int)currentAngleYAW - refAngleYAW) < -angleAccuracy)
  {
   //Serial.print("LEFT");
   read_sensor_values();
   currentAngleYAW = ypr[YAW] * 180/M_PI;
   state[1] = 2;
  }
  
}

void loop()
{
  float referenceAngles[3];

  int delay_loop = 1;
  
  while(delay_loop)
  {
    read_sensor_values();
    delay(5000);
    referenceAngles[YAW] = ypr[YAW] * 180/M_PI;
    referenceAngles[PITCH] = ypr[PITCH] * 180/M_PI;
    referenceAngles[ROLL] = ypr[ROLL] * 180/M_PI;
    delay_loop = 0;
  }

  read_sensor_values();
  

  
//  rotate_left(15);
//  stop_at_angle(PITCH, referenceAngles[PITCH], 20);
//  delay(2000);
//  
//  read_sensor_values();
//  rotate_right(15);
//  stop_at_angle(YAW, referenceAngles[YAW], 40);
//  delay(2000);
//  
//  read_sensor_values();
//  move_forward(15);
//  stop_at_angle(ROLL, referenceAngles[ROLL], 20);
  int state [2];
  int *statePointer;
  statePointer = state;
  while(1)
  {
    findState(referenceAngles[YAW],referenceAngles[PITCH],referenceAngles[ROLL],statePointer, 5);
    lcd.setCursor(0,0);
    lcd.clear();
    switch (state[0])
    {
    case 0:

       lcd.print("Flat");
        break;
      case 1:
        lcd.print("UP");
        break;
      case 2:
        lcd.print("DOWN");
        break;
   }
   lcd.setCursor(0,1);
    switch (state[1])
    {
    case 0:
       lcd.print("Straight");
        break;
      case 1:
        lcd.print("Right");
        break;
      case 2:
        lcd.print("Left");
        break;
   }
  }
//  turn360
//  reverse
//  detectslope
//  detectendofslope
//  stop
 
    
}
