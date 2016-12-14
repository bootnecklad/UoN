#include <Keypad.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <string.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define OUTPUT_READABLE_YAWPITCHROLL

// Defines numbers of rows & columns on keypad
const byte ROWS = 4;
const byte COLUMNS = 3;

// defines interrupt pin for MPU
const int INTERRUPT_PIN = 2;

// Defines keymap of keypad
char keys[ROWS][COLUMNS] = {
  {'#','*','0'},
  {'1','3','2'},
  {'4','6','5'},
  {'7','9','8'}
};

// keypad pins columns
byte rowPins[ROWS] = { A3, 3, 4, 5 };
// keypad pins columns
byte columnPins[COLUMNS] = { 6, 7, 8 }; 

// setups liquid crystal display pins
LiquidCrystal lcd(11, 12, 13, A2, A1, A0);

// setups serial port for car controller board
SoftwareSerial carController(8, 9);

// creates keypad
Keypad kpd = Keypad( makeKeymap(keys), rowPins, columnPins, ROWS, COLUMNS );

char PIN[6]={'1','2','3','4','5','6'}; // SUPER SECRET PIN NUMBER
char COMMAND_INIT_ANGLE[6] = { '0', '1','0','0','0','0'}; // initialise YAW PITCH & ROLL angles
char DISPLAY_ORIENTATION[6] = { '0', '2','0','0','0','0'}; // display orientation
char DISPLAY_YPR[6] = { '0', '3', '0','0','0','0'}; // display YAW, PITCH & ROLL angles
char HALT_MOTORS[6] = { '0', '4', '0','0','0','0'}; // halt motors if running
char RUN_MOTORS_FORWARD[6] = { '0', '5', '0','0','0','0'}; // run motors
char RUN_MOTORS_REVERSE[6] = { '0', '6', '0','0','0','0'}; // run motors
char ROTATE_MOTORS_LEFT[6] = { '0', '7', '0','0','0','0'}; // run motors
char ROTATE_MOTORS_RIGHT[6] = { '0', '8', '0','0','0','0'}; // run motors
char PERFORM_TEST[6] = { '0', '9', '0','0','0','0'};


char characterArray[6] = {'0','0','0','0','0','0'}; // storage array for keypad input

// access arrays of angles with YAW, PITCH & ROLL. NOT 0, 1, .. etc
const int YAW = 0;
const int PITCH = 1;
const int ROLL = 2;

// defined for orientation code readibility
const int YAW_STRAIGHT = 0;
const int YAW_RIGHT = 1;
const int YAW_LEFT = 2;
const int PITCH_FLAT = 0;
const int PITCH_UP = 1;
const int PITCH_DOWN = 2;
const int ROLL_HORIZONTAL = 0;
const int ROLL_BANK_RIGHT = 1;
const int ROLL_BANK_LEFT = 2;

// set during initialisation command
float referenceAngles[3];

// updated everytime updateYPR() called
float currentAngles[3];
float baseAngles[3];

//
MPU6050 mpu;

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

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {

  lcd.begin(16, 2);
  lcd.print("PING");
  delay(1000);
  lcd.clear();
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

    // gyro offsets
    mpu.setXGyroOffset(-46);
    mpu.setYGyroOffset(-25);
    mpu.setZGyroOffset(25);
    mpu.setXAccelOffset(1640);
    mpu.setYAccelOffset(-108);
    mpu.setZAccelOffset(880);

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
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees            
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    #endif
    }
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

void stop_at_angle(int principalAxis, int stopAngle)
{
  while((baseAngles[principalAxis] < stopAngle) && (baseAngles[principalAxis] > -stopAngle))
  {
    updateYPR();
    lcd.clear();
    lcd.print(baseAngles[principalAxis]);
    delay(100);
  }
  
  halt_motors();
}

void findOrientation(int *state, int angleAccuracy)
{
  updateYPR();

  if((baseAngles[PITCH] <= angleAccuracy) && (baseAngles[PITCH] >= -angleAccuracy))
    state[PITCH] = PITCH_FLAT;
  
  if(baseAngles[PITCH] > angleAccuracy)
    state[PITCH] = PITCH_UP;
  
  if(baseAngles[PITCH] < -angleAccuracy)
    state[PITCH] = PITCH_DOWN;

  if((baseAngles[YAW] <= angleAccuracy) && (baseAngles[YAW] >= -angleAccuracy))
    state[YAW] = YAW_STRAIGHT;
  
  if(baseAngles[YAW] > angleAccuracy)
    state[YAW] = YAW_RIGHT;
  
  if(baseAngles[YAW] < -angleAccuracy)
    state[YAW] = YAW_LEFT;

  if((baseAngles[ROLL] <= angleAccuracy) && (baseAngles[ROLL] >= -angleAccuracy))
    state[ROLL] = ROLL_HORIZONTAL;
  
  if(baseAngles[ROLL] > angleAccuracy)
    state[ROLL] = ROLL_BANK_RIGHT;
  
  if(baseAngles[ROLL] < -angleAccuracy)
    state[ROLL] = ROLL_BANK_LEFT;
}

void readKeypad()
{
  int readingInput, cleared, LCDplace, characterCount;

  characterCount = 0;
  LCDplace = 0;
  cleared = 0;
  readingInput = 1;
  
  while(readingInput)
  {
    
    char key = kpd.getKey();

    if (key != NO_KEY)
    {
      
      if(key == '*')
      {
        delay(100); // for extra debounce
        lcd.setCursor(LCDplace, 1);
        lcd.print(" ");
        
        LCDplace--;
        characterCount--;

        if((LCDplace < 0) || (characterCount < 0))
        {
          LCDplace = 0;
          characterCount = 0;
        }
        cleared=1; 
      }

      if(key == '#')
      {
        characterCount=0;
        readingInput = 0;
        delay(100); // for extra debounce
        lcd.clear();
        break;
      }
      
      if((characterCount<6)&&(cleared==0))
      {
        characterArray[characterCount]=key;
        LCDplace = characterCount;
        characterCount++;
        lcd.setCursor(LCDplace, 1);
        lcd.print(key);
        lcd.setCursor(0,0);
        lcd.print(characterCount);
      }
      else
      {
        cleared=0;
      }
      
      if(characterCount==7)
      {
        readingInput=0;
        lcd.clear();
      }
    }
  }
}

void checkPIN()
{  
  if(memcmp(characterArray, PIN, sizeof(characterArray)) == 0)
  {
    correctPIN();
  }
  else
  {
    incorrectPIN();
  }

  clearCharacterArray();
}

void clearCharacterArray()
{
  int characterCount;
  
  for (characterCount = 0; characterCount < 6; characterCount++) 
  {
    characterArray[characterCount]='0';
  }
}

void correctPIN() // do this if correct PIN entered
{
  lcd.print("* Correct PIN *");
  delay(1000);
  lcd.clear();
}
 
void incorrectPIN() // do this if incorrect PIN entered
{
  lcd.print(" * Try again *");
  delay(1000);
  lcd.clear();
}

int findCommand(int commandArray)
{
  int command = 0;

  char temporaryArray[6] = { '0', '0', '0','0','0','0'};

  temporaryArray[0] = characterArray[0];
  temporaryArray[1] = characterArray[1];
  
  if(memcmp(COMMAND_INIT_ANGLE, temporaryArray, sizeof(COMMAND_INIT_ANGLE)) == 0)
    command = 1;
  
  if(memcmp(DISPLAY_YPR, temporaryArray, sizeof(DISPLAY_YPR)) == 0)
    command = 2;
  
  if(memcmp(DISPLAY_ORIENTATION, temporaryArray, sizeof(DISPLAY_ORIENTATION)) == 0)
    command = 3;

  if(memcmp(HALT_MOTORS, temporaryArray, sizeof(HALT_MOTORS)) == 0)
    command = 4;

  if(memcmp(RUN_MOTORS_FORWARD, temporaryArray, sizeof(RUN_MOTORS_FORWARD)) == 0)
    command = 5;

  if(memcmp(RUN_MOTORS_REVERSE, temporaryArray, sizeof(RUN_MOTORS_REVERSE)) == 0)
    command = 6;

  if(memcmp(ROTATE_MOTORS_LEFT, temporaryArray, sizeof(ROTATE_MOTORS_LEFT)) == 0)
    command = 7;

  if(memcmp(ROTATE_MOTORS_RIGHT, temporaryArray, sizeof(ROTATE_MOTORS_RIGHT)) == 0)
    command = 8;

  if(memcmp(PERFORM_TEST, temporaryArray, sizeof(PERFORM_TEST)) == 0)
    command = 9;

  return command;
}

void setReferenceAngles()
{
  read_sensor_values();

  referenceAngles[PITCH] = ypr[PITCH] * 180/M_PI;
  referenceAngles[YAW] = ypr[YAW] * 180/M_PI;
  referenceAngles[ROLL] = ypr[ROLL] * 180/M_PI;
}

int display_YPR = 0;

void displayYPR()
{
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("YAW");
  lcd.setCursor(5,0);
  lcd.print("PITCH");
  lcd.setCursor(12,0);
  lcd.print("ROLL");

  lcd.setCursor(0,1);
  lcd.print(baseAngles[YAW]);
  lcd.setCursor(5,1);
  lcd.print(baseAngles[PITCH]);
  lcd.setCursor(12,1);
  lcd.print(baseAngles[ROLL]);
}

int display_ORIENTATION = 0;

void displayOrientation()
{
  int state[3];
  int *statePointer;
  statePointer = state;
  
  findOrientation(statePointer, 10);

  lcd.setCursor(0,0);
  lcd.clear();
  
  switch (state[PITCH])
  {
    case 0:
      lcd.print("F ");
      lcd.print(baseAngles[PITCH]);
      break;
    case 1:
      lcd.print("U ");
      lcd.print(baseAngles[PITCH]);
      break;
    case 2:
      lcd.print("D ");
      lcd.print(baseAngles[PITCH]);
      break;
  }
   
  lcd.setCursor(0,1);
   
  switch (state[YAW])
  {
    case 0:
      lcd.print("S ");
      lcd.print(baseAngles[YAW]);
      break;
    case 1:
      lcd.print("R ");
      lcd.print(baseAngles[YAW]);
      break;
    case 2:
      lcd.print("L ");
      lcd.print(baseAngles[YAW]);
     break;
  }

  lcd.setCursor(8,1);
  
  switch (state[ROLL])
  {
    case 0:
      lcd.print("H ");
      lcd.print(baseAngles[ROLL]);
      break;
    case 1:
      lcd.print("BR ");
      lcd.print(baseAngles[ROLL]);
      break;
    case 2:
      lcd.print("BL ");
      lcd.print(baseAngles[ROLL]);
      break;
   }
}

void updateYPR()
{
  read_sensor_values();
  
  currentAngles[PITCH] = ypr[PITCH] * 180/M_PI;
  currentAngles[YAW] = ypr[YAW] * 180/M_PI;
  currentAngles[ROLL] = ypr[ROLL] * 180/M_PI;
  
  baseAngles[PITCH] = currentAngles[PITCH] - referenceAngles[PITCH];
  baseAngles[YAW] =  currentAngles[YAW] - referenceAngles[YAW];
  baseAngles[ROLL] =  currentAngles[ROLL] - referenceAngles[ROLL];
}

void readAndSetCommands()
{
  int command;
  
  char key = kpd.getKey();
  
  if (key == '#')
  {
    lcd.clear();
    lcd.print("COMMAND: ");
    
    readKeypad();
    command = findCommand(characterArray);
    
    lcd.clear();

    char charCommandArgument[3];
    int commandArgument;
    charCommandArgument[0] = characterArray[2];
    charCommandArgument[1] = characterArray[3];
    charCommandArgument[2] = "\n";
    commandArgument = atoi(charCommandArgument);

    switch(command)
    {
      case 0:
        lcd.print("NOPE!");
        delay(1000);
        lcd.clear();
        break;
      case 1:
        setReferenceAngles();
        break;
      case 2:
        display_YPR = !display_YPR;
        break;
      case 3:
        display_ORIENTATION = !display_ORIENTATION;
        break;
      case 4:
        halt_motors();
        break;
      case 5:
        move_forward(commandArgument);
        break;
      case 6:
        move_reverse(commandArgument);
        break;
      case 7:
        rotate_left(commandArgument);
        break;
      case 8:
        rotate_right(commandArgument);
      case 9:
        stop_at_angle(commandArgument, 5);
      default:
        break;
    }
  }
}

void executeCommands()
{
  if(display_YPR)
    displayYPR();

  if(display_ORIENTATION)
    displayOrientation();
}

void loop()
{
  updateYPR();
    
  readAndSetCommands();

  executeCommands();

  


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

//  turn360
//  reverse
//  detectslope
//  detectendofslope
//  stop
}
