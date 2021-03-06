// This program is part of a group project at UoN.
// penryu contributed to clean up and reformatting!
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA


const byte SlaveDeviceId = 1;

char emptyBuffer[16] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
char printHello[16] = {'H', 'E', 'L', 'L', 'O', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};

char lightON[16] = {'H', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
char lightOFF[16] = {'L', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};

// Global Constants
const int BUFFER_VALUE = 2000;        // Used to constrain the error value
const int LCD_ROW = 2;
const int LCD_COLUMN = 16;
const int LOOP_DELAY = 0;             // Not in use, used for debugging

const int TOO_FAST = 7;         // defines various error codes

// Global Variables
// Could be improve by creating a structure that represents the system as a whole?

// Values used to calculate sensor of position on car
float sensorPosition;           // Current position of sensor
float sensorPositionInitial;    // Initial position of sensor when calibration after button press
float positionDeviationSum;     // Continuous sum of how far the sensor has deviated over time
float positionDeviationOld;     // Previous value of how far the sensor has deviated over time
float errorInPosition;          // Total error of sensor compared to the ideal/initial position

float straightMotorSpeed = 0;       //Used to smoothly accelerate the car when going straight
float calibrationConstantP = 2;       // The 3 constants to be changed in PID control
float calibrationConstantI = 0.005;   // These will likely be different for every course and every vehicle
float calibrationConstantD = 1;
int MAX_MOTOR_SPEED = 40;
int MIN_MOTOR_SPEED = -40;
int leftMotorBaseSpeed = 15;
int rightMotorBaseSpeed = leftMotorBaseSpeed;

// Sets up serial port for car controller board
#include <SoftwareSerial.h>
SoftwareSerial carController(9, 10);

// LINE SENSOR SETUP
#include <Wire.h>
unsigned char t;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
unsigned char data[16];


void setup()
{
  Serial.begin(9600);
  carController.begin(57600);
  while (!Serial);
  while (!carController);

  Wire.begin();
  t = 0;

  // Sets initial state of button on car
  int goButton = 0;

  pinMode(2, INPUT);

  // Pauses car until button is pressed
  while (!goButton)
  {
    goButton = digitalRead(2);
  }
  delay(1000);
  initialisePIDcontrol(); // Initialises values for PID control
}

int HALT = 0;

void loop()
// INTERNAL FUNCTION: loop()
// ARGUMENTS:
// ARGUMENT TYPES:
// DESCRIPTION: Arduino loop that runs and sends commands to the firmware board
{
  int goButton;

  goButton = digitalRead(2);

  while(goButton)
  {
    goButton = digitalRead(2);
    carController.write("#Hb");
    initialisePIDcontrol();
  }

  if(!HALT)
  {

  fetchSensorValues();

  sensorPosition = calculatePosition();              // Fetches current position of sensor

  errorInPosition = calculateError(sensorPosition);  // Calculates the error in the position

  errorInPosition = bufferError(errorInPosition);    // Buffers error as to not cause rapid movement

  moveMotors(errorInPosition);                       // Moves motors based on the error from initial position and the speed of the motor
  }

  if(HALT)
  {
    carController.write("#Hb");
  }

  sendData();
  getCommands();
}

void sendData()
{
  long unsigned int distanceLeft = 0;
  long unsigned int distanceRight = 0;
  int ConstantP;       // The 3 constants to be changed in PID control
  int ConstantI;   // These will likely be different for every course and every vehicle
  int ConstantD;

  distanceRight = readCounter(1);
  distanceLeft = readCounter(2);

  ConstantP = calibrationConstantP*1000;
  ConstantI = calibrationConstantI*1000;
  ConstantD = calibrationConstantD*1000;
  
  data[0] = (byte) distanceRight;
  data[1] = (byte) (distanceRight >> 8);

  data[2] = (byte) distanceLeft;
  data[3] = (byte) (distanceLeft >> 8);

  data[4] = (byte) ConstantP;
  data[5] = (byte) (ConstantP >> 8);

  data[6] = (byte) ConstantI;
  data[7] = (byte) (ConstantI >> 8);

  data[8] = (byte) ConstantD;
  data[9] = (byte) (ConstantD >> 8);

  Wire.beginTransmission(SlaveDeviceId);
  Wire.write(data, 16);
  Wire.endTransmission();
}

void getCommands()
{
  int bytesRecieved;

  Wire.beginTransmission(SlaveDeviceId);

  bytesRecieved = Wire.requestFrom(SlaveDeviceId, (uint8_t)16);

  if (Wire.available())
  {
    Wire.readBytes(data, bytesRecieved);
  }

  Wire.endTransmission();

  for (t = bytesRecieved; t < 16; t++)
  {
    data[t] = '0';
  }

  char charCommandArgument[5];
  int commandArgument;
  charCommandArgument[0] = data[1];
  charCommandArgument[1] = data[2];
  charCommandArgument[2] = data[3];
  charCommandArgument[3] = data[4];
  charCommandArgument[4] = "\n";
  commandArgument = atoi(charCommandArgument);

  if(data[0] == 'H')
  {
    digitalWrite(13, HIGH);
  }

  if(data[0] == 'L')
  {
    digitalWrite(13, LOW);
  }

  if(data[0] == 'P')
  {
    calibrationConstantP = (float)commandArgument/1000;
  }

  if(data[0] == 'I')
  {
    calibrationConstantI = (float)commandArgument/1000;
  }

  if(data[0] == 'D')
  {
    calibrationConstantD = (float)commandArgument/1000;
  }

  if(data[0] == 'M')
  {
    leftMotorBaseSpeed = (float)commandArgument;
  }

  if(data[0] == 'S')
  {
    HALT = 1;
  }

  if(data[0] == 'G')
  {
    HALT = 0;
  }

  if(data[0] == 'R')
  {
    straightMotorSpeed = 0;       //Used to smoothly accelerate the car when going straight
    calibrationConstantP = 2;       // The 3 constants to be changed in PID control
    calibrationConstantI = 0.005;   // These will likely be different for every course and every vehicle
    calibrationConstantD = 1;
    MAX_MOTOR_SPEED = 40;
    MIN_MOTOR_SPEED = -40;
    leftMotorBaseSpeed = 15;
    rightMotorBaseSpeed = leftMotorBaseSpeed;
  }
}


long unsigned int readCounter(int motor)
{
  long unsigned int count = 0;

  if(motor == 1)
  {
    carController.write("#e1");
  }

  if(motor == 2)
  {
    carController.write("#e2");
  }

  delay(20);

  count = carController.read();

  count += (carController.read() << 8);
  count += (carController.read() << 16);
  count += (carController.read() << 24);

  return count;
}

unsigned int sensorData[8];

void fetchSensorValues()
{
  Wire.requestFrom(9, 16);   // request 16 bytes from slave device #9
  while (Wire.available())   // slave may send less than requested
  {
    data[t] = Wire.read();   // receive a byte as character
    if (t < 15)
      t++;
    else
      t = 0;
  }

  int n;

  for (n = 0; n < 8; n++)
  {
    sensorData[n] = data[n * 2] << 2;   // Shift the 8 MSBs up two places and store in array
    sensorData[n] += data[(n * 2) + 1]; // Add the remaining bottom 2 LSBs
  }
}

void initialisePIDcontrol()
{
  // Initial values for deviation from line at calibration is zero
  // Car is on the line so no deviation
  positionDeviationSum = 0;
  positionDeviationOld = 0;

  fetchSensorValues();

  sensorPositionInitial = calculatePosition(); // Fetches position to calibrate car on line.
}

void moveMotors(float errorInPosition)
// INTERNAL FUNCTION: moveMotors()
// ARGUMENTS: errorInPosition motorSpeed
// ARGUMENT TYPES: floating point integer
// DESCRIPTION: Moves motors at speed in arguements based on the error given

/* ORIGINAL ATTEMPT AT CONTORLLING MOTORS
  Serial.print(lowByte(motorSpeed)); // Serial.print throws bytes out
  Serial.write(",0");
  Serial.print(lowByte((int)moveAmount)); // Serial.print throws bytes out  */

{
  double moveAmount; //difference; // Move amount is a % of the total motor speed, 100% being fastest and 0% being nothing

  int leftMotorSpeed, rightMotorSpeed;

  moveAmount = (errorInPosition / BUFFER_VALUE) * MAX_MOTOR_SPEED; // Move amount of motor is based on the error value

  leftMotorSpeed = leftMotorBaseSpeed - moveAmount;     // Calculate the modified motor speed
  rightMotorSpeed = rightMotorBaseSpeed + moveAmount;

  if (errorInPosition < 300 && errorInPosition > -300)   //An attempt to make the car speed up when going straight
  {
    straightMotorSpeed = straightMotorSpeed + 0.2;
    leftMotorSpeed = leftMotorBaseSpeed + straightMotorSpeed;
    rightMotorSpeed = rightMotorBaseSpeed + straightMotorSpeed;
  }
  else
  {
    straightMotorSpeed = 0;     // Causes the acceleration to reset if error is not small
  }

  // Apply new speed and direction to each motor
  if (leftMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MAX_MOTOR_SPEED);
    carController.write("#D1f");
    carController.write("#S1");
    carController.print((int)leftMotorSpeed);
  }
  else
  {
    leftMotorSpeed = leftMotorSpeed * 4;     //Makes the car accelerate in reverse 4 times faster than going forwards
    leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, 0);
    carController.write("#D1r");
    carController.write("#S1");
    carController.print(-(int)leftMotorSpeed);
  }

  if (rightMotorSpeed > 0)
  {
    rightMotorSpeed = constrain(rightMotorSpeed, 0, MAX_MOTOR_SPEED);
    carController.write("#D2f");
    carController.write("#S2");
    carController.print((int)rightMotorSpeed);
  }
  else
  {
    rightMotorSpeed = rightMotorSpeed * 4;    //Makes the car accelerate in reverse 4 times faster than going forwards
    rightMotorSpeed = constrain(rightMotorSpeed, MIN_MOTOR_SPEED, 0);
    carController.write("#D2r");
    carController.write("#S2");
    carController.print(-(int)rightMotorSpeed);
  }
}

float bufferError(float errorInPosition)
// INTERNAL FUNCTION: bufferError()
// ARGUMENTS: errorInPosition
// ARGUMENT TYPES: floating point
// DESCRIPTION: Limits the created error between certain values, prevents error becoming too large or too small
{
  if (errorInPosition > BUFFER_VALUE)
  {
    errorInPosition = BUFFER_VALUE;
  }

  if (errorInPosition < -BUFFER_VALUE)
  {
    errorInPosition = -BUFFER_VALUE;
  }

  return errorInPosition;
}

float calculateError(float sensorPosition)
// INTERNAL FUNCTION: displayStatus()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  float positionDeviation, positionDeviationToIdeal, errorInPosition;
  float P, I, D;

  positionDeviation = sensorPositionInitial - sensorPosition;             // The proportional term, calculated by finding the difference betweeen current position and the ideal found when initialising PID control

  positionDeviationSum = positionDeviationSum + positionDeviation;        // The integral term, a sum keeping track of how far away from the line the car is over time

  positionDeviationToIdeal = positionDeviation - positionDeviationOld;    // The derivative term, this determines how fast the car is moving back towards or away from the line

  positionDeviationOld = positionDeviation;

  P = calibrationConstantP * positionDeviation;

  I = calibrationConstantI * positionDeviationSum;

  D = calibrationConstantD * positionDeviationToIdeal;

  errorInPosition = P + constrain(I, -500, 500) + D;

  return errorInPosition;
}

float calculatePosition()
// INTERNAL FUNCTION: fetchPosition()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  int sensorSum;
  float sensorAverage, sensorPosition;

  sensorSum = sensorData[0] + sensorData[1] + sensorData[2] + sensorData[3] + sensorData[4] + sensorData[5] + sensorData[6] + sensorData[7];    // Each of these is a value outputted by a photodiode

  sensorAverage = (float)((sensorData[0] * 1) + (sensorData[1] * 2) + (sensorData[2] * 3) + (sensorData[3] * 4) + (sensorData[4] * 5) + (sensorData[5] * 6) + (sensorData[6] * 7) + (sensorData[7] * 8));

  sensorPosition = (float)sensorAverage / (float)sensorSum;  // This makes a weighted average of the data the sensors are receiving, giving the position of the line

  sensorPosition = sensorPosition * 2000;   // Used the make the numbers work nicely, probably unnecessary but would require a rework of a bunch of values

  return sensorPosition;
}

