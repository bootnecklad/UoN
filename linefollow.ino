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

// Global Constants
const int BUFFER_VALUE = 2000;
const int REVERSE_CASE = 1500;
const int MAX_MOTOR_SPEED = 40;
const int MIN_MOTOR_SPEED = -40;
#define leftMotorBaseSpeed 20
#define rightMotorBaseSpeed 20
const int CALIBRATION_DELAY = 2000;
const int LCD_ROW = 2;
const int LCD_COLUMN = 16;
const int LOOP_DELAY = 0;

const int TOO_FAST = 7;         // defines various error codes

// Global Variables
// Could be improve by creating a structure that represents the system as a whole?

// Values used to calculate sensor of position on car
float sensorPosition;           // Current position of sensor
float sensorPositionInitial;    // Initial position of sensor when calibration after button press
float positionDeviationSum;     // Continuous sum of how far the sensor has deviated over time
float positionDeviationOld;     // Previous value of how far the sensor has deviated over time
float errorInPosition;          // Total error of sensor compared to the ideal/initial position


// setups serial port for car controller board
#include <SoftwareSerial.h>
SoftwareSerial carController(9, 10);

// LINE SENSOR SETUP
#include <Wire.h>
#define uchar unsigned char
uchar t;
//void send_data(short a1,short b1,short c1,short d1,short e1,short f1);
uchar data[16];


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

  pinMode(2,INPUT);
  
  // Pauses car until button is pressed
  while(!goButton)
  {
    goButton = digitalRead(2);
  }
  
  initialisePIDcontrol(); // Initialises values for PID control
}

void loop()
// INTERNAL FUNCTION: loop()
// ARGUMENTS: 
// ARGUMENT TYPES:
// DESCRIPTION: Arduino loop that runs and sends commands to the firmware board
{
  int goButton;

  goButton = digitalRead(2);

  if(goButton)
  {
    carController.write("#Hb");
    initialisePIDcontrol();
  }

  fetchSensorValues();
  
  sensorPosition = calculatePosition(); // Fetches current position of sensor

  errorInPosition = calculateError(sensorPosition); // Calculates the error in the position
  
  errorInPosition = bufferError(errorInPosition); // Buffers error as to not cause rapid movement
  
  moveMotors(errorInPosition); // Moves motors based on the error from initial position and the speed of the motor

  //displayStatus(errorInPosition, sensorArray); // Continuously displays information on LCD on top of vehicle platform
  
  //delay(LOOP_DELAY); // Causes a delay inbetween each loop
}

unsigned int sensorData[8];

void fetchSensorValues()
{  
  Wire.requestFrom(9, 16);    // request 16 bytes from slave device #9
  while (Wire.available())   // slave may send less than requested
  {
    data[t] = Wire.read(); // receive a byte as character
    if (t < 15)
      t++;
    else
      t = 0;
  }

  int n;

    for(n=0;n<8;n++)
  {
    sensorData[n] = data[n*2]<< 2;   // Shift the 8 MSBs up two places and store in array
    sensorData[n] += data[(n*2)+1];  // Add the remaining bottom 2 LSBs

    // Apply the calibration values here!
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

  moveAmount = (errorInPosition / BUFFER_VALUE)*MAX_MOTOR_SPEED; // Move amount of motor is based on the error value

  leftMotorSpeed = leftMotorBaseSpeed - moveAmount;     // Calculate the modified motor speed
  rightMotorSpeed = rightMotorBaseSpeed + moveAmount;

  // Apply new speed and direction to each motor
  if(leftMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MAX_MOTOR_SPEED);
    carController.write("#D1f");
    carController.write("#S1");
    carController.print((int)leftMotorSpeed);
  }
  else
  {
    leftMotorSpeed = constrain(leftMotorSpeed, MIN_MOTOR_SPEED, 0);
    carController.write("#D1r");
    carController.write("#S1");
    carController.print(-(int)leftMotorSpeed);
  }

  if(rightMotorSpeed > 0)
  {
    rightMotorSpeed = constrain(rightMotorSpeed, 0, MAX_MOTOR_SPEED);
    carController.write("#D2f");
    carController.write("#S2");
    carController.print((int)rightMotorSpeed);
  }
  else
  {
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
  if(errorInPosition > BUFFER_VALUE)
  {
    errorInPosition = BUFFER_VALUE;
  }

  if(errorInPosition < -BUFFER_VALUE)
  {
    errorInPosition = -BUFFER_VALUE;
  }
  
// Experimenting with low error values to continue vehicle foward
//  if((errorInPosition < 20) && (errorInPosition > -20))
//  {
//    errorInPosition = 0;
//  }

  return errorInPosition;
}

int * readSensorArray()
{
  static int sensorArray[8];

  sensorArray[0] = sensorData[0];
  sensorArray[1] = sensorData[1];
  sensorArray[2] = sensorData[2];
  sensorArray[3] = sensorData[3];
  sensorArray[4] = sensorData[4];
  sensorArray[5] = sensorData[5];
  sensorArray[6] = sensorData[6];
  sensorArray[7] = sensorData[7];

  return sensorArray;
}

float calculateError(float sensorPosition)
// INTERNAL FUNCTION: displayStatus()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  float positionDeviation, positionDeviationToIdeal, errorInPosition, calibrationConstantPosDev, calibrationConstantPosDevToIdeal, calibrationConstantPosDevOld;
  
  calibrationConstantPosDev = 2; // 3 at the end of last project week
  calibrationConstantPosDevToIdeal = 1; // 5 at the end of last project week
  calibrationConstantPosDevOld = 0.05;  // 0 at end of last project week

  positionDeviation = sensorPositionInitial - sensorPosition;

  positionDeviationSum = positionDeviationSum + positionDeviation;

  positionDeviationToIdeal = positionDeviation - positionDeviationOld;

  positionDeviationOld = positionDeviation;

  errorInPosition = calibrationConstantPosDev*positionDeviation + calibrationConstantPosDevToIdeal*positionDeviationToIdeal + calibrationConstantPosDevOld*positionDeviationSum;

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
  
  sensorSum = sensorData[0] + sensorData[1] + sensorData[2] + sensorData[3] + sensorData[4] + sensorData[5] + sensorData[6] + sensorData[7];
  
  sensorAverage = (float)((sensorData[0]*1) + (sensorData[1]*2) + (sensorData[2]*3) + (sensorData[3]*4) + (sensorData[4]*5) + (sensorData[5]*6) + (sensorData[6]*7) + (sensorData[7]*8));

  sensorPosition = (float)sensorAverage/(float)sensorSum;

  sensorPosition = sensorPosition * 2000;

  return sensorPosition;
}
