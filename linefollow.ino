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
const int BUFFER_VALUE = 5000;
const int REVERSE_CASE = BUFFER_VALUE / 4;
const int MOTOR_SPEED = 20;
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


#include <LiquidCrystal.h>
LiquidCrystal lcd(11, 12, 13, A2, A1, A0);


void setup()
{
  Serial.begin(9600);
  carController.begin(57600);
  while (!Serial);
  while (!carController);
  
  Wire.begin();
  t = 0;

  lcd.begin(16,2);
  lcd.print("HELLO");
  delay(200);
  lcd.clear();

  // Sets initial state of button on car
  int goButton = 0;

  pinMode(2,INPUT);
  
  // Pauses car until button is pressed
  while(!goButton)
  {
    goButton = digitalRead(2);
    lcd.clear();
    lcd.print(goButton);
  }
  
  initialisePIDcontrol(); // Initialises values for PID control
  lcd.clear();
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
    

  
  int sensorArray;

  fetchSensorValues();
  sensorArray = readSensorArray();
  
  sensorPosition = calculatePosition(sensorArray); // Fetches current position of sensor

  errorInPosition = calculateError(sensorPosition); // Calculates the error in the position

  errorInPosition = bufferError(errorInPosition); // Buffers error as to not cause rapid movement
  
  moveMotors(errorInPosition, MOTOR_SPEED); // Moves motors based on the error from initial position and the speed of the motor

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
  int sensorArray;
  
  // Initial values for deviation from line at calibration is zero
  // Car is on the line so no deviation
  positionDeviationSum = 0;
  positionDeviationOld = 0;

  fetchSensorValues();
  sensorArray = readSensorArray();

  sensorPositionInitial = calculatePosition(sensorArray); // Fetches position to calibrate car on line.
}

void moveMotors(float errorInPosition, int motorSpeed)
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

  // If error in position is +ve then slow a motor down to change direction
  if((errorInPosition > 0) && (errorInPosition < REVERSE_CASE))
  {
    moveAmount = (errorInPosition / BUFFER_VALUE)*motorSpeed; // move amount of motor is based on the error value
    moveAmount = motorSpeed - moveAmount; // creates change in motor speed
    carController.write("#Dbf"); // Command to set direction of both motors forward
    carController.write("#Sb0"); // Command to Start both motors
    carController.print(lowByte((int)moveAmount)); // Outputs value of motor speed
    carController.write(",0"); // Setsup for next motorspeed
    carController.print(lowByte((int)motorSpeed)); // Outputs value of motor speed
  }

  // If error in position is -ve then slow a motor down to change direction
  if((errorInPosition < 0) && (errorInPosition > -REVERSE_CASE))
  {
    moveAmount = (errorInPosition / -BUFFER_VALUE)*motorSpeed;
    moveAmount = motorSpeed - moveAmount;
    carController.write("#Dbf");
    carController.write("#Sb0");
    carController.print(lowByte((int)motorSpeed)); // Serial.print throws bytes out
    carController.write(",0");
    carController.print(lowByte((int)moveAmount)); // Serial.print throws bytes out
  }

  // If error in position is greater than a certain threshold then drastic action is required to bring vehicle platform back on course
  // By reversing motors on one side and forward motion in the other the vehicle will turn on its own axis like a tank
  if(errorInPosition > REVERSE_CASE)
  {
    moveAmount = (errorInPosition / BUFFER_VALUE)*motorSpeed; // move amount of motor is based on the error value
    moveAmount = motorSpeed - moveAmount; // creates change in motor speed
    carController.write("#D1r");
    carController.write("#D2f");
    //carController.write("#Sb035,020");
    carController.write("#Sb0");
    carController.print(lowByte((int)motorSpeed));
    carController.write(",0");
    carController.print(lowByte((int)motorSpeed));
  }

  // If error in position is greater than a certain threshold then drastic action is required to bring vehicle platform back on course
  // By reversing motors on one side and forward motion in the other the vehicle will turn on its own axis like a tank
  if(errorInPosition < -REVERSE_CASE)
  {
    moveAmount = (errorInPosition / BUFFER_VALUE)*motorSpeed; // move amount of motor is based on the error value
    moveAmount = motorSpeed - moveAmount; // creates change in motor speed
    carController.write("#D1f");
    carController.write("#D2r");
    carController.write("#Sb0");
    carController.print(lowByte((int)motorSpeed));
    carController.write(",0");
    carController.print(lowByte((int)motorSpeed));
  }

  // If error in position is zero then vehicle is perfectly on the line, so move forward
  if(errorInPosition == 0)
  {
    carController.write("#Dbf");
    carController.write("#Sb0");
    carController.print(lowByte((int)motorSpeed));
    carController.write(",0");
    carController.print(lowByte((int)motorSpeed));
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


//  lcd.setCursor(0,0);
//  lcd.print(sensorArray[0]);
//  lcd.print(" ");
//  lcd.print(sensorArray[1]);
//  lcd.print(" ");
//  lcd.print(sensorArray[2]);
//  lcd.print(" ");
//  lcd.print(sensorArray[3]);
//  lcd.setCursor(0,1);
//  lcd.print(sensorArray[4]);
//  lcd.print(" ");
//  lcd.print(sensorArray[5]);
//  lcd.print(" ");
//  lcd.print(sensorArray[6]);
//  lcd.print(" ");
//  lcd.print(sensorArray[7]);

  return sensorArray;
}

float calculateError(float sensorPosition)
// INTERNAL FUNCTION: displayStatus()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  float positionDeviation, positionDeviationToIdeal, errorInPosition, calibrationConstantPosDev, calibrationConstantPosDevToIdeal, calibrationConstantPosDevOld;
  
  calibrationConstantPosDev = 1; // 3 at the end of last project week
  calibrationConstantPosDevToIdeal = 1; // 5 at the end of last project week
  calibrationConstantPosDevOld = 1;  // 0 at end of last project week

  positionDeviation = sensorPositionInitial - sensorPosition;

  positionDeviationSum = positionDeviationSum + positionDeviation;

  positionDeviationToIdeal = positionDeviation - positionDeviationOld;

  positionDeviationOld = positionDeviation;

  errorInPosition = calibrationConstantPosDev*positionDeviation + calibrationConstantPosDevToIdeal*positionDeviationToIdeal + calibrationConstantPosDevOld*positionDeviationOld;

  return errorInPosition;
}

float calculatePosition(int * sensorArray)
// INTERNAL FUNCTION: fetchPosition()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  int sensorValue0, sensorValue1, sensorValue2, sensorValue3, sensorValue4, sensorValue5, sensorValue6, sensorValue7, sensorSum;
  float sensorAverage, sensorPosition;

  sensorValue0 = 1023 - *(sensorArray + 0);
  sensorValue1 = 1023 - *(sensorArray + 1);
  sensorValue2 = 1023 - *(sensorArray + 2);
  sensorValue3 = 1023 - *(sensorArray + 3);
  sensorValue4 = 1023 - *(sensorArray + 4);
  sensorValue5 = 1023 - *(sensorArray + 5);
  sensorValue6 = 1023 - *(sensorArray + 6);
  sensorValue7 = 1023 - *(sensorArray + 7);

  sensorSum = sensorValue0 + sensorValue1 + sensorValue2 + sensorValue3 + sensorValue4 + sensorValue5 + sensorValue6 + sensorValue7;

  sensorAverage = (float)((sensorValue0*1) + (sensorValue1*2) + (sensorValue2*3) + (sensorValue3*4) + (sensorValue4*5) + (sensorValue5*6) + (sensorValue6*7) + (sensorValue7*8));

  sensorPosition = (float)sensorAverage/(float)sensorSum;

  sensorPosition = sensorPosition * 1000;

  lcd.setCursor(0,0);
  lcd.print(sensorPosition);

  return sensorPosition;
}
