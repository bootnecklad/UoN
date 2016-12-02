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

#include <LiquidCrystal.h> // Allows use of LCD display

// Global Constants
const int BUFFER_VALUE = 256;
const int REVERSE_CASE = 200;
const int MOTOR_SPEED = 20;
const int CALIBRATION_DELAY = 2000;
const int LCD_ROW = 2;
const int LCD_COLUMN = 16;
const int BAUD_RATE = 57600;
const int DIGITAL_LEFT = 6;
const int DIGITAL_RIGHT = 7;
const int STATUS_LED = 13;
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


// initialize the library with the interface pin numbers
LiquidCrystal lcd(12 /* rs */,
                  11 /* enable */,
                  5  /* d0 */,
                  4  /* d1 */,
                  3  /* d2 */,
                  2  /* d3 */);
                  
// Function Prototypes
void setup();                   // called to initialize program
void loop();                    // called on each loop iteration

float bufferError(float errorInPosition);
float calculateError(float sensorPosition);
float calculatePosition(int * sensorArray);
long unsigned int counterAverage();
void displayError(char errorCode);
void displayStatus(float errorInPosition, int * sensorArray);
void moveMotors(float errorInPosition, int motorSpeed);
long unsigned int readCounter(char motor);
int * readSensorArrayAnalogue();
int * readSensorArrayDigital();


// Function Implementations

void setup()
{
  int sensorArray, goButton;
  
  lcd.begin(LCD_COLUMN, LCD_ROW); // Setups LCD
  
  Serial.begin(BAUD_RATE); // Setups serial port communication to controller board
  
  pinMode(STATUS_LED, OUTPUT); // Status LED
  pinMode(DIGITAL_LEFT, INPUT); // digital output of left sensor
  pinMode(DIGITAL_RIGHT, INPUT); // digital output of right sensor
  
  // Initial values for deviation from line at calibration is zero
  // Car is on the line so no deviation
  positionDeviationSum = 0;
  positionDeviationOld = 0;

  // Checks motorspeed, shouldn't be greater than 100
  if(MOTOR_SPEED > 100)
  {
    displayError(TOO_FAST);
  }

  // Sets initial state of button on car
  goButton = 0;
  
  // Pauses car until button is pressed
  while(!goButton)
  {
    goButton = digitalRead(8);
    // Prints out funny message
    lcd.setCursor(0,0);
    lcd.print("HELLO HUMANS");
    lcd.setCursor(0,1);
    // Humour is needed when the car doesn't work as desired
    lcd.print("SHOW ME LINES");
  }
  
  lcd.setCursor(0,0);
  lcd.print("EAT LINE MODE:");

  sensorArray = readSensorArrayAnalogue();
  sensorPositionInitial = calculatePosition(sensorArray); // Fetches position to calibrate car on line.

  // Delays car before starting so user pressing button doesn't get run over 
  delay(CALIBRATION_DELAY);
  lcd.setCursor(0,1);
  lcd.print("ENGAGED");
  lcd.clear();
}

void loop()
// INTERNAL FUNCTION: loop()
// ARGUMENTS: 
// ARGUMENT TYPES:
// DESCRIPTION: Arduino loop that runs and sends commands to the firmware board
{

  int sensorArray;

  sensorArray = readSensorArrayAnalogue();
  
  sensorPosition = calculatePosition(sensorArray); // Fetches current position of sensor

  errorInPosition = calculateError(sensorPosition); // Calculates the error in the position

  errorInPosition = bufferError(errorInPosition); // Buffers error to +/- 256 as to not cause rapid movement

  moveMotors(errorInPosition, MOTOR_SPEED); // Moves motors based on the error from initial position and the speed of the motor

  displayStatus(errorInPosition, sensorArray); // Continuously displays information on LCD on top of vehicle platform
  
  delay(LOOP_DELAY); // Causes a delay inbetween each loop
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
    Serial.write("#Dbf"); // Command to set direction of both motors forward
    Serial.write("#Sb0"); // Command to Start both motors
    Serial.print(lowByte((int)moveAmount)); // Outputs value of motor speed
    Serial.write(",0"); // Setsup for next motorspeed
    Serial.print(lowByte(motorSpeed)); // Outputs value of motor speed
  }

  // If error in position is -ve then slow a motor down to change direction
  if((errorInPosition < 0) && (errorInPosition > -REVERSE_CASE))
  {
    moveAmount = (errorInPosition / -BUFFER_VALUE)*motorSpeed;
    moveAmount = motorSpeed - moveAmount;
    Serial.write("#Dbf");
    Serial.write("#Sb0");
    Serial.print(lowByte(motorSpeed)); // Serial.print throws bytes out
    Serial.write(",0");
    Serial.print(lowByte((int)moveAmount)); // Serial.print throws bytes out
  }

  // If error in position is greater than a certain threshold then drastic action is required to bring vehicle platform back on course
  // By reversing motors on one side and forward motion in the other the vehicle will turn on its own axis like a tank
  if(errorInPosition > REVERSE_CASE)
  {
    moveAmount = (errorInPosition / BUFFER_VALUE)*motorSpeed; // move amount of motor is based on the error value
    moveAmount = motorSpeed - moveAmount; // creates change in motor speed
    Serial.write("#D1r");
    Serial.write("#D2f");
    Serial.write("#Sb020,020");
  }

  // If error in position is greater than a certain threshold then drastic action is required to bring vehicle platform back on course
  // By reversing motors on one side and forward motion in the other the vehicle will turn on its own axis like a tank
  if(errorInPosition < -REVERSE_CASE)
  {
    moveAmount = (errorInPosition / BUFFER_VALUE)*motorSpeed; // move amount of motor is based on the error value
    moveAmount = motorSpeed - moveAmount; // creates change in motor speed
    Serial.write("#D1f");
    Serial.write("#D2r");
    Serial.write("#Sb020,035");
  }

  // If error in position is zero then vehicle is perfectly on the line, so move forward
  if(errorInPosition == 0)
  {
    Serial.write("#Dbf");
    Serial.write("#Sb0");
    Serial.print(lowByte(motorSpeed));
    Serial.write(",0");
    Serial.print(lowByte(motorSpeed));
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

int * readSensorArrayDigital()
// INTERNAL FUNCTION: readSensorArray()
// ARGUMENTS:
// ARGUMENT TYPES:
// DESCRIPTION: Reads the values of the digital outputs of the amplifier board
{
  static int sensorArray[4];
  int place_counter;

  for(place_counter=0; place_counter<4; place_counter++)
  {
     sensorArray[place_counter] = digitalRead(6+place_counter);
  }

  return sensorArray;
}

int * readSensorArrayAnalogue()
// INTERNAL FUNCTION: readSensorArrayAnalogue()
// ARGUMENTS:
// ARGUMENT TYPES:
// DESCRIPTION: Reads the values of the Analogue outputs of the amplifier board
{
  static int sensorArray[4]; // To store all sensor values in
  
  sensorArray[0] = analogRead(A0);
  sensorArray[1] = analogRead(A1);
  sensorArray[2] = analogRead(A2);
  sensorArray[3] = analogRead(A3);

  return sensorArray;
}

float calculateError(float sensorPosition)
// INTERNAL FUNCTION: displayStatus()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{
  float positionDeviation, positionDeviationToIdeal, errorInPosition, calibrationConstantPosDev, calibrationConstantPosDevToIdeal, calibrationConstantPosDevOld;
  
  calibrationConstantPosDev = 3;
  calibrationConstantPosDevToIdeal = 5;
  calibrationConstantPosDevOld = 0;

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
  int sensorValueA, sensorValueB, sensorValueC, sensorValueD, sensorSum;
  float sensorAverage, sensorPosition;

  sensorValueA = *(sensorArray + 0);
  sensorValueB = *(sensorArray + 1);
  sensorValueC = *(sensorArray + 2);
  sensorValueD = *(sensorArray + 3);

  sensorSum = sensorValueA + sensorValueB + sensorValueC + sensorValueD;

  sensorAverage = (float)((sensorValueA*1) + (sensorValueB*2) + (sensorValueC*3) + (sensorValueD*4));

  sensorPosition = (float)sensorAverage/(float)sensorSum;

  sensorPosition = sensorPosition * 1000;

  return sensorPosition;
}


void displayStatus(float errorInPosition, int * sensorArray)
// INTERNAL FUNCTION: displayStatus()
// ARGUMENTS: errorInPosition sensorArray
// ARGUMENT TYPES: floating point pointer to array
// DESCRIPTION: Displays error in position and the values of each sensor
{  
  lcd.setCursor(0,1);
  lcd.print(*(sensorArray+0)); // DIRTY POINTER ARRAY HACK NOT PROUD :'(
  
  lcd.setCursor(2,1);
  lcd.print(*(sensorArray+1));

  lcd.setCursor(4,1);
  lcd.print(*(sensorArray+2)); 

  lcd.setCursor(8,1);
  lcd.print(*(sensorArray+3)); 

  lcd.setCursor(0,0);
  lcd.print(errorInPosition);
}

void displayError(char errorCode)
// INTERNAL FUNCTION: displayError()
// ARGUMENTS: errorCode
// ARGUMENT TYPES: char
// DESCRIPTION: Displays error on LCD and stops motors to prevent damage
{
  Serial.write("#Hb"); // Command to halt both motors
  lcd.clear(); // Clears LCD of everything
  lcd.setCursor(0,0);
  lcd.print("ERROR :("); // Errors are bad sad face :(
  lcd.setCursor(0,1);
  lcd.print(errorCode); // Displays error code given so team knows what to look up

  // Sends controller into infinite loop so it does nothing
  for(;;)
  {
    delay(1000);
  }
}
