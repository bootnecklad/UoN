// Include files for required libraries
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <lcd.h>
#include <softPwm.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions

int lcd, robot;     // Variables to store numeric identifiers for wiringPi

const int MAX_MOTOR_SPEED = 30;
const int MIN_MOTOR_SPEED = -30;
const int leftMotorBaseSpeed = 15;
const int rightMotorBaseSpeed = 15;
const int BUFFER_VALUE = 1000;
const int servoPin = 1;
void setup(void)
{
    wiringPiSetup();    // Initialise the wiringPi library

    pinMode(TRIG, OUTPUT);      // Setup the SR04 sensor pins
    pinMode(ECHO, INPUT);
    digitalWrite(TRIG, LOW);    // Prepare the trigger output of the SR04
    delay(50);

    pinMode(robotReset, OUTPUT);    // Setup the robotReset pin
    robot = serialOpen("/dev/serial0", 57600);  // Configure the serial port for the robot and bounce the reset pin
    digitalWrite(robotReset, LOW);
    delayMicroseconds(50);
    digitalWrite(robotReset, HIGH);

    lcd = lcdInit(2,16, 4, 29,21, 22,23,24,25,0,0,0,0); // Setup the LCD screen and display a message
    lcdPosition(lcd, 0,0);
    lcdPuts(lcd, "Hello World!");

    pinMode(servoPin, OUTPUT);     // Configure the output to the camera servo and set to a default position
    digitalWrite(servoPin, LOW);
    softPwmCreate(servoPin,14,200);   // (pin, initial position, range)
    delay(1000);
    softPwmWrite(servoPin,5);
    //softPwmStop(servoPin);
    delay(1000);
    //softPwmStop(servoPin);
    softPwmWrite(servoPin,14);   // (pin, initial position, range)
    delay(1000);


    setupCamera(320, 240);  // Enable the camera for OpenCV
}

float constrain(float x, float a, float b)  //Can send any float x and constrain it between a and b
{
    if(x < a)
    {
        return a;
    }
    else if(b < x)
    {
        return b;
    }
    else
        return x;
}


bool seeing_purple(Mat frame, int pixel_number)
{

    Mat frameHSV;
    cv::cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    cv::inRange(frameHSV, Scalar(144, 78, 20), Scalar(179, 152, 255), frameHSV);
    int purple_pixels = cv::countNonZero(frameHSV);
    cv::imshow("HSV", frameHSV);
    delay(5);

    if(purple_pixels > pixel_number)
    {
        return true;
    }
    else
    {
        return false;
    }


}


Mat symbol_captured(Mat frame)
{
    Mat emptyMat=imread("/home/pi/image_processing/images/SymbolsNew/Slide1.PNG");
    cv::cvtColor(emptyMat, emptyMat, COLOR_BGR2HSV);
    cv::inRange(emptyMat, Scalar(0, 0, 0), Scalar(0, 0, 0), emptyMat);
    Mat frameHSV;
    cv::cvtColor(frame, frameHSV, COLOR_BGR2HSV);
    cv::inRange(frameHSV, Scalar(121, 39, 111), Scalar(179, 215, 255), frameHSV);
    //cv:imshow("test", frameHSV);
    //cv::waitKey(1000);
    int purple_pixels = cv::countNonZero(frameHSV);
    if(purple_pixels > 5000)
    {
        printf("An image!\n");
        std::vector<std::vector<Point> >  cont;

        std::vector<Vec4i> hierarchy;
        std::vector<Point> largest_contour;
        //cv::blur(frameHSV, frameHSV, Size(5,5), Point(-1,-1));
        //cv::threshold(frameHSV, frameHSV, 80, 255, THRESH_BINARY);
        cv::findContours(frameHSV,cont,hierarchy,0,cv::CHAIN_APPROX_NONE);

        if (cont.size() > 0)
        {
            std::vector<Point> simple_cont;
            unsigned int i;
            double largest_area = 0;
            Rect symbol_contour;

            for (i =0 ; i<cont.size(); ++i)
            {
                double area = cv::contourArea(cont[i]);

                if(area > largest_area)
                {
                    largest_area = area;
                    //symbol_contour = cv::boundingRect(cont[i]);
                    largest_contour = cont[i];
                }
            }

            cv::approxPolyDP(largest_contour,simple_cont,40,true);
            frameHSV = transformPerspective(simple_cont,frameHSV,320,240);          //frameHSV = frameHSV(symbol_contour);
            //frameHSV = transformPerspective(largest_contour, frameHSV, 100, 100);
            //cv::drawContours(frameHSV,largest_contour,-1,cv::Scalar(0,255,0),5);
            if (frameHSV.rows != 0 && frame.cols != 0)
            {

                cv::blur(frameHSV, frameHSV, Size(5,5), Point(-1,-1));
                cv::threshold(frameHSV, frameHSV, 80, 255, THRESH_BINARY);
                cv::imshow("name",frameHSV);
                return frameHSV;
            }
            else
            {
                return emptyMat;
            }
        }
    }
    else
    {
        printf("Nothing\n");
        return emptyMat;
    }
}

Mat symbolHSV(Mat symbol)
{
    cv::cvtColor(symbol, symbol, COLOR_BGR2HSV);
    cv::inRange(symbol, Scalar(144, 15, 20), Scalar(179, 255, 255), symbol);
    return symbol;
}

float our_integral_top = 0;     //Called our_integral as integral already used in library
float proportional_old_top = 0; //Previous value of position, used to calculate the derivative

float our_integral_middle = 0;     //Called our_integral as integral already used in library
float proportional_old_middle = 0; //Previous value of position, used to calculate the derivative

float calculate_error(int x, String strip_position)    //Calculates error using PID calculations
{
    float proportional, derivative;
    float p_constant = 10, i_constant = 0.4, d_constant = 10;
    float error;

    if(strip_position == "TOP")
    {
        proportional = x;           // The proportional term, calculated by finding the current position
        printf("P: %.3f \n", proportional);

        our_integral_top = our_integral_top + proportional;         // The integral term, a sum keeping track of how far away from the line the car is over time
        our_integral_top = constrain(our_integral_top, -500, 500);
        printf("I: %.3f \n", our_integral_top);

        derivative = proportional - proportional_old_top;       // The derivative term, this determines how fast the car is moving back towards or away from the line
        printf("D: %.3f \n", derivative);

        proportional_old_top = proportional;        //Recording the previous value of position
        error = (proportional * p_constant) + (our_integral_top * i_constant) + (derivative * d_constant);
    }

    else if(strip_position == "MIDDLE")
    {
        proportional = x;           // The proportional term, calculated by finding the current position
        printf("P: %.3f \n", proportional);

        our_integral_middle = our_integral_middle + proportional;         // The integral term, a sum keeping track of how far away from the line the car is over time
        our_integral_middle = constrain(our_integral_middle, -500, 500);
        printf("I: %.3f \n", our_integral_middle);

        derivative = proportional - proportional_old_middle;       // The derivative term, this determines how fast the car is moving back towards or away from the line
        printf("D: %.3f \n", derivative);

        proportional_old_middle = proportional;

        error = (proportional * p_constant) + (our_integral_middle * i_constant) + (derivative * d_constant);
    }

    error = constrain(error, -BUFFER_VALUE, BUFFER_VALUE);
    printf("Error: %.3f \n", error);

    return(error);
}

void moveMotors(float errorInPosition)
// INTERNAL FUNCTION: moveMotors()
// ARGUMENTS: errorInPosition
// ARGUMENT TYPES: floating point integer
// DESCRIPTION: Moves motors at speed in arguments based on the error given

{
    float moveAmount; //difference; // Move amount is a % of the total motor speed, 100% being fastest and 0% being nothing

    int leftMotorSpeed, rightMotorSpeed;

    moveAmount = (errorInPosition/BUFFER_VALUE) * MAX_MOTOR_SPEED; // Move amount of motor is based on the error value

    leftMotorSpeed = leftMotorBaseSpeed - moveAmount;     // Calculate the modified motor speed
    rightMotorSpeed = rightMotorBaseSpeed + moveAmount;

    /*if (errorInPosition < 300 && errorInPosition > -300)   //An attempt to make the car speed up when going straight
    {
        straightMotorSpeed = straightMotorSpeed + 0.2;
        leftMotorSpeed = leftMotorBaseSpeed + straightMotorSpeed;
        rightMotorSpeed = rightMotorBaseSpeed + straightMotorSpeed;
    }
    else
    {
        straightMotorSpeed = 0;     // Causes the acceleration to reset if error is not small
    }*/

    // Apply new speed and direction to each motor
    if (leftMotorSpeed > 0)
    {
        leftMotorSpeed = constrain((float)leftMotorSpeed, 0, (float)MAX_MOTOR_SPEED);
        serialPrintf(robot, "#D2f#S2%d", (int)leftMotorSpeed);
    }
    else
    {
        leftMotorSpeed = leftMotorSpeed * 4;     //Makes the car accelerate in reverse 4 times faster than going forwards
        leftMotorSpeed = constrain((float)leftMotorSpeed, (float)MIN_MOTOR_SPEED, 0);
        serialPrintf(robot, "#D2r#S2%d", -(int)leftMotorSpeed);
    }

    if (rightMotorSpeed > 0)
    {
        rightMotorSpeed = constrain((float)rightMotorSpeed, 0, (float)MAX_MOTOR_SPEED);
        serialPrintf(robot, "#D1f#S1%d", (int)rightMotorSpeed);
    }
    else
    {
        rightMotorSpeed = rightMotorSpeed * 4;    //Makes the car accelerate in reverse 4 times faster than going forwards
        rightMotorSpeed = constrain((float)rightMotorSpeed, (float)MIN_MOTOR_SPEED, 0);
        serialPrintf(robot, "#D1r#S1%d", -(int)rightMotorSpeed);
    }
}


int find_contour_centre(Mat strip,String name)  //Can be sent an array of pixels, find contours in them and display the results in a named window. Returns the centre point of the contour
{
    Point centre;

    std::vector<std::vector<Point> >  cont;
    std::vector<Vec4i> hierarchy;
    Mat gray,blur,thresholded;
    cv::cvtColor(strip,gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray,blur, Size(5,5),0);
    cv::threshold(blur,thresholded,60,255,cv::THRESH_BINARY_INV);
    cv::findContours(thresholded,cont,hierarchy,0,cv::CHAIN_APPROX_NONE);

    if (cont.size() > 0)
    {

        unsigned int i;

        for (i =0 ; i<cont.size(); ++i)
        {
            centre = findContourCentre(cont[i]);
        }
        cv::drawContours(strip,cont,-1,Scalar(30,255,255),5);
        cv::imshow(name,strip);
    }

    printf("\n Centre: %d\n", centre.x - 160);
    return (centre.x);
}

void match_image(Mat symbolsArray[9], Mat frame_org)
{
    float match = 0;
    float maxMatch = 0;
    int symbol_ID = 100;

    for (int i = 0; i<9; i++)
    {
        match = compareImages(symbol_captured(frame_org), symbolsArray[i]);

        if(match > maxMatch)
        {
            maxMatch = match;
            symbol_ID = i;
        }
    }
    printf("Best match is symbol: %d, match percentage: %.f\n", symbol_ID, maxMatch);
}

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices
    Mat symbolsArray[9];
    symbolsArray[0] = imread("/home/pi/image_processing/images/SymbolsNew/Slide1.PNG");
    symbolsArray[1] = imread("/home/pi/image_processing/images/SymbolsNew/Slide2.PNG");
    symbolsArray[2] = imread("/home/pi/image_processing/images/SymbolsNew/Slide3.PNG");
    symbolsArray[3] = imread("/home/pi/image_processing/images/SymbolsNew/Slide4.PNG");
    symbolsArray[4] = imread("/home/pi/image_processing/images/SymbolsNew/Slide5.PNG");
    symbolsArray[5] = imread("/home/pi/image_processing/images/SymbolsNew/Slide6.PNG");
    symbolsArray[6] = imread("/home/pi/image_processing/images/SymbolsNew/Slide7.PNG");
    symbolsArray[7] = imread("/home/pi/image_processing/images/SymbolsNew/Slide8.PNG");
    symbolsArray[8] = imread("/home/pi/image_processing/images/SymbolsNew/Slide9.PNG");

    for (int i = 0; i<9; i++)
    {
        symbolsArray[i] = symbolHSV(symbolsArray[i]);
    }
    int mode = 1;
    while(1)    // Main loop to perform image processing
    {
        switch (mode)
        {
        case 1://line following
        {
            Mat frame_org = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
            //cv::imshow("camera",frame_org);

            Mat frame_middle;
            float error_middle, centre_middle;
            Rect a(0, 119, 320, 2);

            //Mat frame_org = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
            frame_middle = frame_org(a);

            centre_middle = find_contour_centre(frame_middle,"Middle Strip");

            centre_middle = centre_middle - 160;

            error_middle = calculate_error(centre_middle, "MIDDLE");

            float error = error_middle;

            moveMotors(error);
            if (seeing_purple(frame_org, 5000)==true)
            {
                mode = 2;
            }

            break;
        }

        case 2: //saw the square
        {
            printf("Purple!");
            serialPrintf(robot, "#Hb");
            delay(1000);

            softPwmWrite(servoPin,5);
            delay(1000);
            Mat frame_org2 = captureFrame();
            frame_org2 = captureFrame();
frame_org2 = captureFrame();
frame_org2 = captureFrame();
frame_org2 = captureFrame();
            do
            {
                delay(10);
                serialPrintf(robot, "#Bbfr020,020");
                frame_org2 = captureFrame();
            }
            while(seeing_purple(frame_org2, 500) == false);
            serialPrintf(robot, "#Hb");
            mode=3 ;
            break;
        }
        case 3:
        {
            resizeCamera(640, 480);
            Mat frame_org3 = captureFrame();
            frame_org3 = captureFrame();
            cv::flip(frame_org3,frame_org3,-1);
            cv::imshow("3", frame_org3);
            match_image(symbolsArray,frame_org3);
            break;
        }

        }




        /*Mat frame_org = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        //cv::imshow("camera",frame_org);

        Mat frame_middle;
        float error_middle, centre_middle;
        Rect a(0, 119, 320, 2);

        //Mat frame_org = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        frame_middle = frame_org(a);

        centre_middle = find_contour_centre(frame_middle,"Middle Strip");

        centre_middle = centre_middle - 160;

        error_middle = calculate_error(centre_middle, "MIDDLE");

        float error = error_middle;

        moveMotors(error);

        if (seeing_purple(frame_org) == true)
        {
            //match_image(symbolsArray, frame_org);
            printf("Purple!");
            serialPrintf(robot, "#Hb");
            delay(1000);

            softPwmWrite(servoPin,5);
            //delay(1000);
            //softPwmStop(servoPin);
            //delay(5000);
            Mat frame_org2;
            do
            {
                //delay(10);
                serialPrintf(robot, "#Bbfr020,020");

                frame_org2 = captureFrame();
            }
            while(seeing_purple(frame_org2) == false);

            serialPrintf(robot, "#Hb");
            printf("More purple!");


            //closeCV();  // Disable the camera and close any windows
            //return 0;

        }
        //match = compareImages(check_purple(frame_org),);
        //printf("\n\n\n%.f\n\n\n",match);
        //check_purple(frame_org);
        */
        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows) (DO NOT DELETE THIS LINE FOR ANY REASON)

        key = (key==255) ? -1 : key;    // Check if the esc key has been pressed
        if (key == 27)
        {
            serialPrintf(robot, "#Hb");
            closeCV();  // Disable the camera and close any windows
            return 0;
        }

        if(key>0)
            printf("%d\n", key);

    }

    closeCV();  // Disable the camera and close any windows

    return 0;
}
