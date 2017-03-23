// Include files for required libraries
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <lcd.h>
#include <softPwm.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions

int lcd, robot;     // Variables to store numeric identifiers for wiringPi

const int MAX_MOTOR_SPEED = 50;
const int MIN_MOTOR_SPEED = -50;
const int leftMotorBaseSpeed = 30;
const int rightMotorBaseSpeed = 30;
const int BUFFER_VALUE = 1000;

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

    pinMode(servo, OUTPUT);     // Configure the output to the camera servo and set to a default position
    digitalWrite(servo, LOW);
    softPwmCreate(servo,100,200);   // (pin, initial position, range)
    delay(3000);
    softPwmStop(servo);

    setupCamera(320, 240);  // Enable the camera for OpenCV
}

float constrain(float x, float a, float b)
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

double bufferError(double errorInPosition)
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

float our_integral = 0;
float proportional_old = 0;

double calculate_position(int x)
{
    float proportional, derivative;
    float p_constant = 7.5, i_constant = 0.01, d_constant = 1.8;
    double error;

    proportional = x;
    printf("P: %.3f \n", proportional);
    our_integral = our_integral + proportional;
    our_integral = constrain(our_integral, -500, 500);
    printf("I: %.3f \n", our_integral);
    derivative = proportional - proportional_old;
    printf("D: %.3f \n", derivative);
    proportional_old = proportional;

    error = (proportional * p_constant) + (our_integral * i_constant) + (derivative * d_constant);
    error = bufferError(error);
    printf("Error: %.3f \n", error);

    return(error);
}



void moveMotors(double errorInPosition)
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
        //carController.write("#S1");
        //carController.print((int)leftMotorSpeed);
    }
    else
    {
        leftMotorSpeed = leftMotorSpeed * 4;     //Makes the car accelerate in reverse 4 times faster than going forwards
        leftMotorSpeed = constrain((float)leftMotorSpeed, (float)MIN_MOTOR_SPEED, 0);
        serialPrintf(robot, "#D2r#S2%d", -(int)leftMotorSpeed);
        //carController.write("#S1");
        //carController.print(-(int)leftMotorSpeed);
    }

    if (rightMotorSpeed > 0)
    {
        rightMotorSpeed = constrain((float)rightMotorSpeed, 0, (float)MAX_MOTOR_SPEED);
        serialPrintf(robot, "#D1f#S1%d", (int)rightMotorSpeed);
        //carController.write("#S2");
        //carController.print((int)rightMotorSpeed);
    }
    else
    {
        rightMotorSpeed = rightMotorSpeed * 4;    //Makes the car accelerate in reverse 4 times faster than going forwards
        rightMotorSpeed = constrain((float)rightMotorSpeed, (float)MIN_MOTOR_SPEED, 0);
        serialPrintf(robot, "#D1r#S1%d", -(int)rightMotorSpeed);
        //carController.write("#S2");
        //carController.print(-(int)rightMotorSpeed);
    }
}


int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo");   // Create a GUI window called photo

    //Mat ramp_template;

    //Mat frameHSV, thresholded, ramp_templateHSV, ramp_template_thresholded, frame_thresholded;

    //ramp_template_thresholded = imread("/home/pi/image_processing/images/Symbols.bmp", CV_LOAD_IMAGE_UNCHANGED);
    //Mat frame = imread("/home/pi/image_processing/images/test.bmp", CV_LOAD_IMAGE_UNCHANGED);



    //cvtColor(ramp_template, ramp_templateHSV, COLOR_BGR2HSV); // Convert the image to HSV
    //inRange(ramp_templateHSV, Scalar(120, 0, 109), Scalar(179, 255, 212), ramp_template_thresholded);

    //-- Get the corners from the ramp_template ( the object to be "detected" )
    //std::vector<Point2f> obj_corners(4);
    /*obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( ramp_template_thresholded.cols, 0 );
    obj_corners[2] = cvPoint( ramp_template_thresholded.cols, ramp_template_thresholded.rows );
    obj_corners[3] = cvPoint( 0, ramp_template_thresholded.rows );*/

    while(1)    // Main loop to perform image processing
    {
        Mat gray,blur,thresholded,frame;
        std::vector<std::vector<Point> >  cont;
        std::vector<Vec4i> hierarchy;
        std::vector<cv::Point> largercontour;
        std::vector<cv::Point> con_hull;

        Rect a(0,119,320,2);
        Mat frame_org = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        frame = frame_org(a);
        cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray,blur, Size(5,5),0);
        //cv::medianBlur(blur,blur,3);
        cv::threshold(blur,thresholded,60,255,cv::THRESH_BINARY_INV);
        cv::findContours(thresholded,cont,hierarchy,0,cv::CHAIN_APPROX_NONE);
        //cv::findContours(thresholded,cont,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
        if (cont.size() > 0)
        {

            /*cv::max(cont,cv::contourArea(cont),);
            c = cv::max(contours, key=cv2.contourArea);
            M = cv2.moments(c);

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
            cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)



            */
            unsigned int i;
            Point centre;
            for (i =0 ; i<cont.size(); ++i)
            {
                //int area = (int) cv::contourArea(cont[i]);

                //if (area > max_area)
                // {
                //  largercontour = cont[i];
                // max_area = area;
                centre = findContourCentre(cont[i]);
                //}

            }
            /*cv::approxPolyDP(cv::Mat(largercontour),largercontour,5,true);
            cv::convexHull(largercontour,con_hull,false);
            if(con_hull.size()>=3)
            {i9u
                cv::Moments con_mo = cv::moments(con_hull);
                cv::Point centre = cv::Point((con_mo.m10/con_mo.m00) , (con_mo.m01/con_mo.m00));
                //cv::circle(frame,centre,1,Scalar(30,0,255),1);
            }*/
            //cv::circle(frame,centre,1,Scalar(30,0,255),1);
            printf("x = %d     y = %d\n",centre.x-160,centre.y);
            cv::drawContours(frame,cont,-1,Scalar(30,255,255),5);

            moveMotors(calculate_position(centre.x-160));

        }

        cv::imshow("Photo",frame); //Display the image in the window

        //Mat frameGray, edges, frameHSV, thresholded, ramp_template_output;

        //cvtColor(frame, frameGray, COLOR_BGR2GRAY); // Convert the image to grayscale
        //cv::Canny(frameGray, edges, 100, 100*3, 3); // Highlight the edges with a min
        //threshold of 100

        /*printf("Hello");

        //Mat frame_thresholded_before;

        //cvtColor(frame, frameHSV, COLOR_BGR2HSV); // Convert the image to HSV
        //inRange(frameHSV, Scalar(100, 30, 8), Scalar(161, 198, 87), frame_thresholded);

        //Mat kernal = cv::getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

        //cv::erode(frame_thresholded_before, frame_thresholded, kernal);

        //cv::matchTemplate(frame, ramp_template, ramp_template_output, CV_TM_CCOEFF, noArray());
        //cv::normalize(ramp_template_output, ramp_template_output, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

        /// Localizing the best match with minMaxLoc
        //double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
        //cv::Point matchLoc;
        //minMaxLoc( ramp_template_output, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

        /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
        //if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        //matchLoc = maxLoc;
        // else
        //{ matchLoc = maxLoc; }

        /// Show me what you got
        //cv::rectangle( frame, matchLoc, cv::Point( matchLoc.x + ramp_template.cols , matchLoc.y + ramp_template.rows ), cv::Scalar::all(0), 2, 8, 0 );
        //cv::rectangle( ramp_template_output, matchLoc, cv::Point( matchLoc.x + ramp_template.cols , matchLoc.y + ramp_template.rows ), cv::Scalar::all(0), 2, 8, 0 );

        //cvtColor(ramp_template_output, ramp_template_output, COLOR_GRAY2BGR);

        int minHessian = 400;
        Ptr<SURF> detector = SURF::create( minHessian );
        std::vector<KeyPoint> keypoints_object, keypoints_scene;
        Mat descriptors_object, descriptors_scene;
        detector->detectAndCompute( ramp_template_thresholded, Mat(), keypoints_object, descriptors_object );
        detector->detectAndCompute( frame_thresholded, Mat(), keypoints_scene, descriptors_scene );
        //-- Step 2: Matching descriptor vectors using FLANN matcher
        FlannBasedMatcher matcher;
        std::vector< DMatch > matches;
        matcher.match( descriptors_object, descriptors_scene, matches );
        if(matches.size() > 0)
        {
        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_object.rows; i++ )
        {
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        for( int i = 0; i < descriptors_object.rows; i++ )
        {
            if( matches[i].distance < 3*min_dist )
            {
                good_matches.push_back( matches[i]);
            }
        }
        Mat img_matches;
        drawMatches( ramp_template_thresholded, keypoints_object, frame_thresholded, keypoints_scene,
                     good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     std::vector<char>(), 0);           //DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;
        for( size_t i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }
        Mat H = findHomography( obj, scene, RANSAC );

        if(H.empty() == FALSE)
        {
            std::vector<Point2f> scene_corners(4);
            perspectiveTransform( obj_corners, scene_corners, H);
            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            line( img_matches, scene_corners[0] + Point2f( ramp_template_thresholded.cols, 0), scene_corners[1] + Point2f( ramp_template_thresholded.cols, 0), Scalar(0, 255, 0), 4 );
            line( img_matches, scene_corners[1] + Point2f( ramp_template_thresholded.cols, 0), scene_corners[2] + Point2f( ramp_template_thresholded.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[2] + Point2f( ramp_template_thresholded.cols, 0), scene_corners[3] + Point2f( ramp_template_thresholded.cols, 0), Scalar( 0, 255, 0), 4 );
            line( img_matches, scene_corners[3] + Point2f( ramp_template_thresholded.cols, 0), scene_corners[0] + Point2f( ramp_template_thresholded.cols, 0), Scalar( 0, 255, 0), 4 );
            //-- Show detected matches
            cv::imshow( "Good Matches & Object detection", img_matches );

        }

        }

        cv::imshow("Photo", ramp_template_thresholded); //Display the image in the window

        */
        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows
        //}
        key = (key==255) ? -1 : key;    // Check if the esc key has been pressed
        if (key == 27)
        {
            serialPrintf(robot, "#Hb");

            break;
        }
    }

    //closeCV();  // Disable the camera and close any windows
    while(true);
    return 0;
}





