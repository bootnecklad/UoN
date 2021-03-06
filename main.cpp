// Include files for required libraries
#include <stdio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <lcd.h>
#include <softPwm.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions

int lcd, robot;     // Variables to store numeric identifiers for wiringPi

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

int main( int argc, char** argv )
{
    setup();    // Call a setup function to prepare IO and devices

    cv::namedWindow("Photo");   // Create a GUI window called photo

    Mat ramp_template;

    Mat frameHSV, thresholded, ramp_templateHSV, ramp_template_thresholded, frame_thresholded;

    ramp_template_thresholded = imread("/home/pi/image_processing/images/Symbols.bmp", CV_LOAD_IMAGE_UNCHANGED);
    //Mat frame = imread("/home/pi/image_processing/images/test.bmp", CV_LOAD_IMAGE_UNCHANGED);



    //cvtColor(ramp_template, ramp_templateHSV, COLOR_BGR2HSV); // Convert the image to HSV
    //inRange(ramp_templateHSV, Scalar(120, 0, 109), Scalar(179, 255, 212), ramp_template_thresholded);

    //-- Get the corners from the ramp_template ( the object to be "detected" )
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( ramp_template_thresholded.cols, 0 );
    obj_corners[2] = cvPoint( ramp_template_thresholded.cols, ramp_template_thresholded.rows );
    obj_corners[3] = cvPoint( 0, ramp_template_thresholded.rows );



    while(1)    // Main loop to perform image processing
    {
        Mat frame_thresholded = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
        //Mat frameGray, edges, frameHSV, thresholded, ramp_template_output;

        //cvtColor(frame, frameGray, COLOR_BGR2GRAY); // Convert the image to grayscale
        //cv::Canny(frameGray, edges, 100, 100*3, 3); // Highlight the edges with a min
        //threshold of 100

        printf("Hello");

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


        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows
        //}
        //  key = (key==255) ? -1 : key;    // Check if the esc key has been pressed
        //  if (key == 27)
        //    break;
    }

    //closeCV();  // Disable the camera and close any windows
    while(true);
    return 0;
}



