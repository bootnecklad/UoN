#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <lcd.h>
#include <math.h>
#include <wiringSerial.h>
#include <errno.h>

#define TIME_OUT_VALUE 12000            //Number of microseconds to wait before timing out - sound travels 1m in ~3000 microseconds, must travel to and from object
#define DISTANCE_AVERAGE_NUMBER 5       //Number of readings to take averages from

#define TRIG 5          //Defining pin used for sending a pulse
#define ECHO 6          //Defining pin used for listening for a pulse

//USE WIRINGPI PIN NUMBERS
#define LCD_RS  25               //Register select pin
#define LCD_E   24               //Enable Pin
#define LCD_D4  23               //Data pin 4
#define LCD_D5  22               //Data pin 5
#define LCD_D6  21               //Data pin 6
#define LCD_D7  14               //Data pin 7

void setup()
{
    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    }
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    //TRIG pin must start LOW
    digitalWrite(TRIG, LOW);
    delay(30);
}

float getCM()       //Function to calculate the distance a single pulse has travelled in cm
{
    //Send trig pulse
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(20);
    digitalWrite(TRIG, LOW);

    //Wait for echo start
    long startTime = micros();
    while(digitalRead(ECHO) == LOW);
    {
        int travelTime = micros() - startTime;
        if(travelTime > TIME_OUT_VALUE)
        {
            return -1;
        }
    }

    long travelTime = micros() - startTime;

    //Wait for echo end
    startTime = micros();
    while(digitalRead(ECHO) == HIGH)
    {
        travelTime = micros() - startTime;
        if(travelTime > TIME_OUT_VALUE)
        {
            return -1;
        }
    }

    travelTime = micros() - startTime;

    //Get distance in cm
    float distance = (float)travelTime / 58;        //Sound takes ~29 microseconds to travel 1 cm, so dividing here by 58 accounts for the sound wave to be sent out and then travel back

    return distance;
}

float averageDistance()
{
    int i;

    float distance, unAveragedDistance;

    for(i = 0; i < DISTANCE_AVERAGE_NUMBER+1; i++)
    {
        do
        {
            unAveragedDistance = getCM();
        }
        while(unAveragedDistance == -1);                //If unAveragedDistance is -1 then time out has occurred - take another reading

        distance += unAveragedDistance;
    }

    return (float)distance / DISTANCE_AVERAGE_NUMBER;
}

int main(void)
{
    setup();

    int lcd, fd, count;

    lcd = lcdInit(2, 16, 4, LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7, 0, 0, 0, 0);
    lcdPuts(lcd, "Hello, world!");
    delay(1000);
    lcdClear(lcd);

    fd = serialOpen("/dev/ttyAMA0", 57600);

    if (fd < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }

    float distance;

    do
    {
        serialPrintf(fd, "#Bbff040,040");
        distance = averageDistance();
        lcdPosition(lcd, 0, 0);
        lcdPrintf(lcd, "%.2f", distance);
    }
    while(distance > 20);

    serialPrintf(fd, "#Hb");

    return 0 ;
}
