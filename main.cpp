#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>

#define CIRCUMFRENCE 7.853975

FEHMotor rightMotor(FEHMotor::Motor0,9.0);
FEHMotor leftMotor(FEHMotor::Motor1,9.0);

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_1);

FEHServo tinyServo(FEHServo::Servo0);


void Move(int x, float y);
//Given an integer argument for power and a float argument for distance, moves the robot that far at that speed



int main(void)
{

    LCD.SetFontColor(FEHLCD::White);

    tinyServo.SetMin(540);
    tinyServo.SetMax(2469);
/**********************************************************
    //TO MOVE TOWARDS THE POWERED WHEELS USE THIS FORMAT  *
    rightMotor.SetPercent(50);                            *
    leftMotor.SetPercent(-50);                            *
**********************************************************/


    LCD.WriteLine("Awesome");
    rightMotor.Stop();
    leftMotor.Stop();




}

/*
    Code to move robot in a straight line
*/
void Move(int x, float y)
{
    int powerPercent = x;
    float distanceToMove = y;
    bool negativeDistance = false;
    float rotations = 0;
    int encoderTicks = 0;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    if(distanceToMove < 0) //throw a flag for a negative distance
        negativeDistance = true;

    //Calculate rotations needed to travel that distance
    if(negativeDistance == false)
    {
        rotations = distanceToMove/CIRCUMFRENCE;
        encoderTicks = rotations * 318;
    }
    else if(negativeDistance == true)
    {
        rotations = -1*distanceToMove/CIRCUMFRENCE;
        encoderTicks = rotations * 318;
    }

    //Move the robot that number of rotations, balancing speed as it goes
    bool moveComplete = false;
    
    int leftCoeff = -1; //add in the negative bit for the left motor
    int rightCoeff = 1;
    int rightCounts = 0;
    int leftCounts = 0;
    float rightCountsRatio = 0;
    float leftCountsRatio = 0;

    if(negativeDistance == true) //flip direction for a negative move
    {
        leftCoeff = 1;
        rightCoeff = -1;
    }

    rightMotor.SetPercent(rightCoeff*powerPercent);
    leftMotor.SetPercent(leftCoeff*powerPercent);

    while(moveComplete == false)
    {
        leftCounts = leftEncoder.Counts();
        rightCounts = rightEncoder.Counts();
        rightCountsRatio = leftCounts / rightCounts; //used to determine magnitude of speed cut if one motor is moving faster, roughly
        leftCountsRatio = rightCounts / leftCounts;

        if(rightCounts > leftCounts)
        {
            rightMotor.SetPercent((rightCoeff*powerPercent)*rightCountsRatio);
            leftMotor.SetPercent(leftCoeff*powerPercent);
        }

        else if(leftCounts < rightCounts)
        {
            rightMotor.SetPercent(rightCoeff*powerPercent);
            leftMotor.SetPercent((leftCoeff*powerPercent)*leftCountsRatio);
        }

        else if(leftCounts == rightCounts)
        {
            rightMotor.SetPercent(rightCoeff*powerPercent);
            leftMotor.SetPercent(leftCoeff*powerPercent);
        }

        if(rightCounts >= encoderTicks && leftCounts >= encoderTicks)
        // if either motor is past the threshold, stop the move and balance it out at the finish
        {
            rightMotor.Stop(); //stop them
            leftMotor.Stop();
            while(rightCounts != leftCounts) // then verify that they went the same distance
            {
                leftCounts = leftEncoder.Counts();
                rightCounts = rightEncoder.Counts();

                if(rightCounts > leftCounts) // if not, balance it out
                {
                    leftMotor.SetPercent(leftCoeff*powerPercent);
                }
                else if(leftCounts > rightCounts)
                {
                    rightMotor.SetPercent(rightCoeff*powerPercent);
                }
            }
            rightEncoder.ResetCounts();
            leftEncoder.ResetCounts();
            moveComplete = true;
        }
    }
}
