#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>

#define PI 3.1415926
#define CIRCUMFRENCE 7.853975
#define WHEEL_TO_WHEEL_WIDTH 8.375

FEHMotor rightMotor(FEHMotor::Motor0,9.0);
FEHMotor leftMotor(FEHMotor::Motor1,9.0);

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_1);

AnalogInputPin cds(FEHIO::P1_0);

FEHServo tinyServo(FEHServo::Servo0);


void Move(int pow, float dis);
//Given an integer argument for power and a float argument for distance, moves the robot that far at that speed
void Turn(bool piv, int pow, float deg);
//Given an integer argument of 1 for a pivot or 0 for a standard turn, an integer argument for power, and a float argument for degrees to turn, turns the robot the specified number of degrees.
//Positive should turn clockwise
void runDiagnostics();
//Check that all motors and input ports are working correctly
void SansUndertale();
//Does what you'd expect



int main(void)
{

    SansUndertale();
    Move(15,12);
    Turn(true,25,90);

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
    {
        negativeDistance = true;
        distanceToMove = distanceToMove  * -1;
    }


    //Calculate rotations needed to travel that distance
    rotations = distanceToMove/CIRCUMFRENCE;
    encoderTicks = rotations * 318;

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

        if(rightCounts >= encoderTicks || leftCounts >= encoderTicks)
        // if either motor is past the threshold, stop the move and balance it out at the finish
        {
            rightMotor.Stop(); //stop them
            leftMotor.Stop();
            while(rightCounts != leftCounts) // then verify that they went the same distance
            {
                leftCounts = leftEncoder.Counts();
                LCD.WriteLine(leftCounts); //TESTING
                rightCounts = rightEncoder.Counts();
                LCD.WriteLine(rightCounts);//TESTING

                if(rightCounts > leftCounts) // if not, balance it out
                {
                    leftMotor.SetPercent(leftCoeff*powerPercent);
                    rightMotor.Stop();
                }
                else if(leftCounts > rightCounts)
                {
                    rightMotor.SetPercent(rightCoeff*powerPercent);
                    leftMotor.Stop();
                }
            }
            rightMotor.Stop();
            leftMotor.Stop();
            rightEncoder.ResetCounts();
            leftEncoder.ResetCounts();
            moveComplete = true;
        }
    }
}

void Turn(bool t, int x, float y)
{
    int powerPercent = x;
    float angleToTurn = y;
    float distanceToMove = 0;
    bool negativeAngle = false;
    bool pivot = t;
    float rotations = 0;
    int encoderTicks = 0;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    if(angleToTurn < 0) //throw a flag for a negative distance
    {
        negativeAngle = true;
        angleToTurn = angleToTurn * -1;
    }

    if(pivot == true) //calculate the distance using a circle either centered in the middle of the axle for a pivot
        distanceToMove = (WHEEL_TO_WHEEL_WIDTH*PI) * (angleToTurn/360);
    else if(pivot == false) //or at one end of it for the other method
        distanceToMove = (2*WHEEL_TO_WHEEL_WIDTH*PI) * (angleToTurn/360);

    //Calculate rotations needed to travel that distance
    rotations = distanceToMove/CIRCUMFRENCE;
    encoderTicks = rotations * 318;

    //Choose between the two types of turns, which is a bit of a chore given the 4 seperate possible types.
    bool moveComplete = false;
    
    int leftCoeff = -1; //add in the negative bit for the left motor
    int rightCoeff = 1;
    int rightCounts = 0;
    int leftCounts = 0;
    float rightCountsRatio = 0;
    float leftCountsRatio = 0;

    if(negativeAngle == true) //flip coefficients for a negative move
    {
        leftCoeff = 1;
        rightCoeff = -1;
    }

    if(pivot == true)
    {
        rightMotor.SetPercent(-1*rightCoeff*powerPercent); //assumes a right turn, because the negative coefficients make it left if need be
        leftMotor.SetPercent(leftCoeff*powerPercent);   
        while(moveComplete == false)
        {
            leftCounts = leftEncoder.Counts();
            rightCounts = rightEncoder.Counts();
            rightCountsRatio = leftCounts / rightCounts; //used to determine magnitude of speed cut if one motor is moving faster, roughly speaking
            leftCountsRatio = rightCounts / leftCounts;

            if(rightCounts > leftCounts)
            {
                rightMotor.SetPercent((-1*rightCoeff*powerPercent)*rightCountsRatio); //driving the right motor backwards causes the robot to pivot right if the inital direction was forward, and left if it was backwards.
                leftMotor.SetPercent(leftCoeff*powerPercent);
            }

            else if(leftCounts < rightCounts)
            {
                rightMotor.SetPercent(-1*rightCoeff*powerPercent); //so that's what goes down in all of these.
                leftMotor.SetPercent((leftCoeff*powerPercent)*leftCountsRatio);
            }

            else if(leftCounts == rightCounts)
            {
                rightMotor.SetPercent(-1*rightCoeff*powerPercent); //rather than just updating the coefficient because reasons
                leftMotor.SetPercent(leftCoeff*powerPercent); //look, I coded it, you can recode it if you've got problems with it. Just make a seperate branch first, I don't want to have to revert broken nonsense because we pissed off the C++ gods.
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
                        rightMotor.Stop();
                    }
                    else if(leftCounts > rightCounts)
                    {
                        rightMotor.SetPercent(-1*rightCoeff*powerPercent);
                        leftMotor.Stop();
                    }
                }
                rightMotor.Stop(); //actually stop them, suspiciously missing in earlier revisions.
                leftMotor.Stop(); //I say, like it wasn't my fault in those revisions.
                rightEncoder.ResetCounts();
                leftEncoder.ResetCounts();
                moveComplete = true;
            }
        }
    }
    else if(pivot == false) //other kinds of turns, wooo
    { //fortunately with only one wheel moving here, the speed balancing code is irrelevant. Knock on digital wood.
        if(negativeAngle == false)
        {
            leftMotor.SetPercent(leftCoeff*powerPercent);
            rightMotor.Stop(); //just to make like, triple sure.
            while(moveComplete == false)
            {
                leftCounts = leftEncoder.Counts();
                if(leftCounts >= encoderTicks)
                {
                    leftMotor.Stop();
                    moveComplete = true;
                }
            }
        }
        else if(negativeAngle == true)
        {
            rightMotor.SetPercent(-1*rightCoeff*powerPercent); //it's not a perfect system, don't judge.
            //essentially, cribbing the code from the move forward function solved a bunch of basic issues, but the coefficient stuff isn't great when a "negative" turn is still a forward move, sorta.
            leftMotor.Stop(); //makin sure
            while(moveComplete == false)
            {
                rightCounts = leftEncoder.Counts();
                if(rightCounts >= encoderTicks)
                {
                    rightMotor.Stop();
                    moveComplete = true;
                }
            }
        }
    }
}

void runDiagnostics(){
    LCD.Clear();
    //Check that motors and encoders work
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    //Run right motor
    LCD.WriteLine("Running right Motor");
    rightMotor.SetPercent(15);
    Sleep(1.0);
    rightMotor.Stop();
    LCD.Write("Right Encoder Value (should be >0): ");
    LCD.WriteLine(rightEncoder.Counts());
    LCD.Write("Left Encoder Value: ");
    LCD.WriteLine(leftEncoder.Counts());
    Sleep(5.0);
    LCD.Clear();

    //Run Left motor test
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();

    LCD.WriteLine("Running left Motor");
    leftMotor.SetPercent(15);
    Sleep(1.0);
    leftMotor.Stop();
    LCD.Write("Right Encoder Value: ");
    LCD.WriteLine(rightEncoder.Counts());
    LCD.Write("Left Encoder Value (should be >0): ");
    LCD.WriteLine(leftEncoder.Counts());
    Sleep(5.0);
    LCD.Clear();

    //Future Additions:
    //Servos
    //CdS cell
}

void SansUndertale
{
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(D5, 125);
    Sleep(125);
    Buzzer.Tone(C5, 250);
    Sleep(125);
    Buzzer.Tone(A4, 250);
    Buzzer.Tone(Af4, 250);
    Buzzer.Tone(G4, 250);
    Buzzer.Tone(F4, 250);
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(F4, 125);
    Buzzer.Tone(G4, 125);
    //Dear god, this is a lot of work, that was one measure.
    Buzzer.Tone(C4, 125);
    Buzzer.Tone(C4, 125);
    Buzzer.Tone(D5, 125);
    Sleep(125);
    Buzzer.Tone(C5, 250);
    Sleep(125);
    Buzzer.Tone(A4, 250);
    Buzzer.Tone(Af4, 250);
    Buzzer.Tone(G4, 250);
    Buzzer.Tone(F4, 250);
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(F4, 125);
    Buzzer.Tone(G4, 125);

    Buzzer.Tone(B3, 125);
    Buzzer.Tone(B3, 125);
    Buzzer.Tone(D5, 125);
    Sleep(125);
    Buzzer.Tone(C5, 250);
    Sleep(125);
    Buzzer.Tone(A4, 250);
    Buzzer.Tone(Af4, 250);
    Buzzer.Tone(G4, 250);
    Buzzer.Tone(F4, 250);
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(F4, 125);
    Buzzer.Tone(G4, 125);

    Buzzer.Tone(Bf3, 125);
    Buzzer.Tone(Bf3, 125);
    Buzzer.Tone(D5, 125);
    Sleep(125);
    Buzzer.Tone(C5, 250);
    Sleep(125);
    Buzzer.Tone(A4, 250);
    Buzzer.Tone(Af4, 250);
    Buzzer.Tone(G4, 250);
    Buzzer.Tone(F4, 250);
    Buzzer.Tone(D4, 125);
    Buzzer.Tone(F4, 125);
    Buzzer.Tone(G4, 125);

    Sleep(500);

    Buzzer.Tone(F4, 149);
    Buzzer.Tone(Gs4, 149);
    Buzzer.Tone(As4, 149);
    Buzzer.Tone(As4, 447);
    Buzzer.Tone(Gs4, 298);
    Sleep(596);

    Sleep(596);
    Buzzer.Tone(F4, 149);
    Buzzer.Tone(Gs4, 149);
    Buzzer.Tone(As4, 149);
    Buzzer.Tone(As4, 447);
    Buzzer.Tone(Gs4, 298);
    Buzzer.Tone(F4, 298);
    Buzzer.Tone(Ef4, 149);
    Buzzer.Tone(F4, 149);

}