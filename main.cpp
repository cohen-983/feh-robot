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
DigitalEncoder LeftEncoder(FEHIO::P0_1);

FEHServo tinyServo(FEHServo::Servo0);


/***********************
 * Code to move Forward*
 * *********************/
void MoveForward(int powerPercent, float inches){
    rightMotor.SetPercent(powerPercent);
    leftMotor.SetPercent(powerPercent);

    //Calculate rotations needed to travel that distance
    rotations = inches/CIRCUMFRENCE;
    encoderTicks=rotations * 318;

    //Wait for Robot to traverse distance

    bool leftDone = false, rightDone = false;
    while (!leftDone || !rightDone){
        if (LeftEncoder.Counts()>= encoderTicks){
            leftDone = true;
            leftMotor.SetPercent(0);
        }
        if (rightEncoder.Counts()>=encoderTicks){
            rightDone = true;
            rightMotor.SetPercent(0);
        }
    }
}


int main(void)
{

    LCD.SetFontColor(FEHLCD::White);


/**********************************************************
    //TO MOVE TOWARDS THE POWERED WHEELS USE THIS FORMAT  *
    rightMotor.SetPercent(50);                            *
    leftMotor.SetPercent(-50);                            *
**********************************************************/


    LCD.WriteLine("Awesome");
    rightMotor.SetPercent(0);
    leftMotor.SetPercent(0);


    tinyServo.SetMin(540);
    tinyServo.SetMax(2469);


}

