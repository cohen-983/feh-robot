#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <FEHBuzzer.h>
#include <FEHBattery.h>
#include <FEHSD.h>
#include <math.h>

#define PI 3.1415926
#define CIRCUMFRENCE 7.85
#define WHEEL_TO_WHEEL_WIDTH 7.5
//#define CORRECT_X 18.699
//#define CORRECT_Y 6.599
//#define CORRECT_H 0

FEHMotor rightMotor(FEHMotor::Motor0,9.0);
FEHMotor leftMotor(FEHMotor::Motor1,9.0);

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_1);

AnalogInputPin cds(FEHIO::P1_0);

FEHServo stickOfDestiny(FEHServo::Servo0);
FEHServo bigBoy(FEHServo::Servo7);
FEHServo coinArm(FEHServo::Servo5);

float zeroHeading, hCorrectionFactor;
float xLight, yLight, xCoinSlot, yCoinSlot;

void RPSCorrect();
//Get RPS to work on every course


void checkXPlus(float xCoord);
void checkYPlus(float yCoord);

float correctH(float angle);


void Move(int pow, float dis);
//Given an integer argument for power and a float argument for distance, moves the robot that far at that speed
void Turn(bool piv, int pow, float deg);
//Given an integer argument of 1 for a pivot or 0 for a standard turn, an integer argument for power, and a float argument for degrees to turn, turns the robot the specified number of degrees.
//Positive should turn clockwise
void runDiagnostics();
//Check that all motors and input ports are working correctly
void SansUndertale();
//Does what you'd expect
void PIDDrive(float distance, float expectedSpeed);
//Attempt at a PID driving system
void resetPIDVariables();
//Resets the vaiables
float rightPIDAdjustment(float expectedSpeed);
//Returns the corrected speed for the motors
float leftPIDAdjustment(float expectedSpeed);

void waitForLight(){
    coinArm.SetDegree(20);
    float startTime = TimeNow();//Take start time for timeout
    while(cds.Value()>.5){ //While the cds cell does not see the light
        if(TimeNow()-startTime > 60){//Timeout after 60 seconds
            break;
        }
    }
}

void checkYMinus(float yCoord);

void checkXMinus(float xCoord);

void checkHeading(float angle);

void TurnWithRPS(float angleToFace);

void start(){
    PIDDrive(.5,3);
    checkYPlus(yLight); //Check the y coordinate of the DDR light
    Turn(true,40,41);//Turn to the lights
    checkHeading(0); //Check heading to the lights

}

void moveToLight(){
    checkXPlus(xLight); //Move to the RPS X value of the light(should now be directly over the light)
    checkHeading(0);
}

void pressCorrectButton(){
    //Read Light
        if(cds.Value()<.45){//If light is red
            LCD.SetBackgroundColor(SCARLET);//Change screen to red
            Turn(true,40,85);//Turn to the button
            checkHeading(272);//Make sure the robot is facing the button
            Move(18,6);//Push button
            Move(15,-5);//Back away from button
            Turn(true,40,-85);//Turn to right wall
            checkHeading(0);//check that the robot is straight
            PIDDrive(5.2,5);//Move to base of ramp
        } else{//if light is blue or CdS doesnt see a light
            LCD.SetBackgroundColor(BLUE); //Set screen to blue
            PIDDrive(5.2,5);//Move to blue light
            Turn(true,40,85);//turn to button
            checkHeading(272);//face button
            Move(18,6);//push button
            Move(15,-5);//back away from button
            Turn(true,40,-85);//turn to right wall
            checkHeading(0);

        }
        PIDDrive(1.75,3);//Move closer to ramp
        Turn(true,40,-90);//turn to ramp
}

void moveUpRamp(){
    checkHeading(91);//Align to ramp
    PIDDrive(31.2,10);//Up ramp
    PIDDrive(12.8,5);//down steps
}

void moveToCoinSlot(){
    Turn(true,40,-140);//Turn to coin
    PIDDrive(5,7);//move out of dead zone
    checkYMinus(yCoinSlot);//Move to the appropriate y value
    Turn(true,35,45);//Perp to wall
    Sleep(100);
    checkHeading(180);
    checkXMinus(xCoinSlot);//move to coin dropping zone
    checkHeading(180);
}

void dropCoin(){
    coinArm.SetDegree(180);//Drop coin
    Sleep(2.0);
    coinArm.SetDegree(90);//reset arm
}

void flipLever(){
    Turn(true,40,15);//turn from coin slot
    PIDDrive(3,5);//move away from coin slot
    Turn(true,40,-15);//turn to left wall
    checkHeading(180);
    checkXMinus(4.5);//move to x value of lever
    Turn(true,40,-85); //Turn to lever
    checkHeading(268.5);
    checkYMinus(yCoinSlot+.776457135); //Check a referenced value for y
    PIDDrive(-12,8);//move to lever(entering dead zone)
    Turn(true,50,-35);//turn to lever
    //Move into wall
    rightMotor.SetPercent(-20);
    leftMotor.SetPercent(20);
    Sleep(1.5);
    leftMotor.Stop();
    rightMotor.Stop();
    dropCoin();//Drop the arm on the lever
}

void slideSlider(){
    coinArm.SetDegree(180);//drop arm onto sliders
    Sleep(1.0);//wait for arm to move
    PIDDrive(-8,4);//drive with the sliders
//    coinArm.SetDegree(150);//let go of sliders
//    PIDDrive(5,4);//move back to get a better grip on the sliders
//    coinArm.SetDegree(180);//regrip sliders
//    PIDDrive(-6.6,4);//move sliders the rest of the way
    coinArm.SetDegree(35);//let go of sliders
}

void moveToSlider(){
    Turn(true,20,5);
    PIDDrive(15.5,8);//drive away from lever
    Turn(true,40,80);//turn so back end of robot is facing the sliders
    checkHeading(230);
    PIDDrive(-14,8);//move in front of ramp
    Turn(true,40,-60);//turn so back is facing sliders
    PIDDrive(4,6);
    Turn(true,40,15);
    checkHeading(273);
    PIDDrive(-11.75,8);//drive to sliders
    Turn(true,40,-90);//turn facing the right wall
    /*
     * Turn moves the sliders, could be used to ush them all the way
     */
    //move into the wall
    rightMotor.SetPercent(20);
    leftMotor.SetPercent(-20);
    Sleep(3.0);
    leftMotor.Stop();
    rightMotor.Stop();


}

void goDownRamp(){
    Turn(true,40,25);//turn away from slider
    PIDDrive(3,5);//move from slider
    Turn(true,40,67);//turn towards ramp
    PIDDrive(10,5);//move towards ramp
    checkHeading(269);//check heading to ramp
    PIDDrive(14.5,10);//get up steps
    checkHeading(267);//turn angled down ramp
    PIDDrive(22,4);//get down ramp
}

void pushFinalButton(){
    Turn(true,40,60);//turn to button
    PIDDrive(1,5);
    checkHeading(202.5);//align with button
    PIDDrive(500,15);//full speed into button
}




#define INITIAL_TURN_ANGLE 90
#define ALIGN_WITH_LEVER_ANGLE 20
#define DISTANCE_TO_RAMP 0
#define DISTANCE_TO_LEVEL 0
#define PCONST .75
#define ICONST .1
#define DCONST .25

float lPreviousTime,rPreviousTime;
int rightPreviousCounts,leftPreviousCounts, leftSign, rightSign;
float rightActualVelocity, leftActualVelocity;
float expectedVelocity;
float lPTerm,lITerm,lDTerm,rPTerm,rITerm,rDTerm;
float leftPreviousError,rightPreviousError;
float rErrorSum,lErrorSum;
float rOldMotorPower,lOldMotorPower;


int main(void)
{
    float x,y;
        stickOfDestiny.SetMin(540);
        stickOfDestiny.SetMax(2470);
        stickOfDestiny.SetDegree(90);

        bigBoy.SetMin(510);
        bigBoy.SetMax(2500);
        LCD.WriteLine(Battery.Voltage());

        coinArm.SetMin(520);
        coinArm.SetMax(1955);

        coinArm.SetDegree(35);

        RPS.InitializeTouchMenu();

        RPSCorrect();

        waitForLight();

        start();

        moveToLight();

        pressCorrectButton();

        moveUpRamp();

        moveToCoinSlot();

        dropCoin();

        flipLever();

        moveToSlider();

        slideSlider();

        goDownRamp();

        pushFinalButton();


}

/*
    Code to move robot in a straight line
*/
void Move(int percent, float distance)
{
    float startTime=TimeNow();
    int powerPercent = percent;
    float distanceToMove = distance;
    bool negativeDistance = false,rightDone=false,leftDone=false;
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

        if(rightCounts > leftCounts + 5)
        {
            rightMotor.SetPercent((rightCoeff*powerPercent)*rightCountsRatio);
            leftMotor.SetPercent(leftCoeff*powerPercent);
            //LCD.WriteLine("Turn right");
        }

        else if(leftCounts > rightCounts + 5)
        {
            rightMotor.SetPercent(rightCoeff*powerPercent);
            leftMotor.SetPercent((leftCoeff*powerPercent)*leftCountsRatio);
            //LCD.WriteLine("Turn Left");
        }

        else if(leftCounts == rightCounts)
        {
            rightMotor.SetPercent(rightCoeff*powerPercent);
            leftMotor.SetPercent(leftCoeff*powerPercent);
        }

        /***************************************
         * Trying to eliminate end variablility*
         * *************************************/
        if(rightCounts >= encoderTicks){
            rightMotor.Stop();
            rightDone=true;
        }
        if(leftCounts >= encoderTicks){
            leftMotor.Stop();
            leftDone=true;
        }

        if(leftDone && rightDone){
            moveComplete=true;
        }
        if(TimeNow()-startTime >= 6.5){
            moveComplete=true;
        }

    }
    leftMotor.Stop();
    rightMotor.Stop();
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
    bool leftDone=false,rightDone=false;
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
    float startTime = TimeNow();

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
            if(rightCounts >= encoderTicks){
                rightMotor.Stop();
                rightDone=true;
            }
            if (leftCounts >= encoderTicks){
                leftMotor.Stop();
                leftDone=true;
            }
            if(TimeNow()-startTime > 7.5){
                leftMotor.Stop();
                rightMotor.Stop();
                moveComplete=true;
            }


            if(leftDone && rightDone){

                //Ensure turn is accurate
                LCD.Write("Right Encoder Counts: " );
                LCD.WriteLine(rightEncoder.Counts());
                LCD.Write("Left Encoder Counts: ");
                LCD.WriteLine(leftEncoder.Counts());
                //Testing code

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
            leftMotor.SetPercent(-leftCoeff*powerPercent);
            rightMotor.Stop(); //just to make like, triple sure.
            while(moveComplete == false)
            {
                leftCounts = leftEncoder.Counts();
                if(leftCounts >= encoderTicks)
                {
                    leftMotor.Stop();
                    moveComplete = true;
                }
                if(TimeNow()-startTime > 7.5){
                    leftMotor.Stop();
                    rightMotor.Stop();
                    moveComplete=true;
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
                rightCounts = rightEncoder.Counts();
                if(rightCounts >= encoderTicks)
                {
                    rightMotor.Stop();
                    moveComplete = true;
                }
                if(TimeNow()-startTime > 7.5){
                    leftMotor.Stop();
                    rightMotor.Stop();
                    moveComplete=true;
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
    leftMotor.SetPercent(-15);
    Sleep(1.0);
    leftMotor.Stop();
    LCD.Write("Right Encoder Value: ");
    LCD.WriteLine(rightEncoder.Counts());
    LCD.Write("Left Encoder Value (should be >0): ");
    LCD.WriteLine(leftEncoder.Counts());
    Sleep(5.0);
    LCD.Clear();

    //Test coin arm
    coinArm.SetDegree(180);
    Sleep(2.0);
    coinArm.SetDegree(20);

    //Test CdS cell
    float startTime = TimeNow();
    while(TimeNow()-startTime<5){
        LCD.WriteLine(cds.Value());
        Sleep(300);
        LCD.Clear();
    }
}

void SansUndertale()
{
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::D5, 125);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::C5, 250);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::A4, 250);
    Buzzer.Tone(FEHBuzzer::Af4, 250);
    Buzzer.Tone(FEHBuzzer::G4, 250);
    Buzzer.Tone(FEHBuzzer::F4, 250);
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::F4, 125);
    Buzzer.Tone(FEHBuzzer::G4, 125);
    //Dear god, this is a lot of work, that was one measure.
    Buzzer.Tone(FEHBuzzer::C4, 125);
    Buzzer.Tone(FEHBuzzer::C4, 125);
    Buzzer.Tone(FEHBuzzer::D5, 125);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::C5, 250);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::A4, 250);
    Buzzer.Tone(FEHBuzzer::Af4, 250);
    Buzzer.Tone(FEHBuzzer::G4, 250);
    Buzzer.Tone(FEHBuzzer::F4, 250);
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::F4, 125);
    Buzzer.Tone(FEHBuzzer::G4, 125);

    Buzzer.Tone(FEHBuzzer::B3, 125);
    Buzzer.Tone(FEHBuzzer::B3, 125);
    Buzzer.Tone(FEHBuzzer::D5, 125);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::C5, 250);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::A4, 250);
    Buzzer.Tone(FEHBuzzer::Af4, 250);
    Buzzer.Tone(FEHBuzzer::G4, 250);
    Buzzer.Tone(FEHBuzzer::F4, 250);
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::F4, 125);
    Buzzer.Tone(FEHBuzzer::G4, 125);

    Buzzer.Tone(FEHBuzzer::Bf3, 125);
    Buzzer.Tone(FEHBuzzer::Bf3, 125);
    Buzzer.Tone(FEHBuzzer::D5, 125);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::C5, 250);
    Sleep(125);
    Buzzer.Tone(FEHBuzzer::A4, 250);
    Buzzer.Tone(FEHBuzzer::Af4, 250);
    Buzzer.Tone(FEHBuzzer::G4, 250);
    Buzzer.Tone(FEHBuzzer::F4, 250);
    Buzzer.Tone(FEHBuzzer::D4, 125);
    Buzzer.Tone(FEHBuzzer::F4, 125);
    Buzzer.Tone(FEHBuzzer::G4, 125);

    Sleep(500);

    Buzzer.Tone(FEHBuzzer::F4, 149);
    Buzzer.Tone(FEHBuzzer::Gs4, 149);
    Buzzer.Tone(FEHBuzzer::As4, 149);
    Buzzer.Tone(FEHBuzzer::As4, 447);
    Buzzer.Tone(FEHBuzzer::Gs4, 298);
    Sleep(596);

    Sleep(596);
    Buzzer.Tone(FEHBuzzer::F4, 149);
    Buzzer.Tone(FEHBuzzer::Gs4, 149);
    Buzzer.Tone(FEHBuzzer::As4, 149);
    Buzzer.Tone(FEHBuzzer::As4, 447);
    Buzzer.Tone(FEHBuzzer::Gs4, 298);
    Buzzer.Tone(FEHBuzzer::F4, 298);
    Buzzer.Tone(FEHBuzzer::Ef4, 149);
    Buzzer.Tone(FEHBuzzer::F4, 149);

}

void PIDDrive(float distance, float expectedSpeed){ //Drive function that matches wheel speeds
    resetPIDVariables();
    if(distance<0){//Allows PID to go negative distances
        distance=-distance;//reset distance sign
        //make both motors go backwards
        rightSign=-rightSign;
        leftSign=-leftSign;
    }
    float startTime = TimeNow();

    while(((leftEncoder.Counts() / 318.0) * CIRCUMFRENCE) < distance || ((rightEncoder.Counts() / 318.0) * CIRCUMFRENCE) < distance){
        rightMotor.SetPercent(rightSign * rightPIDAdjustment(expectedSpeed)); //dynamically adjust motor percents
        leftMotor.SetPercent(leftSign * leftPIDAdjustment(expectedSpeed));
        LCD.Clear();
        if(((rightEncoder.Counts() / 318.0) * CIRCUMFRENCE) >= distance){//when right wheel has travelled the distance
            rightMotor.Stop();//stop
        }
        if(((leftEncoder.Counts() / 318.0) * CIRCUMFRENCE) >= distance){//when left wheel has covered distance
            leftMotor.Stop();//stop
        }
        if(TimeNow()-startTime>10){
            break;
        }
    }
    //just in case
    leftMotor.Stop();
    rightMotor.Stop();
}


float rightPIDAdjustment(float expectedSpeed){
    int currentCounts=rightEncoder.Counts();
    int countDiff=currentCounts-rightPreviousCounts;
    float currentTime = TimeNow();
    float timeDiff = currentTime-rPreviousTime;
    rightActualVelocity=(CIRCUMFRENCE/318)*(countDiff/timeDiff);//calculate current velocity
    float currentError = expectedSpeed - rightActualVelocity;//calculate error
    rErrorSum+=currentError;//add to cumulative error
    //calculate three terms for motor speed
    rPTerm = currentError * PCONST;
    rITerm = rErrorSum * ICONST;
    rDTerm = (currentError - rightPreviousError) * DCONST;
    //assign current values as previous for use in next loop
    rPreviousTime=currentTime;
    rightPreviousError=currentError;
    rightPreviousCounts=currentCounts;

    //update motor power
    rOldMotorPower += (rPTerm+rITerm+rDTerm);

    LCD.WriteLine(currentCounts);
    return rOldMotorPower;

}

float leftPIDAdjustment(float expectedSpeed){
    //Assign current values
    int currentCounts=leftEncoder.Counts();
    int countDiff=currentCounts-leftPreviousCounts;
    float currentTime = TimeNow();
    float timeDiff = currentTime-lPreviousTime;
    leftActualVelocity=(CIRCUMFRENCE/318)*(countDiff/timeDiff); //calculate current velocity
    float currentError = expectedSpeed - leftActualVelocity; //take difference in velocities for error
    lErrorSum+=currentError;//add error to error sum
    //calculate the three terms
    lPTerm = currentError * PCONST;
    lITerm = lErrorSum * ICONST;
    lDTerm = (currentError - leftPreviousError) * DCONST;
    //save values for use on the next loop
    lPreviousTime=currentTime;
    leftPreviousError=currentError;
    leftPreviousCounts=currentCounts;
    //update motor percentage
    lOldMotorPower += (lPTerm+lITerm+lDTerm);
    LCD.WriteLine(currentCounts);
    return lOldMotorPower;
}

void resetPIDVariables(){
    //set all variables to their initial state
    leftEncoder.ResetCounts();
    rightEncoder.ResetCounts();
    rPreviousTime=0;
    lPreviousTime=0;
    rightPreviousCounts = 0;
    leftPreviousCounts=0;
    rightActualVelocity=0;
    leftActualVelocity=0;
    expectedVelocity=0;
    rErrorSum=0;
    lErrorSum=0;
    lPTerm=0;
    lITerm=0;
    lDTerm=0;
    rPTerm=0;
    rITerm=0;
    rDTerm=0;
    leftPreviousError=0;
    rightPreviousError=0;
    rOldMotorPower=5;
    lOldMotorPower=5;
    leftSign=-1;
    rightSign=1;
}

void checkHeading(float heading){

    float currentHeading;
    heading = correctH(heading); //Correct the heading
    currentHeading= RPS.Heading();//Get current heading
    if(currentHeading>=0){ //If not negative
        float startTime=TimeNow(); //take start time for timeout
            while(currentHeading > heading + .3 || currentHeading < heading - .3){ //.6 degree error
                Sleep(100);
                LCD.Clear();
                currentHeading= RPS.Heading(); //check heading
                LCD.Write("Heading: ");
                LCD.WriteLine(currentHeading);
                LCD.Write("Expected Heading: ");
                LCD.WriteLine(heading);

                if(currentHeading<0){
                    PIDDrive(.25,3);
                }
                else if(abs(currentHeading-heading) <= 180){ //if heading is greater than the range
                    if(abs(currentHeading-heading) >= 20){
                        Turn(true,20,.9*(currentHeading-heading));
                    }
                    else if(currentHeading>heading){
                        leftMotor.SetPercent(-15); //turn right
                        Sleep(50);//for 10 ms
                        leftMotor.Stop();
                    } else{
                        leftMotor.SetPercent(15); //turn left
                        Sleep(50);//for 10 ms
                        leftMotor.Stop();
                    }
                } else{//if heading difference is greater than 180
                    if((currentHeading-heading)-360 >= 10){
                        Turn(true,20,(.9*(currentHeading-heading)-360));

                    }
                    else if((currentHeading-heading)+360 <= -10){
                        Turn(true,20,(.9*(currentHeading-heading)+360));

                    }
                    else if(currentHeading>heading){
                        leftMotor.SetPercent(15); //turn left
                        Sleep(50);//for 50 ms
                        leftMotor.Stop();
                    } else{
                        leftMotor.SetPercent(-15); //turn right
                        Sleep(50);//for 50 ms
                        leftMotor.Stop();
                    }
                }
                if(TimeNow()-startTime > 10){//timeout
                    break;
                }

            }

    }else{
        LCD.SetBackgroundColor(RED); //Set background to red if -1 or -2 was checked before while loop
        LCD.Clear();
        Sleep(10);
        Move(10,.25);
        checkHeading(heading); //retry
    }

}

void checkYMinus(float yCoord) //using RPS while robot is in the -y direction
{

    //check whether the robot is within an acceptable range
    while(RPS.Y() < yCoord - .1 || RPS.Y() > yCoord + .1)
    {
        LCD.Write("Expected Y: ");
        LCD.WriteLine(yCoord);
        LCD.Write("Current Y: ");
        LCD.WriteLine(RPS.Y());
        Sleep(100);
        LCD.Clear();
        if(abs(RPS.Y()-yCoord)>1){
            PIDDrive((RPS.Y()-yCoord),5);
        }
        else if(RPS.Y() > yCoord)
        {
            //pulse the motors for a short duration in the correct direction
            rightMotor.SetPercent(10);
            leftMotor.SetPercent(-10);
            Sleep(50);
            rightMotor.Stop();
            leftMotor.Stop();
        }
        else if(RPS.Y() < yCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(-10);
            leftMotor.SetPercent(10);
            Sleep(50);
            rightMotor.Stop();
            leftMotor.Stop();
        }
    }
}

void checkYPlus(float yCoord) //using RPS while robot is in the +y direction
{

    //check whether the robot is within an acceptable range
    while(RPS.Y() < yCoord - .1 || RPS.Y() > yCoord + .1)
    {
        LCD.Write("Expected Y: ");
        LCD.WriteLine(yCoord);
        LCD.Write("Current Y: ");
        LCD.WriteLine(RPS.Y());
        Sleep(100);
        LCD.Clear();
        if(abs(RPS.Y()-yCoord)>1){
            PIDDrive(-(RPS.Y()-yCoord),5);
        }
        if(RPS.Y() > yCoord)
        {
            //pulse the motors for a short duration in the correct direction
            rightMotor.SetPercent(-10);
            leftMotor.SetPercent(10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();

        }
        else if(RPS.Y() < yCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(10);
            leftMotor.SetPercent(-10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();
        }
    }
}

void checkXMinus(float xCoord) //using RPS while robot is in the +x direction
{

    //check whether the robot is within an acceptable range
    while(RPS.X() < xCoord - .2 || RPS.X() > xCoord + .2)
    {
        LCD.Write("Expected X: ");
        LCD.WriteLine(xCoord);
        LCD.Write("Current X: ");
        LCD.WriteLine(RPS.X());
        Sleep(100);
        LCD.Clear();
        if(abs(RPS.X()-xCoord)>1){
            PIDDrive((RPS.X()-xCoord),5);
        }
        else if(RPS.X() > xCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(10);
            leftMotor.SetPercent(-10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();
        }
        else if(RPS.X() < xCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(-10);
            leftMotor.SetPercent(10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();
        }
    }
}

void checkXPlus(float xCoord) //using RPS while robot is in the +x direction
{

    //check whether the robot is within an acceptable range
    while(RPS.X() < xCoord - .1 || RPS.X() > xCoord + .1)
    {
        LCD.Write("Expected X: ");
        LCD.WriteLine(xCoord);
        LCD.Write("Current X: ");
        LCD.WriteLine(RPS.X());
        Sleep(100);
        LCD.Clear();
        if(xCoord>0){
            if(abs(RPS.X()-xCoord)>2){
                PIDDrive(-(RPS.X()-xCoord),5);
            }
        }
        if(RPS.X() > xCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(-10);
            leftMotor.SetPercent(10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();
        }
        else if(RPS.X() < xCoord)
        {
            //pulse the motors for a short duration in the correct direction

            rightMotor.SetPercent(10);
            leftMotor.SetPercent(-10);
            Sleep(70);
            rightMotor.Stop();
            leftMotor.Stop();
        }
    }
}

void RPSCorrect(){ //Take course specific RPS values to use in code

    LCD.Clear();
    float x,y;
    LCD.WriteLine("Place on DDR Light");

    //wait for touch
    while(LCD.Touch(&x,&y));
    while(!LCD.Touch(&x,&y));
    Sleep(500);
    
    do{//take rps values until the values are positive
        xLight=RPS.X();
        yLight=RPS.Y();
        Sleep(200);
    }while (xLight < 0 || yLight < 0);

    //display values
    LCD.WriteLine(xLight);
    LCD.WriteLine(yLight);

    LCD.WriteLine("Place at heading 0");

    //wait for touch
    while(LCD.Touch(&x,&y));
    while(!LCD.Touch(&x,&y));
    Sleep(500);


    do{//take rps values until the values are positive
        zeroHeading = RPS.Heading();
        hCorrectionFactor = zeroHeading;
        Sleep(200);
    }while (zeroHeading < 0);
    LCD.WriteLine(zeroHeading);

    LCD.WriteLine("Place at coin Slot");

    //wait for touch
    while(LCD.Touch(&x,&y));
    while(!LCD.Touch(&x,&y));
    Sleep(500);

    do{//take rps values until the values are positive
        xCoinSlot = RPS.X();
        yCoinSlot =RPS.Y();
        Sleep(10);
    }while (xCoinSlot < 0 || yCoinSlot < 0);

    LCD.WriteLine(xCoinSlot);
    LCD.WriteLine(yCoinSlot);
    Sleep(1.0);
    LCD.Clear();

}

float correctH(float angle){//adjust angle based on the zero angle aquired in RPSCorrect
    float correctAngle = angle + hCorrectionFactor;
    if(correctAngle>360){
        correctAngle-=360;
    }
    return correctAngle;
}
