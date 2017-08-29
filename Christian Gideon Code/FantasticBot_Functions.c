//These are variables that will be used in functions of "Fantasticbot"
int off = 0;

//These are functions for the Robot Virtual World "Fantasticbot" to use.

//Moves the robot forward for a certaim amount of time
void moveForwardForTime(float forwardSpeed, float forwardTime)
{
	motor[frontLeftMotor] = forwardSpeed;
	motor[frontRightMotor] = forwardSpeed;
	motor[rearLeftMotor] = forwardSpeed;
	motor[rearRightMotor] = forwardSpeed;
	wait1Msec(forwardTime*1000);
	motor[frontLeftMotor] = off;
	motor[frontRightMotor] = off;
	motor[rearLeftMotor] = off;
	motor[rearRightMotor] = off;
}

//Moves the robot backward for a certaim amount of time
void moveBackwardForTime(float backwardSpeed, float backwardTime)
{
	motor[frontLeftMotor] = -backwardSpeed;
	motor[frontRightMotor] = -backwardSpeed;
	motor[rearLeftMotor] = -backwardSpeed;
	motor[rearRightMotor] = -backwardSpeed;
	wait1Msec(backwardTime*1000);
	motor[frontLeftMotor] = off;
	motor[frontRightMotor] = off;
	motor[rearLeftMotor] = off;
	motor[rearRightMotor] = off;
}

//Turns the robot left for a certain amount of time
void turnLeftForTime(float turnLeftSpeed, float turnLeftTime)
{
	motor[frontLeftMotor] = -turnLeftSpeed;
	motor[frontRightMotor] = turnLeftSpeed;
	motor[rearLeftMotor] = -turnLeftSpeed;
	motor[rearRightMotor] = turnLeftSpeed;
	wait1Msec(turnLeftTime*1000);
	motor[frontLeftMotor] = off;
	motor[frontRightMotor] = off;
	motor[rearLeftMotor] = off;
	motor[rearRightMotor] = off;
}

//Turns the robot right for a certain amount of time
void turnRightForTime(float turnRightSpeed, float turnRightTime)
{
	motor[frontLeftMotor] = turnRightSpeed;
	motor[frontRightMotor] = -turnRightSpeed;
	motor[rearLeftMotor] = turnRightSpeed;
	motor[rearRightMotor] = -turnRightSpeed;
	wait1Msec(turnRightTime*1000);
	motor[frontLeftMotor] = off;
	motor[frontRightMotor] = off;
	motor[rearLeftMotor] = off;
	motor[rearRightMotor] = off;
}

//Opens the claw to grab a cone or mobile goal
void openClaw()
{
	motor[clawMotor] = -127;
	wait1Msec(500);
	motor[clawMotor] = off;
}

//Closes the claw to grab a cone or mobile goal
void closeClaw()
{
	motor[clawMotor] = 127;
	wait1Msec(500);
	motor[clawMotor] = off;
}

//Moves the wrist of the claw down for a certain time
void moveWristDownForTime(float wristDSpeed, float wristDTime)
{
	motor[wristMotor] = -wristDSpeed;
	wait1Msec(wristDTime*1000);
	motor[wristMotor] = off;
}

//Moves the wrist of the claw up for a certain time
void moveWristUpForTime(float wristUSpeed, float wristUTime)
{
	motor[wristMotor] = wristUSpeed;
	wait1Msec(wristUTime*1000);
	motor[wristMotor] = off;
}

//Moves the arm down for a certain amount of time
void moveArmDownForTime(float armDSpeed, float armDTime)
{
	motor[armMotor] = -armDSpeed;
	wait1Msec(armDTime*1000);
	motor[armMotor] = off;
}

//Moves the arm up for a certain amount of time
void moveArmUpForTime(float armUSpeed, float armUTime)
{
	motor[armMotor] = armUSpeed;
	wait1Msec(armUTime*1000);
	motor[armMotor] = off;
}

//Extends the arm for a certain amount of time
void extendArmForTime(float extensionSpeed, float extensionTime)
{
	motor[rightExtendMotor] = extensionSpeed;
	motor[leftExtendMotor] = -extensionSpeed;
	wait1Msec(extensionTime*1000);
	motor[rightExtendMotor] = off;
	motor[leftExtendMotor] = off;
}

//Retract the arm for a certain amount of time
void retractArmForTime(float retractionSpeed, float retractionTime)
{
	motor[rightExtendMotor] = -retractionSpeed;
	motor[leftExtendMotor] = retractionSpeed;
	wait1Msec(retractionTime*1000);
	motor[rightExtendMotor] = off;
	motor[leftExtendMotor] = off;
}
