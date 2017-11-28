float averageEncoderValueRelative = 0;
float rightEncoderValueRelative = 0;
float leftEncoderValueRelative = 0;
float encoderReference = 0;
bool encoderReferenceFlag = false;
const int kMaxMotorSpeed = 127;
const int kRobotLength = 46; //in centimeters

#pragma systemFile;

//This function moves the base purely based off of the two values given when the function is called
void move(int speedLeft, int speedRight)
{
	//Right side of the base
	motor[FrontRightMotor] = speedRight;
	motor[RearRightMotor] = speedRight;
	//Left side of the base
	motor[FrontLeftMotor] = speedLeft;
	motor[RearLeftMotor] = speedLeft;
}

//Function that sets a reference value for the goForward and Arc functions based on the encoder value when each function is called for the first time
void setReference(float value)
{
	if(encoderReferenceFlag == false)
	{
		encoderReference = value;
		encoderReferenceFlag = true;
	}
}

//This function moves the base a certain distance based either on the encoder or the sonar sensor
//The target is the encoder value (or sonar value if the sonar boolean is set to true) that the robot will aim for
//A higher logValue makes the robot more precise, but slower, and vice versa
float goForward(int target, float logValue = 750, bool sonar = false)
{
	setReference(target + averageEncoderValueRelative);
	//Sets a fixed value for the speed function

	float sonarValue = SensorValue[SonarSensor];
	//Set a variable to the sonar sensor value

	float speed = 0;

	if(!sonar)
	{
		speed = floor((kMaxMotorSpeed * log(abs(encoderReference - averageEncoderValueRelative) + 1)) / (log(logValue + 1)));
		sonarValue = 0;
	}
	else
		speed = floor((kMaxMotorSpeed * log(abs(target - sonarValue) + 1)) / (log(logValue + 1)));
	//As the robot gets close to its destination, it slows down to minimize overshoot

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	speed *= sonar ? -1 : 1;

	if((target - sonarValue) > 0)
		move(speed, speed);
	else
		move(-speed, -speed);
	//Goes either forwards or backwards depending on whther the distance value is positive or negative

	return speed;
}

//This function lets the robot follow an arc based on a radius and angle
//curveLeft determines which way the robot will turn. True for left, false for right
//A higher logValue makes the robot more precise, but slower, and vice versa
float Arc(int radius, int angle, bool curveLeft = true, float logValue = 750) //Note: angles are in radians
{	
	float rightScaleFactor = 1;
	float leftScaleFactor = rightScaleFactor * (((2 * radius) + kRobotLength) / ((2 * radius) - kRobotLength));
	//Sets the scale factors for the speed
	
	float speed = 0;
	
	if(curveLeft)
	{	
		setReference((radius * angle) + rightEncoderValueRelative);
		//Set a fixed value for the speed function based off of the right encoder's value
		
		speed = floor((kMaxMotorSpeed * log(abs(encoderReference - rightEncoderValueRelative) + 1)) / (log(logValue + 1)));
		//As the robot gets close to its destination, it slows down to minimize overshoot
	}
	else
	{
		rightScaleFactor = leftScaleFactor;
		leftScaleFactor = 1;
		//If curve left is set to false, these variables are reset in order to curve right
		
		setReference((radius * angle) + leftEncoderValueRelative);
		//Set a fixed value for the speed function based off of the left encoder's value
		
		speed = floor((kMaxMotorSpeed * log(abs(encoderReference - leftEncoderValueRelative) + 1)) / (log(logValue + 1)));
		//As the robot gets close to its destination, it slows down to minimize overshoot
	}
	//Determines whether the robot will curve to the right or to the left and adjusts variables accordingly
	
	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed
		
	if((abs(angle) / angle) == 1)
		move(leftScaleFactor * speed, rightScaleFactor * speed);
	else
		move(-1 * leftScaleFactor * speed, -1 * rightScaleFactor * speed);
	//Moves the robot in a curve either forwards or backwards depending on the sign of the angle value
		
	return speed;
}

//This function lowers or raises the wrist
void wristUp(int speed)
{
	motor[WristMotor] = speed;
}

//This function moves the arm
void moveArm(int speed)
{
	motor[ArmMotor] = speed;
}

//This function extends the arm
void extendArm(int speed)
{
	motor[LeftExtendMotor] = speed;
	motor[RightExtendMotor] = speed;
}

//This function sets the wrist to a specific potentiometer value (the target)
//A higher logValue makes the wrist more precise, but slower, and vice versa
float setWristTo(int target, float logValue = 250)
{
	float Pot = SensorValue[WristPotentiometer];
	//Sets a variable to the wrist potentiometer's value

	float speed = floor((kMaxMotorSpeed * log(abs(target - Pot) + 1)) / (log(logValue + 1)));
	//Calculates a speed based on how far the sensor value is from the target value

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	if((target - Pot) > 0)
		wristUp(speed);
	else
		wristUp(-speed	);
	//Goes either forwards or backwards depending on whther the target value is positive or negative

	return abs(target - Pot) <= 15 ? 0 : speed;
}

//This function sets the arm to a target encoder value (target)
//A higher logValue makes the arm more precise, but slower, and vice versa
//minOffset determines the minimum distance the arm can be from the target value
float setArmTo(int target, float logValue = 250, float minOffset = 8)
{
	float encoder = SensorValue[ArmEncoder];
	//Sets a variable to the wrist potentiometer's value

	float speed = floor((kMaxMotorSpeed * log(abs(target - encoder) + 1)) / (log(logValue + 1)));
	//Calculates a speed based on how far the sensor value is from the target value

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	if((target - encoder) > 0)
		moveArm(speed);
	else
		moveArm(-speed);
	//Goes either forwards or backwards depending on whther the distance value is positive or negative

	return speed <= (127 * log(minOffset + 1) / log(logValue + 1)) ? 0 : speed;
}

//This function extends the arm to a given target encoder value
//A higher logValue makes the arm more precise, but slower, and vice versa
float extendArmTo(int target, float logValue = 250)
{
	float extensionEncoder = SensorValue[LeftExtensionEncoder];
	//Sets a variable to the extension encoder's value

	float speed = floor((kMaxMotorSpeed * log(abs(target - extensionEncoder) + 1)) / (log(logValue + 1)));
	//Calculates a speed based on how far the sensor value is from the target value

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	if((target - extensionEncoder) > 0)
		extendArm(speed);
	else
		extendArm(-speed);
	//Goes either forwards or backwards depending on whther the target value is positive or negative

	return abs(target - extensionEncoder) <= 3 ? 0 : speed;
}

//This function opens or closes the claw
void openClaw(int speed)
{
	motor[ClawMotor] = speed;
}

//This function returns the difference between the timer value and a given value
//I use this instead of waits, so that other things can run at the same time
int wait(int msec)
{
	return msec - time1[T2] < 0 ? 0 : msec - time1[T2];
}

//This function turns the robot a given amount of degrees
//A higher logValue makes the robot more precise, but slower, and vice versa
//By default, the robot will make the shortest turn, but the direction variable overrides this. 1 for a right turn, -1 for a left turn
float turn(int degrees, float logValue = 1800, int direction = 0)
{
	float Gyro = SensorValue[GyroSensor];
	if(Gyro < 0)
		Gyro += 3600;
	if(degrees < 0)
		degrees += 3600;

	float rightAngle = degrees > Gyro ? degrees - Gyro : (3600 - Gyro) + degrees;
	float leftAngle = degrees > Gyro ? (3600 - degrees) + Gyro : Gyro - degrees;
	//Determines the angle of the right and left turns

	float speed = floor((kMaxMotorSpeed * log(abs(Gyro - degrees) + 1)) / (log(logValue + 1)));
	//As the difference between the desired degree and the gyro value get bigger, the speed increases logarithmically

	if(abs(direction) != 1)
	{
		if(rightAngle < leftAngle)
			move(speed, -speed); //Turn right
		else
			move(-speed, speed); //Turn left
	}
	else if(direction == 1)
		move(speed, -speed); //Turn right
	else if(direction == -1)
		move(-speed, speed); //Turn left

	return speed <= 15 ? 0 : speed;
}
