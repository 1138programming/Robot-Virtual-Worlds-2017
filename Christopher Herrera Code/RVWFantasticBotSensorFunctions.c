int step = 0;
float averageEncoderValueAbsolute = 0;
float averageEncoderValueRelative = 0;
float encoderReference = 0;
float encoderOffset = 0;
bool encoderReferenceFlag = false;
const int kMaxMotorSpeed = 127;

#pragma systemFile;

void move(int speedLeft, int speedRight)
{
	motor[FrontRightMotor] = speedRight;
	motor[RearRightMotor] = speedRight;
	motor[FrontLeftMotor] = speedLeft;
	motor[RearLeftMotor] = speedLeft;
}

void setReference(float value)
{
	if(encoderReferenceFlag == false)
	{
		encoderReference = value;
		encoderReferenceFlag = true;
	}
}

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

void wristUp(int speed)
{
	motor[WristMotor] = speed;
}

//This function moves the arm
void moveArm(int speed)
{
	motor[ArmMotor] = speed;
}

void extendArm(int speed)
{
	motor[LeftExtendMotor] = speed;
	motor[RightExtendMotor] = speed;
}

//This function sets the wrist to a specific potentiometer value
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

//This function sets the arm to a target encoder value
float setArmTo(int target, float logValue = 250, float maxOffset = 8)
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

	return speed <= (127 * log(maxOffset + 1) / log(logValue + 1)) ? 0 : speed;
}

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

void openClaw(int speed)
{
	motor[ClawMotor] = speed;
}

int wait(int msec)
{
	return msec - time1[T2] < 0 ? 0 : msec - time1[T2];
}

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
