int step = 0;
float averageEncoderValueAbsolute = 0;
float averageEncoderValueRelative = 0;
float encoderReference = 0;
float encoderOffset = 0;
bool encoderReferenceFlag = false;
const int kMaxMotorSpeed = 127;

#pragma systemFile;

void init()
{
	SensorValue[GyroSensor] = 0;
	SensorValue[RightEncoder] = 0;
	SensorValue[LeftEncoder] = 0;
	clearTimer(T1);
	clearTimer(T2);
}

void loopCode()
{
	averageEncoderValueAbsolute = (((-1 * SensorValue[RightEncoder]) + SensorValue[LeftEncoder]) / 2);
	averageEncoderValueRelative = averageEncoderValueAbsolute - encoderOffset;
}

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

float goForward(int distance, float logValue = 750)
{
	setReference(distance + averageEncoderValueRelative);
	//Sets a fixed value for the speed function

	float speed = floor((kMaxMotorSpeed * log(abs(encoderReference - averageEncoderValueRelative) + 1)) / (log(logValue + 1)));
	//As the robot gets close to its destination, it slows down to minimize overshoot

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	if(distance > 0)
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

void moveArm(int speed, bool pinWrist = true)
{
	motor[ArmMotor] = speed;
	if(pinWrist)
		wristUp(-speed);
}

float setArmTo(int target, float logValue = 750)
{
	float Pot = SensorValue(WristPotentiometer);
	//Sets a variable to the wrist potentiometer's value

	float speed = floor((kMaxMotorSpeed * log(abs(target - Pot) + 1)) / (log(logValue + 1)));
	//Calculates a speed based on how far the sensor value is from the target value

	if(speed > kMaxMotorSpeed)
		speed = 127 * (abs(speed) / speed);
	//Makes sure that the speed doesn't go over the max motor speed

	if((target - Pot) > 0)
		moveArm(speed, false);
	else
		moveArm(-speed, false);
	//Goes either forwards or backwards depending on whther the distance value is positive or negative

	return speed;
}

void openClaw(int speed)
{
	motor[ClawMotor] = speed;
}

void extendArm(int speed)
{
	motor[LeftExtendMotor] = speed;
	motor[RightExtendMotor] = speed;
}

void clearStep()
{
	clearTimer(T2);
	move(0, 0);
	moveArm(0);
	openClaw(0);
	wristUp(0);
	extendArm(0);
	encoderOffset = averageEncoderValueAbsolute;
	encoderReference = 0;
	encoderReferenceFlag = false;
	step++;
}

int wait(int msec)
{
	return msec - time1[T2] < 0 ? 0 : msec - time1[T2];
}

float turn(int degrees, float logValue = 1800)
{
	float Gyro = SensorValue[GyroSensor];
	if(Gyro < 0)
		Gyro += 3600;
	if(degrees < 0)
		degrees += 3600;

	float rightAngle = degrees > Gyro ? degrees - Gyro : (3600 - Gyro) + degrees;
	float leftAngle = degrees > Gyro ? (3600 - degrees) + Gyro : Gyro - degrees;

	float speed;
	//As the difference between the desired degree and the gyro value get bigger, the speed increases logarithmically

	if(rightAngle < leftAngle)
	{
		speed = floor((kMaxMotorSpeed * log(abs(degrees - Gyro) + 1)) / (log(logValue + 1)));
		move(speed, -speed); //Turn right
	}
	else
	{
		speed = floor((kMaxMotorSpeed * log(abs(Gyro - degrees) + 1)) / (log(logValue + 1)));
		move(-speed, speed); //Turn left
	}

	return speed <= 15 ? 0 : speed;
}
