int step = 0;
double averageEncoderValueAbsolute = 0;
double averageEncoderValueRelative = 0;
double encoderReference = 0;
double encoderOffset = 0;
bool encoderReferenceFlag = false;
const int kMaxMotorSpeed = 127;

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

double goForward(int distance)
{
	setReference(distance + averageEncoderValueRelative);
	//Sets a fixed value for the speed function

	double speed = floor((kMaxMotorSpeed * log(abs(encoderReference - averageEncoderValueRelative) + 1)) / (log(301.0)));
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

void openClaw(int speed)
{
	motor[ClawMotor] = speed;
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

void clearStep()
{
	clearTimer(T2);
	move(0, 0);
	moveArm(0);
	openClaw(0);
	wristUp(0);
	encoderOffset = averageEncoderValueAbsolute;
	encoderReference = 0;
	encoderReferenceFlag = false;
	step++;
}

int wait(int msec)
{
	return msec - time1[T2] < 0 ? 0 : msec - time1[T2];
}

double turn(int degrees)
{
	float Gyro = SensorValue[GyroSensor];
	if(Gyro < 0)
		Gyro = 3600 - (Gyro * -1);
	if(degrees < 0)
		degrees = 3600 - (degrees * -1);

	double rightAngle = degrees > Gyro ? degrees - Gyro : (3600 - Gyro) + degrees;
	double leftAngle = degrees > Gyro ? (3600 - degrees) : Gyro - degrees;

	float speed;
	//As the difference between the desired degree and the gyro value get bigger, the speed increases logarithmically

	if(rightAngle < leftAngle)
	{
		speed = floor((kMaxMotorSpeed * log(abs(degrees - Gyro) + 1)) / (log(1801.0)));
		move(speed, -speed); //Turn right
	}
	else
	{
		speed = floor((kMaxMotorSpeed * log(abs((3600 - degrees) - (3600 - Gyro)) + 1)) / (log(1801.0)));
		move(-speed, speed); //Turn left
	}

	return speed;
}
