int step = 0;
float averageEncoderValueAbsolute = 0;
float rightEncoderValueAbsolute = 0;
float leftEncoderValueAbsolute = 0;
float averageEncoderOffset = 0;
float rightEncoderOffset = 0;
float leftEncoderOffset = 0;

void init()
{
	SensorValue[GyroSensor] = 0;
	SensorValue[RightEncoder] = 0;
	SensorValue[LeftEncoder] = 0;
	SensorValue[ArmEncoder] = 0;
	SensorValue[LeftExtensionEncoder] = 0;
	SensorValue[WristEncoder] = 0;
	SensorValue[WristPotentiometer] = 0;
	clearTimer(T1);
	clearTimer(T2);
}

void loopCode()
{
	rightEncoderValueAbsolute = -1 * SensorValue[RightEncoder];
	leftEncoderValueAbsolute = SensorValue[LeftEncoder];
	//Record the cumulative values of each sensor
	
	averageEncoderValueAbsolute = ((rightEncoderValueAbsolute + leftEncoderValueAbsolute) / 2);
	//Record the average cumulative value of the two sensors
	
	rightEncoderValueRelative = rightEncoderValueAbsolute - rightEncoderOffset;
	leftEncoderValueRelative = leftEncoderValueAbsolute - leftEncoderOffset;
	//Record the relative values of each sensor
	
	averageEncoderValueRelative = averageEncoderValueAbsolute - averageEncoderOffset;
	//Record the average relative value of the two sensors
}

void clearStep()
{
	clearTimer(T2);
	move(0, 0);
	moveArm(0);
	openClaw(0);
	wristUp(0);
	extendArm(0);
	averageEncoderOffset = averageEncoderValueAbsolute;
	rightEncoderOffset = rightEncoderValueAbsolute;
	leftEncoderOffset = leftEncoderValueAbsolute;
	encoderReference = 0;
	encoderReferenceFlag = false;
	step++;
}
