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
	averageEncoderValueAbsolute = (((-1 * SensorValue[RightEncoder]) + SensorValue[LeftEncoder]) / 2);
	averageEncoderValueRelative = averageEncoderValueAbsolute - encoderOffset;
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
