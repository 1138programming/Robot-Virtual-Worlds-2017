#pragma config(StandardModel, "RVW Fantasticbot")
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

task main()
{
resetMotorEncoder(rearLeftMotor);
setMotorTarget(rearLeftMotor, 600, 80, false);
while(getMotorTargetCompleted(rearLeftMotor) == false)
{
	motor[rearLeftMotor] = 80;
	getMotorEncoder(rearLeftMotor);
}
	motor[rearLeftMotor] = 0;
}
