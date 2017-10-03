#pragma config(Sensor, in1,    LeftLightSensor, sensorReflection)
#pragma config(Sensor, in2,    MiddleLightSensor, sensorReflection)
#pragma config(Sensor, in3,    RightLightSensor, sensorReflection)
#pragma config(Sensor, in4,    WristPotentiometer, sensorPotentiometer)
#pragma config(Sensor, in6,    GyroSensor,     sensorGyro)
#pragma config(Sensor, dgtl7,  TouchSensor,    sensorTouch)
#pragma config(Sensor, dgtl8,  SonarSensor,    sensorSONAR_cm)
#pragma config(Sensor, I2C_1,  RightEncoder,   sensorNone)
#pragma config(Sensor, I2C_2,  LeftEncoder,    sensorNone)
#pragma config(Sensor, I2C_3,  ArmEncoder,     sensorNone)
#pragma config(Motor,  port1,           FrontRightMotor, tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           RearRightMotor, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           FrontLeftMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           RearLeftMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           ClawMotor,     tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ArmMotor,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LeftExtendMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           RightExtendMotor, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          WristMotor,    tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "RVWFantasticBotFunctions.c"

task main()
{
	init();
	while(true)
	{
		loopCode();
		switch(step)
		{
			case 0: //Move to the mobile goal and lift arm
				moveArm(20);
				if(goForward(1200) == 0)
					clearStep();
				break;
			case 1: //Release cone onto the mobile goal (2 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 2: //Lower the arm and wrist
				moveArm(-100, false);
				wristUp(-127);
				if(wait(350) == 0)
					clearStep();
				break;
			case 3: //Grab the mobile goal
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 4: //Turn to face the matchloads
				if(turn(2561) == 0)
					clearStep();
				break;
			case 5: //Move toward the matchload and stack another cone (4 points)
				if(goForward(825) == 0)
					clearStep();
				break;
			case 6: //Move back
				if(goForward(-300) == 0)
					clearStep();
				break;
			case 7: //Turn towards the wall
				if(turn(2250) == 0)
					clearStep();
				break;
			case 8: //Move forward a small amount
				if(goForward(140, 1000) == 0)
					clearStep();
				break;
			case 9: //Let go of the mobile goal
				openClaw(127);
				if(wait(100) == 0)
					clearStep();
				break;
			case 10: //Move back
				if(goForward(-150, 800) == 0)
					clearStep();
				break;
			case 11: //Turn to face a cone
				if(turn(3150) == 0)
					clearStep();
				break;
			case 12: //Move to the cone
				if(goForward(400) == 0)
					clearStep();
				break;
			case 13: //Close the claw to grab the cone
				openClaw(-127);
				if(wait(100) == 0)
					clearStep();
				break;
			case 14: //Lift up arm
				moveArm(127);
				if(wait(700) == 0)
					clearStep();
				break;
			case 15: //Move forward
				if(goForward(275) == 0)
					clearStep();
				break;
			case 16: //Turn to between the mobile goal and the preloads
				if(turn(1880) == 0)
					clearStep();
				break;
			case 17: //Move forward
				wristUp(55);
				if(goForward(320) == 0)
					clearStep();
				break;
			case 18: //Turn to place third cone
				if(turn(1680) == 0)
					clearStep();
				break;
			case 19: //Release cone to score it (6 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 20: //Turn to preloads
				if(turn(2200, 14400) == 0)
					clearStep();
				break;
			case 21: //Grab preload
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 22: //Turn to place fourth cone
				if(turn(1700, 14400) == 0)
					clearStep();
				break;
			case 23: //Release cone to score it (8 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 24: //Turn to preloads
				if(turn(2200, 14400) == 0)
					clearStep();
				break;
			case 25: //Grab preload
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 26: //Turn to place fifth cone
				if(turn(1720, 14400) == 0)
					clearStep();
				break;
			case 27: //Release cone to score it (10 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 28: //Turn to preloads
				if(turn(2240, 14400) == 0)
					clearStep();
				break;
			case 29: //Grab preload
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 30: //Turn to place sixth cone
				moveArm(40);
				extendArm(50);
				if(turn(1720, 14400) == 0)
					clearStep();
				break;
			case 31: //Release cone to score it (12 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 32: //Turn to preloads
				moveArm(-20, false);
				extendArm(-60);
				if(turn(2240, 14400) == 0)
					clearStep();
				break;
			case 33: //Grab preload
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 34: //Turn to place sixth cone
				moveArm(60, false);
				extendArm(60);
				if(turn(1720, 14400) == 0)
					clearStep();
				break;
			case 35: //Release cone to score it (12 points)
				openClaw(127);
				if(wait(50) == 0)
					clearStep();
				break;
			case 36: //Move arm down to pick up mobile goal
				moveArm(-60, false);
				extendArm(-50);
				wristUp(77);
				if(wait(1000) == 0)
					clearStep();
				break;
			case 37: //Move forward towards mobile goal
				if(goForward(250) == 0)
					clearStep();
				break;
			case 38: //Grab mobile goal
				openClaw(-127);
				if(wait(50) == 0)
					clearStep();
				break;
		}
	}
}