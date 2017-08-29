#pragma config(StandardModel, "RVW Fantasticbot")
//*!!Code automatically generated by 'ROBOTC' configuration wizard

//This code includes functions to operate the robot, "Fantasticbot"
#include "FantasticBot_Functions.c"

//This file is the main file for the RVW challenge.
task main()
{
	//This is the set of tasks to move the robot to get 4 points by:
	//Scoring a cone on a mobile goal
	//Scoring a cone on a post goal
	moveArmUpForTime(127, 0.5);
	moveForwardForTime(127, 2);
	moveArmDownForTime(127, 0.4);
	openClaw();
	moveBackwardForTime(127, 0.55);
	turnLeftForTime(127, 0.7);
	moveArmUpForTime(127, 0.4);
	moveWristDownForTime(127, 0.3);
	moveForwardForTime(127, 1.2);
	closeClaw();
	moveBackwardForTime(127, 1);
	turnRightForTime(127, 1.35);
	moveArmUpForTime(127, 0.6);
	moveWristDownForTime(127, 0.8);
	moveForwardForTime(127, 0.85);
	openClaw();
}
