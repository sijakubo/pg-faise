/*
 * SampleMoving.cpp
 *
 *  Created on: Apr 11, 2012
 *      Author: Martin Seidel
 *      modified by Jannik Flessner
 */

#include "SampleMoving.h"

//public
SampleMoving::SampleMoving(ros::NodeHandle sampleMovingRoshandle, inputDevice getInfoFrom, msgTypes msgType, double refreshRate, double keyboardSpeedStepSize, int angle)//: action("move_base")
{
	sampleMovingSet.sineAngle = angle%360;
	sampleMovingSet.refreshRate = refreshRate;
	sampleMovingSet.SpeedStepSize = keyboardSpeedStepSize;

	roshandle = sampleMovingRoshandle;

	init(msgType, getInfoFrom);
}

SampleMoving::~SampleMoving()
{
	int kfd=0;//SDTIN_FILEN0;

	tcsetattr(kfd, TCSANOW, &(sampleMovingSet.oldKeyboardIO));	//restore terminal settings
}

//private
void SampleMoving::init(msgTypes msgType, inputDevice getInfoFrom)
{
	sampleMovingSet.epos2MPS = roshandle.advertise<epos2_control::velocity>("epos2_MPS_left_right", 1000);
	sampleMovingSet.epos2Twist = roshandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::param::get("/epos2_control/max_mps", sampleMovingSet.maxMPS);
	ros::param::get("/epos2_control/wheelBase", sampleMovingSet.wheelBase);
	ros::param::get("/epos2_control/axisLength", sampleMovingSet.axisLength);
	ros::param::get("/epos2_control/wheelPerimeter", sampleMovingSet.wheelPerimeter);

	for (int i=left; i<=right; i++) { sampleMovingSet.targetVelocity[i] = 0; }	//starting speed
	sampleMovingSet.ableToMove = false;	//only for 3-axis wheel against bad init
	sampleMovingSet.sineCurrentAngle = 42;
	sampleMovingSet.joyControll=noJoy;
	sampleMovingSet.gamepadMode = oneStick;
	sampleMovingSet.joyInUse=0;

	if (msgType == noMsg) {
		ROS_ERROR("No valid Message Type given to the epos2_talker. Use the argument \"msgType:=\" (default is \"SI\").\nPossible Message Types are: cmdVel; SI; navStack; onlyTF. msgType will be set to default.");
		sampleMovingSet.msgType = SI;
	} else {
		sampleMovingSet.msgType = msgType;
	}

	/*if (msgType == navStack) {
		while(!action.waitForServer(ros::Duration(5.0))){ ROS_INFO("Waiting for the move_base action server to come up"); }
	}*/

	keyboardInit();

	sampleMovingSet.epos2SetSpeedTimer = roshandle.createTimer(ros::Rate(sampleMovingSet.refreshRate), &SampleMoving::publishSpeed, this);

	ROS_ERROR("[Talker] maxMPS: %.3f, speedStep: %.3f and speedUpdateRate: %.3f", sampleMovingSet.maxMPS, sampleMovingSet.SpeedStepSize , sampleMovingSet.refreshRate);
	ROS_ERROR("[Talker] wheelBase: %.4lf, axisLength: %.4lf, wheelPerimeter: %.4lf", sampleMovingSet.wheelBase, sampleMovingSet.axisLength, sampleMovingSet.wheelPerimeter);
	ROS_ERROR("[Talker] inputDevice: %i, msgType: %i", getInfoFrom, sampleMovingSet.msgType);

	//start the callbacks depending on input
	if (getInfoFrom == sinus) {
		sampleMovingSet.sineTimer = roshandle.createTimer(ros::Rate(sampleMovingSet.refreshRate), &SampleMoving::sineCallback, this);
	} else if (getInfoFrom == humanInterface) {
		sampleMovingSet.keyboardTimer = roshandle.createTimer(ros::Rate(sampleMovingSet.refreshRate), &SampleMoving::keyboardCallback, this);
		sampleMovingSet.gamepad_sub = roshandle.subscribe("joy", 1000, &SampleMoving::gamepadCallback, this);
	} else {
		ROS_ERROR("No valid command-source given to the epos2_talker.\nUse the argument \"input:=\" (default is \"humanInterface\"). Possible sources are: humanInterface; sinus");
	}
}

void SampleMoving::publishSpeed(const ros::TimerEvent& event)
{
	double speedMPS[2], pos_delta[2];
	epos2_control::velocity speed;
	geometry_msgs::Twist speedVector;

		//get velocity in meter per second
	for (int i=left; i<=right; i++) { speedMPS[i] = sampleMovingSet.targetVelocity[i] * sampleMovingSet.maxMPS; }
	//ROS_ERROR("[Talker-Publish] targetSpeed_left: %.4lf & targetSpeed_right: %.4lf", sampleMovingSet.targetVelocity[left], sampleMovingSet.targetVelocity[right]);


	switch (sampleMovingSet.msgType) {
		case SI:	//publish current velocity in meter per second
			speed.left=speedMPS[left];
			speed.right=speedMPS[right];
			sampleMovingSet.epos2MPS.publish(speed);
			break;
		case cmdVel:	//publish current velocity in a cmd_vel message
			speedVector.linear.x = 0.5;
			//speedVector.linear.x = ( speedMPS[right] + speedMPS[left] ) * 0.5;
			speedVector.linear.y= sampleMovingSet.targetVelocity[hub];
			speedVector.linear.z= sampleMovingSet.targetVelocity[flow];
			speedVector.angular.x=0;
			speedVector.angular.y=0;
			speedVector.angular.z=(speedMPS[right] - speedMPS[left]) / sampleMovingSet.axisLength;
			sampleMovingSet.epos2Twist.publish(speedVector);
			break;
	}

		//calculate current position depending on last position
	for (int i=left; i<=right; i++) { pos_delta[i] = speedMPS[i] * (event.current_real - event.last_real).toSec(); }

	double polar_s  =(pos_delta[right] + pos_delta[left] ) * 0.5;
	double polar_theta=(pos_delta[right] - pos_delta[left]) / sampleMovingSet.axisLength;
	double delta_r = (polar_theta == 0) ? polar_s : (polar_s / polar_theta) * sin(polar_theta/2) * 2;

	pos.now.x = pos.last.x + delta_r * cos(polar_theta/2 + pos.last.theta);
	pos.now.y = pos.last.y + delta_r * sin(polar_theta/2 + pos.last.theta);
	pos.now.theta = pos.last.theta + polar_theta;

	//copy new position to last position for next turn
	pos.last.x = pos.now.x;
	pos.last.y = pos.now.y;
	pos.last.theta = pos.now.theta;

	//sending actionLib-msg for NavStack ...doesn't work by now on ros::groovy
	/*if (targetSpeed.publishMsgType == navStack) {
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = pos.now.x;
		goal.target_pose.pose.position.y = pos.now.y;
		goal.target_pose.pose.orientation.w = pos.now.theta;

		//ROS_ERROR("Sending goal");
		action.sendGoal(goal);
		action.waitForResult();

		if(action.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_ERROR("The base failed to move for some reason.");
		}
	}*/

	//publish tf base_link
	sampleMovingSet.transform.setOrigin( tf::Vector3(pos.now.x, pos.now.y, 0.0) );
	sampleMovingSet.transform.setRotation(tf::Quaternion( 0,0, sin(pos.now.theta/2), cos(pos.now.theta/2) ));
	sampleMovingSet.tfBroad.sendTransform(tf::StampedTransform(sampleMovingSet.transform, event.current_real, "odom", "talker_base"));
}

void SampleMoving::gamepadCallback (const sensor_msgs::Joy joymsg)
{
	double targetSpeed[2] = {0,0};
	int moving = false;
	int lastJoyControll = sampleMovingSet.joyControll;
	int lastGamepadMode = sampleMovingSet.gamepadMode;
	std::string padMode="";
	std::string gamePadType="";
	axesStruct axesRemap;

	//init of axesRemap
	axesRemap.leftx = 0;
	axesRemap.lefty = 0;
	axesRemap.rightx = 0;
	axesRemap.righty = 0;
	axesRemap.switchButton = 0;
	axesRemap.quickStopButton = 0;

//----------Switching of interfaces--------------------------//
	switch (joymsg.axes.size()) {
		case 2:	sampleMovingSet.joyControll=wheel;
				axesRemap.lefty = joymsg.axes[1];
				sampleMovingSet.ableToMove = true;
				padMode = "steering wheel with two pedals";
				gamePadType = wheel;
				break;
		case 3:	axesRemap.lefty=(joymsg.axes[1] - joymsg.axes[2])/2;
				gamePadType = wheel;
				padMode = "steering wheel with two pedals";
				sampleMovingSet.joyControll=wheel;

				if (sampleMovingSet.ableToMove == false) {
					padMode = "steering wheel with two pedals - but not initialed (press both pedals once)";
					sampleMovingSet.joyControll=noJoy;
				}
				if (joymsg.axes[1] == -1 && joymsg.axes[2] == -1) {
					sampleMovingSet.ableToMove = true;
				}
				//ROS_ERROR("xvel: %.4f, axes1: %.4f, axes2: %.4f", xVelocity, joymsg.axes[1], joymsg.axes[2]);
				break;
		case 4:	sampleMovingSet.joyControll = gamePad;
				gamePadType = "normal Gamepad";
				axesRemap.leftx = joymsg.axes[0];
				axesRemap.lefty = joymsg.axes[1];
				axesRemap.rightx = joymsg.axes[2];
				axesRemap.righty = joymsg.axes[3];
				axesRemap.switchButton = joymsg.buttons[3];
				sampleMovingSet.ableToMove = true;
				break;
		case 6:	sampleMovingSet.joyControll = gamePad;
				gamePadType = "Xbox DirectInput";
				axesRemap.leftx = joymsg.axes[0];
				axesRemap.lefty = joymsg.axes[1];
				axesRemap.rightx = joymsg.axes[2];
				axesRemap.righty = joymsg.axes[3];
				axesRemap.switchButton = joymsg.buttons[3];
				axesRemap.quickStopButton = joymsg.buttons[2];
				sampleMovingSet.ableToMove = true;
				break;
		case 8:	sampleMovingSet.joyControll = gamePad;
				gamePadType = "Xbox XInput";
				axesRemap.leftx = joymsg.axes[0];
				axesRemap.lefty = joymsg.axes[1];
				axesRemap.rightx = joymsg.axes[3];
				axesRemap.righty = joymsg.axes[4];
				axesRemap.switchButton = joymsg.buttons[3];
				axesRemap.quickStopButton = joymsg.buttons[1];
				sampleMovingSet.ableToMove = true;
				break;
	}

	//switching of gamepad-modes
	if (axesRemap.leftx != 0 || axesRemap.lefty != 0 || axesRemap.rightx != 0 || axesRemap.righty != 0) {moving = true;}

	if (sampleMovingSet.joyControll == gamePad && axesRemap.switchButton == 1  && moving == false) {
		sampleMovingSet.gamepadMode = (sampleMovingSet.gamepadMode + 1) % 3;

		switch (sampleMovingSet.gamepadMode) {
			case oneStick:	padMode = "oneStick: left Stick controls the robot (back- & forward + left & right)";
							break;
			case twoStick:	padMode = "twoStick: left Stick back- and forward; right stick left and right";
							break;
			case tankStick: padMode = "tankStick: left stick = left side; right stick = right side";
							break;
		}
	}

	//output new mode and interface
	if( lastJoyControll != sampleMovingSet.joyControll || lastGamepadMode != sampleMovingSet.gamepadMode) {
		ROS_ERROR("change humanInterface mode: (%s) %s (Count of Axis: %i)", gamePadType.c_str(), padMode.c_str(), joymsg.axes.size());
	}

//----------Changing of Speed depending on mode---------------//
	if (sampleMovingSet.ableToMove == true) {
		double Ratio;	//pretend against values higher 1
		float r_min = 0.3, r_delta = 1;	//in meter - wheel
		double sideShift = 0, radius;	//wheel

		switch (sampleMovingSet.joyControll) {
			case gamePad:
				switch (sampleMovingSet.gamepadMode) {
					case oneStick: //left Stick controls the robot (back- & forward + left & right)
						Ratio = fabs(axesRemap.lefty) + fabs(axesRemap.leftx);
						if (Ratio < 1) {Ratio = 1;}
						targetSpeed[left] = (axesRemap.lefty - axesRemap.leftx) / Ratio;
						targetSpeed[right] =(axesRemap.lefty + axesRemap.leftx) / Ratio;
						break;
					case twoStick:	//left Stick back- and forward; right stick left and right
						Ratio = fabs(axesRemap.lefty) + fabs(axesRemap.rightx);
						if (Ratio < 1) {Ratio = 1;}
						targetSpeed[left] = (axesRemap.lefty - axesRemap.rightx) / Ratio;
						targetSpeed[right] =(axesRemap.lefty + axesRemap.rightx) / Ratio;
						break;
					case tankStick:	//left stick=left side; right stick = right side
						targetSpeed[left]=axesRemap.lefty;
						targetSpeed[right]=axesRemap.righty;
						break;
				}
				break;
			case wheel:	//left stick only, because the steering wheel got only two axes
				if (fabs(joymsg.axes[0]) > 0.001) {	//get radius and depending on it the delta between both sides
					radius = ( r_min + r_delta * (1 - fabs(joymsg.axes[0])) ) * sign(joymsg.axes[0]);
					sideShift = sampleMovingSet.axisLength * 0.5 / radius;
				}
				if ( ( fabs(axesRemap.lefty) * (1 + fabs(sideShift)) ) > 1) {//prevent against values greater 1 or less -1
					axesRemap.lefty = 1 / (1 + fabs(sideShift)) * sign(axesRemap.lefty);
				}
				//ROS_ERROR("xVelocity: %.4lf & sideShift: %.4lf", xVelocity, sideShift);
				targetSpeed[left] = axesRemap.lefty * (1 - sideShift);
				targetSpeed[right]= axesRemap.lefty * (1 + sideShift);
				break;
		}//end switch joyControll
	}//end if ableToMove

	double smoothVeloLeft = targetSpeed[left], smoothVeloRight = targetSpeed[right];
	//smoothVelocity(targetSpeed, &smoothVeloLeft, &smoothVeloRight, sampleMovingSet.SpeedStepSize); //changes will only take effect with new joy-Messages
	//ROS_ERROR("[gamepad] left: %.2f & right: %.2f", smoothVeloLeft, smoothVeloRight);

	sampleMovingSet.joyInUse = ( fabs(smoothVeloLeft) < 0.001 && fabs(smoothVeloRight) < 0.001 ) ? 0 : 1;
	sampleMovingSet.targetVelocity[left] = smoothVeloLeft;
	sampleMovingSet.targetVelocity[right] = smoothVeloRight;
}

void SampleMoving::sineCallback(const ros::TimerEvent& event)
{
	sampleMovingSet.sineCurrentAngle=(sampleMovingSet.sineCurrentAngle + 1) % 360;

	sampleMovingSet.targetVelocity[left]=sin(sampleMovingSet.sineCurrentAngle * pi / 180);
	sampleMovingSet.targetVelocity[right]=sin( (sampleMovingSet.sineCurrentAngle+sampleMovingSet.sineAngle) * pi / 180);
}

void SampleMoving::keyboardInit()
{
	long sec, usec;

	//------set Terminal------------------------------//
	int kfd=0;//SDTIN_FILEN0;
	struct termios new_io;

	tcgetattr( kfd, &(sampleMovingSet.oldKeyboardIO) );
	memcpy(&new_io, &(sampleMovingSet.oldKeyboardIO), sizeof(struct termios));
	new_io.c_lflag &=~ (ICANON | ECHO);
	new_io.c_cc[VEOL] = 1;
	new_io.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &new_io);
	//------ set Terminal end------------------------//


	if (sampleMovingSet.refreshRate > 0.01) {	// calculate the time to wait
		sec = (long) (1/sampleMovingSet.refreshRate);
		usec = ((long) (1000000/sampleMovingSet.refreshRate))%1000000;
	} else {
		sec = 100;
		usec = 0;
	}

	sampleMovingSet.timeForKeys.tv_sec = sec;
	sampleMovingSet.timeForKeys.tv_usec = usec;
}

void SampleMoving::keyboardCallback(const ros::TimerEvent& event)
{
	struct timeval timeForKeys;
	fd_set rfds;
	char latestChar[10];
	int keyPressed=0, returnBit;
	double targetSpeed[2] = {0,0};
	double smoothVeloLeft = 0, smoothVeloRight = 0;

	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	memcpy(&timeForKeys, &(sampleMovingSet.timeForKeys), sizeof(timeval));	//set time

	returnBit = select(1, &rfds, NULL, NULL, &timeForKeys);	//wait for Key

	if(returnBit > 0)
	{
		if (read(0, &latestChar, 5) >= 0) {
			//ROS_INFO("last char: %s", latestChar);

			switch(latestChar[0]) {
			case KEYCODE_W:	targetSpeed[left] = 1;
							targetSpeed[right]= 1;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_S:	targetSpeed[left] = -1;
							targetSpeed[right]= -1;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_A:	targetSpeed[left] = -1;
							targetSpeed[right]= 1;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_D:	targetSpeed[left] = 1;
							targetSpeed[right]= -1;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_Q:	targetSpeed[left] = 0.5;
							targetSpeed[right]= 1;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_E:	targetSpeed[left] = 1;
							targetSpeed[right]= 0.5;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_H:	targetSpeed[left] = 0;
							targetSpeed[right]= 0;
							sampleMovingSet.targetVelocity[hub] = 0.5;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_B:	targetSpeed[left] = 0;
							targetSpeed[right]= 0;
							sampleMovingSet.targetVelocity[hub] = -0.5;
							sampleMovingSet.targetVelocity[flow] = 0;
							break;
			case KEYCODE_F:	targetSpeed[left] = 0;
							targetSpeed[right]= 0;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = 0.02;
							break;
			case KEYCODE_C:	targetSpeed[left] = 0;
							targetSpeed[right]= 0;
							sampleMovingSet.targetVelocity[hub] = 0;
							sampleMovingSet.targetVelocity[flow] = -0.02;
							break;

			}
		}
	}

	if (sampleMovingSet.joyControll == noJoy || sampleMovingSet.joyInUse == 0) {
		smoothVelocity(targetSpeed, &smoothVeloLeft, &smoothVeloRight, sampleMovingSet.SpeedStepSize, 1);
		//ROS_ERROR("[keyboard] left: %.2f & right: %.2f", smoothVeloLeft, smoothVeloRight);
		sampleMovingSet.targetVelocity[left] = smoothVeloLeft;
		sampleMovingSet.targetVelocity[right] = smoothVeloRight;
	}
}

void SampleMoving::smoothVelocity(double inputVelocity[2], double *outputVelocityLeft, double *outputVelocityRight, double speedStepSize, int useDelta)
{
	double deltaDelta = 0, speedDelta = 0, currentVelocity[2];
	int  deltaSign = 0;
	double outputVelocity[2] = {0,0};

	for (int i=left; i<=right; i++) {
		currentVelocity[i] = sampleMovingSet.targetVelocity[i];
	}

	deltaDelta = (inputVelocity[left] - inputVelocity[right]) - (currentVelocity[left] - currentVelocity[right]);
	deltaDelta *= useDelta;

	if (deltaDelta == 0) {
		for (int i=left; i<=right; i++) {
			speedDelta = inputVelocity[i] - currentVelocity[i];
			deltaSign = sign(speedDelta, 1);
			outputVelocity[i] = nextToZero(deltaSign, currentVelocity[i] + deltaSign*nextToZero(speedStepSize, fabs(speedDelta)) );
		}
	} else {
		deltaSign = sign(deltaDelta, 1);
		outputVelocity[left] = nextToZero(deltaSign, currentVelocity[left] + deltaSign * nextToZero(speedStepSize, fabs(deltaDelta)/2) );
		outputVelocity[right]= nextToZero((-1)*deltaSign, currentVelocity[right] - deltaSign * nextToZero(speedStepSize, fabs(deltaDelta)/2) );
	}

	 *outputVelocityLeft = outputVelocity[left];
	 *outputVelocityRight = outputVelocity[right];
	//ROS_ERROR("[keyboard] left: %.2f & right: %.2f", outputVelocity[left], outputVelocity[right]);
}

double SampleMoving::nextToZero(double arg1, double arg2)
{
	return ( fabs(arg1) < fabs(arg2) )?arg1:arg2;
}

int SampleMoving::sign(double number, int zeroIsPositiv)
{
	return (number>0) ? 1 : (number<0) ? -1 : (zeroIsPositiv == 1) ? 1 : 0;
}




