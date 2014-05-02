//============================================================================
// Name        : epos2_driver.cpp
// Author      : Martin Seidel
// Version     :
// Copyright   :
// Description :
//============================================================================

#include "SampleMoving.h"
#include "ros/ros.h"
#include <bondcpp/bond.h>		//for sending alive-messages
#include <signal.h>

sig_atomic_t gotSignal=0;

void killCallback(int signal)
{
	gotSignal=signal;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "epos2_talker");
	ros::NodeHandle roshandle;

	//Elemente von talkerStruct:	speedStepSize	- in part of the maximum speed (<- read from the parameter-server)
	//								refreshRate		- with this rate a new speed-message will be send; in Hz
	//								angle			- angle between right and left; takes only effect when input:=sinus

	ros::Rate loopRate = 20;
	double speedStepSize = 0.2, refreshRate = 11;
	int angle = 90;

	ros::param::get("~speedStep", speedStepSize);
	ros::param::get("~speedUpdateRate", refreshRate);

	std::string  getParam;
	SampleMoving::inputDevice getInfoFrom=SampleMoving::nodrive;
	SampleMoving::msgTypes msgType = SampleMoving::noMsg;

	if (ros::param::get("~input", getParam)) {
		if (strstr(getParam.c_str(), "humanInterface") != NULL) {getInfoFrom=SampleMoving::humanInterface;}
		//if (strstr(getParam.c_str(), "keyboard") != NULL) {getInfoFrom=SampleMoving::keyboard;}
		if (strstr(getParam.c_str(), "sinus") != NULL) {getInfoFrom=SampleMoving::sinus;}
		//if (strstr(getParam.c_str(), "gamepad") != NULL) {getInfoFrom=SampleMoving::gamepad;}
	}

	if (ros::param::get("~msgType", getParam)) {
		if (strstr(getParam.c_str(), "cmdVel") != NULL) {msgType=SampleMoving::cmdVel;}
		if (strstr(getParam.c_str(), "SI") != NULL) {msgType=SampleMoving::SI;}
		if (strstr(getParam.c_str(), "navStack") != NULL) {msgType=SampleMoving::navStack;}
		if (strstr(getParam.c_str(), "onlyTF") != NULL) {msgType=SampleMoving::onlyTF;}
	}

	bond::Bond sampleBond("epos2_bond", "sampleMoving");	//new local bond
	sampleBond.setConnectTimeout(90);
	signal(SIGTERM, &killCallback);
	signal(SIGABRT, &killCallback);
	signal(SIGBUS, &killCallback);
	signal(SIGHUP, &killCallback);
	signal(SIGILL, &killCallback);
	signal(SIGINT, &killCallback);
	signal(SIGSEGV, &killCallback);

	SampleMoving startMove(roshandle, getInfoFrom, msgType, refreshRate, speedStepSize, angle);

	sampleBond.start();

	while(!gotSignal && ros::ok())
	{
		loopRate.sleep();
		ros::spinOnce();
	}

	sampleBond.breakBond();

	return gotSignal;
}

