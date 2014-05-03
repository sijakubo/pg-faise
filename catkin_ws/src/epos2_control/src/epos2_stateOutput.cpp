/*
 * StateOutput.cpp
 *
 *  Created on: 11.07.2013
 *      Author: eos
 */

#include "ros/ros.h"
#include <fstream>
#include <unistd.h>
#include <bondcpp/bond.h>

/**
* Structure for state-output informations
*/
struct outputStruct {
unsigned int dutyCycle;	/**< the pwm-rate (255 * pwm_onTime / pwm_sampleTime) */
std::ofstream file;	/**< file-handler */
};

outputStruct ready, error;

/**
 * describes the current state off another process watched with bond
 */
struct statusStruct {
	int broken;	/**< 1 if the bond is broken, else 0 */
	int formed;	/**< 1 if the bond os formed, else 0 */
	int isReset;	/**< will be set to 1 if the old, broken bond was destructed and a new one was created, is normally 0 */
};

statusStruct sampleStat, driverStat;


void ledCallback(const ros::TimerEvent& event)
{
	if (driverStat.formed == 1 && sampleStat.formed == 1 && driverStat.broken != 1 && sampleStat.broken !=1){
		ready.dutyCycle = 250;
	} else {
		ready.dutyCycle = 0;
	}

	ROS_ERROR("[stateOutput] PWM-ready-DutyCycle (0-255) -> %i", ready.dutyCycle);

	/*if (driverStat.broken == 1 || sampleStat.broken == 1) {
		error.dutyCycle = 250;
	} else {
		error.dutyCycle = 0;
	}

	ROS_ERROR("[stateOutput] PWM-error-DutyCycle (0-255) -> %i", error.dutyCycle);

	error.file.seekp(0);
	error.file << error.dutyCycle;
	error.file.flush();*/

	ready.file.seekp(0);
	ready.file << ready.dutyCycle;
	ready.file.flush();
}

void sampleBroken()
{
	ROS_ERROR("epos2_talker-Node is Broken");
	sampleStat.broken = 1;
	sampleStat.formed = 0;
	sampleStat.isReset = 0;
}

void sampleFormed()
{
	ROS_ERROR("epos2_talker-Node is Formed");
	sampleStat.formed = 1;
	sampleStat.broken = 0;
	sampleStat.isReset = 0;
}

void driverBroken()
{
	ROS_ERROR("epos2_driver-Node is Broken");
	driverStat.broken = 1;
	driverStat.formed = 0;
	driverStat.isReset = 0;
}

void driverFormed()
{
	ROS_ERROR("epos2_driver-Node is Formed");
	driverStat.broken = 0;
	driverStat.formed = 1;
	driverStat.isReset = 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "epos2_state_output");
	ros::NodeHandle roshandle;

	ros::Rate loopRate = 10;
	ros::Timer setLEDstat;
	int filesOpen = 0;

	driverStat.broken = 0;
	driverStat.formed = 0;
	driverStat.isReset = 0;
	sampleStat.broken = 0;
	sampleStat.formed = 0;
	sampleStat.isReset = 0;

	ready.dutyCycle = 0;
	error.dutyCycle = 0;
	ready.file.open("/sys/class/hwmon/hwmon0/device/pwm1", std::ios::out | std::ios::binary);
	error.file.open("/sys/class/hwmon/hwmon0/device/pwm1", std::ios::out | std::ios::binary); //| std::ios::trunc);

	if(ready.file.is_open() && error.file.is_open()) {
		setLEDstat = roshandle.createTimer(ros::Duration(1.5), ledCallback);
		filesOpen = 1;
	} else {
		ROS_ERROR("[stateOutput] couldn't open one or more of the pwm-files. Will continue in text-mode without LED-Output.");
		filesOpen = 0;
	}

	bond::Bond sampleBond("epos2_bond", "sampleMoving", sampleBroken, sampleFormed);
	bond::Bond driverBond("epos2_bond", "motorController", driverBroken, driverFormed);
	sampleBond.setConnectTimeout(90);
	driverBond.setConnectTimeout(90);
	sampleBond.start();
	driverBond.start();

	//ros::spin();
	while (ros::ok())
	{
		if (sampleStat.broken == 1 && sampleStat.isReset == 0) {
			sampleBond.breakBond();
			sampleBond.~Bond();
			new(&sampleBond) bond::Bond("epos2_bond", "sampleMoving", sampleBroken, sampleFormed);
			sampleBond.setConnectTimeout(90);
			sampleBond.start();
			sampleStat.isReset=1;
		}
		if (driverStat.broken == 1 && driverStat.isReset == 0) {
			driverBond.breakBond();
			driverBond.~Bond();
			new(&driverBond) bond::Bond("epos2_bond", "motorController", driverBroken, driverFormed);
			driverBond.setConnectTimeout(90);
			driverBond.start();
			driverStat.isReset=1;
		}
		loopRate.sleep();
		ros::spinOnce();
	}

	sampleBond.breakBond();
	driverBond.breakBond();

	if (filesOpen == 1) {
		error.file.close();
		ready.file.close();
	}

	return 0;
}



