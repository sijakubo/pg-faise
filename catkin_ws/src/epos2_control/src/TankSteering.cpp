/*
 * TankSteering.cpp
 *
 *  Created on: 25.04.2013
 *      Author: eos
 *      modified by Jannik Flessner
 */

#include "TankSteering.h"

//public

TankSteering::TankSteering(ros::NodeHandle roshandle, Epos2MotorController* epos[4], double wheelPerimeter, double axisLength, double maxMPS, double odomDuration, double velocityDuration)
{
	tankSettings.maxMPS = maxMPS;
	tankSettings.odomDuration = odomDuration;
	tankSettings.velocityDuration = velocityDuration;
	tankSettings.axisLength = axisLength;
	tankSettings.wheelPerimeter = wheelPerimeter;
	tankSettings.factorMPSToRPM = 60 / tankSettings.wheelPerimeter;
	tankSettings.maxRPM = tankSettings.maxMPS * tankSettings.factorMPSToRPM;

	ROS_ERROR("TAnkSteering: maxMPS: %.4lf, factorMPSToRPM: %.4lf, maxRPM: %.4lf", tankSettings.maxMPS, tankSettings.factorMPSToRPM, tankSettings.maxRPM);

	tankSettings.roshandle = roshandle;
	for (int i=left; i<=flow;i++) {
		tankSettings.epos[i] = epos[i];
	}

	pos.now.x = pos.now.y =	pos.now.theta = 0;
	pos.last.x = pos.last.y = pos.last.theta = 0;

	tankSettings.maxSafetyMPS = tankSettings.maxMPS;

	init();
}

TankSteering::~TankSteering()
{

}

void TankSteering::driveCallbackMPS(const epos2_control::velocity speed)
{
	tankSettings.targetVelocityMPS[left] = speed.left;
	tankSettings.targetVelocityMPS[right] = speed.right;
}

void TankSteering::driveCallback(const geometry_msgs::Twist velocityVector)
{
	double dxTarget = tankSettings.axisLength * velocityVector.angular.z / 2;
	double xTarget = velocityVector.linear.x;
	double y_hub = velocityVector.linear.y;
	double z_flow = velocityVector.linear.z;

	//ROS_ERROR("linear.x: %.6f ; angular.z: %.6f", velocityVector.linear.x, velocityVector.angular.z);	//debugging

	if ( fabs(tankSettings.maxSafetyMPS) > 0.01 ) {
		//forward velocity restriction
		if ( tankSettings.maxMPS < (fabs(dxTarget) + fabs(xTarget)) ) { xTarget = (tankSettings.maxMPS - fabs(dxTarget)) * sign(xTarget); }

		double x = (xTarget > tankSettings.maxSafetyMPS) ? tankSettings.maxSafetyMPS : xTarget;

		//rotation velocity restriction
		double dx = (fabs(dxTarget) > tankSettings.maxSafetyMPS) ? tankSettings.maxSafetyMPS * sign(dxTarget) : dxTarget;

		//ROS_ERROR("x: %.4f ; dx: %.4f", x, dx);	//debugging

		//output velocities
		tankSettings.targetVelocityMPS[left]  = x - dx;
		tankSettings.targetVelocityMPS[right] = x + dx;
		tankSettings.targetVelocityMPS[hub] = tankSettings.maxMPS * y_hub;
		tankSettings.targetVelocityMPS[flow] = tankSettings.maxMPS * z_flow;
		//tankSettings.targetVelocityMPS[flow] = tankSettings.targetVelocityMPS[hub];
		//ROS_ERROR("TankSteering:maxMPS: %.4lf",tankSettings.targetVelocityMPS[left]);
	} else {
		ROS_ERROR("No movement possible because the maximum safety Velocity is to low.");
		tankSettings.targetVelocityMPS[left]  = 0;
		tankSettings.targetVelocityMPS[right]  = 0;
		tankSettings.targetVelocityMPS[hub] = 0;
		tankSettings.targetVelocityMPS[flow] = 0;
	}
}

void TankSteering::setVelocityCallback(const ros::TimerEvent& event)
{
	double targetVelocityRPM[4];

	for (int i=left; i <=hub; i++) {targetVelocityRPM[i] = tankSettings.factorMPSToRPM * tankSettings.targetVelocityMPS[i];}

	ROS_DEBUG("Try to set Velocity left to %.3f rpm and right to %.3f rpm", targetVelocityRPM[left], targetVelocityRPM[right]);
	ROS_INFO("Try to set Velocity left to %.4f m/s and right to %.4f m/s", tankSettings.targetVelocityMPS[left], tankSettings.targetVelocityMPS[right]);

	for (int i=left; i <=hub; i++) {tankSettings.epos[i]->changeRotationPerMinute(targetVelocityRPM[i]);}
}

void TankSteering::odomCallback(const ros::TimerEvent& event)
{
	double pos_linear_z=0;
	double pos_delta[2], temp_pos[2];
	tf::StampedTransform readTransform;
	geometry_msgs::TransformStamped transform;
	nav_msgs::Odometry nav_odom;

	for (int i=left; i<=right; i++) { temp_pos[i] = tankSettings.epos[i]->getAbsolutePosition() * tankSettings.wheelPerimeter; }
	for (int i=left; i<=right; i++) {
		pos_delta[i] = temp_pos[i] - pos.lastPosition[i];
		pos.lastPosition[i] = temp_pos[i];
	}

	float timeDelta = (event.current_real - event.last_real).toSec();
	double polar_s  =(pos_delta[right] + pos_delta[left] ) * 0.5;
	double polar_theta=(pos_delta[right] - pos_delta[left]) / tankSettings.axisLength;
	double delta_r = (polar_theta == 0) ? polar_s : (polar_s / polar_theta) * sin(polar_theta/2) * 2;

	try {
		tankSettings.tfListener.lookupTransform("/odom", "/base_link", ros::Time(0), readTransform);
		pos.last.x = readTransform.getOrigin().x();
		pos.last.y = readTransform.getOrigin().y();
		pos.last.theta = tf::getYaw(readTransform.getRotation());
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

	pos.now.x = pos.last.x + delta_r * cos(polar_theta/2 + pos.last.theta);
	pos.now.y = pos.last.y + delta_r * sin(polar_theta/2 + pos.last.theta);
	pos.now.theta = pos.last.theta + polar_theta;

	//odom
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pos.now.theta);

	transform.header.stamp = event.current_real;
	transform.header.frame_id = "odom";
	transform.child_frame_id = "base_link";

	transform.transform.translation.x = pos.now.x;
	transform.transform.translation.y = pos.now.y;
	transform.transform.translation.z = pos_linear_z;
	transform.transform.rotation = odom_quat;

	tankSettings.tfBroad.sendTransform(transform);

	nav_odom.header.stamp = event.current_real;
	nav_odom.header.frame_id = "odom";
	nav_odom.pose.pose.position.x = pos.now.x;
	nav_odom.pose.pose.position.y = pos.now.y;
	nav_odom.pose.pose.position.z = pos_linear_z;
	nav_odom.pose.pose.orientation = odom_quat;
	nav_odom.twist.twist.linear.x = polar_s / timeDelta;
	nav_odom.twist.twist.linear.y = 0;
	nav_odom.twist.twist.linear.z = 0;
	nav_odom.twist.twist.angular.x = 0;
	nav_odom.twist.twist.angular.y = 0;
	nav_odom.twist.twist.angular.z = polar_theta / timeDelta;
	tankSettings.odomPub.publish(nav_odom);

	pos.last.x = pos.now.x;
	pos.last.y = pos.now.y;
	pos.last.theta = pos.now.theta;

	//ROS_ERROR("last pos (x, y, phi): %.4lf, %.4lf, %.4lf", pos.last.x, pos.last.y, pos.last.theta);
	//ROS_ERROR("new pos (x, y, phi): %.4lf, %.4lf, %.4lf", pos.now.x, pos.now.y, pos.now.theta);

	//ROS_INFO("velocity left:  %.4f  and right:  %.4f",  ((float)(pos_delta[left] )/timeDelta), ((float)(pos_delta[right])/timeDelta) );

	//float odomTime = (ros::Time::now() - event.current_real).toSec();
	//ROS_DEBUG("time taken by odom: %.5f sec", odomTime);

	//sicklmsCounter++;	//testing laserscanner
}

void TankSteering::LaserScanCallback(const sensor_msgs::LaserScan sensorData)
{
	double minValue = sensorData.range_max;
	int points = 0, start = 0;

	points = ( abs(sensorData.ranges.size()) > 180 ) ? 180 : abs(sensorData.ranges.size());

	if (points > LaserDistance.range) {
		start = (int)((points - LaserDistance.range) / 2);	//quick & dirty ... only produce even ranges - if LaserDistance.range is uneven it behaves the same as the next lower even integer
	}

	for (int i=start; i<points-start; i++) {
		if (sensorData.ranges[i] < minValue) {minValue = sensorData.ranges[i];}
	}

	if (minValue < LaserDistance.high) {
		if (minValue > LaserDistance.low) {
			tankSettings.maxSafetyMPS = LaserDistance.a * pow(minValue,2) + LaserDistance.b * minValue + LaserDistance.c;
		} else {
			tankSettings.maxSafetyMPS = 0.5;
		}
	}else{
		tankSettings.maxSafetyMPS = tankSettings.maxMPS;
	}

	/*if (sicklmsCounter >= 20) {
		sicklmsCounter=0;
		ROS_ERROR("seq: %i, Nearest: %.4lf, maxVel: %.4lf, VelTrans: %.4lf, VelRot: %.4lf", sensorData.header.seq, minValue, maxSafetyForwardVelocity, cmdVelTimerSet.x, cmdVelTimerSet.dx);
	}*/
}

void TankSteering::getDeviceErrorCallback(const ros::TimerEvent& event)
{
	int error[2];

	for (int i = left; i <= right; i++) { error[i] = tankSettings.epos[i]->testForErrorsAndPrint(); }

	if (error[left] == 0 || error[right] == 0) {
		for (int i = left; i <= right; i++) { tankSettings.epos[i]->disableEpos2();}
		resetEPOS2Fault();
	}
}


// Hier werden die Nachrichten abgehoert und Callbacks gestartet
int TankSteering::init()
{
	int j = 0;
	int active[4] = {0,0,0,0};
	Epos2MotorController::EposState state = Epos2MotorController::disabled;

	for (int i = left; i <= flow; i++) { tankSettings.epos[i]->testForErrorsAndPrint(); }
	resetEPOS2Fault();
	for (int i=left; i<=flow;i++) {tankSettings.epos[i]->init(tankSettings.maxRPM, tankSettings.wheelPerimeter);}
	ROS_INFO("initialize both EPOS2 successfully");

	j = 0;
	do {
		if (tankSettings.epos[left]->getEpos2State() == Epos2MotorController::enabled && tankSettings.epos[right]->getEpos2State() == Epos2MotorController::enabled && tankSettings.epos[hub]->getEpos2State() == Epos2MotorController::enabled && tankSettings.epos[flow]->getEpos2State() == Epos2MotorController::enabled) {
			state = Epos2MotorController::enabled;
			if (j == 0) { for (int i=left; i<=flow;i++) {tankSettings.epos[i]->enableEpos2();} }
		}
		j++;
	} while ( state != Epos2MotorController::enabled && j < 6);

	if (state == Epos2MotorController::enabled) {

		for (int i=left; i<=flow; i++) {
			active[i] = tankSettings.epos[i]->activateProfileVelocity();
		}

		if ( active[left] == 1 && active[right] == 1  && active[hub] == 1  && active[flow] == 1 ) {
			// Abhoeren von SteuerNachrichten
			tankSettings.epos2MPS = tankSettings.roshandle.subscribe("epos2_MPS_left_right", 1000, &TankSteering::driveCallbackMPS, this);
			tankSettings.epos2Twist = tankSettings.roshandle.subscribe("cmd_vel", 1000, &TankSteering::driveCallback, this);

			for (int i=left; i<=right; i++) { pos.lastPosition[i]=tankSettings.epos[i]->getAbsolutePosition() * tankSettings.wheelPerimeter; }

			tankSettings.odomTimer = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.odomDuration), &TankSteering::odomCallback, this);
			tankSettings.setVelocity = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.velocityDuration), &TankSteering::setVelocityCallback, this);
			tankSettings.odomPub = tankSettings.roshandle.advertise<nav_msgs::Odometry>("odom", 50);

			tankSettings.getErrorLoop = tankSettings.roshandle.createTimer(ros::Duration(1.02), &TankSteering::getDeviceErrorCallback, this);

			ROS_INFO("Epos2-TankSteering is now listing for velocity messages");
			return 1;

		} else {
			ROS_ERROR("at least one Epos2 is not in ProfileVelocityMode - no movement is possible");
			return 0;
		}

	} else {
		ROS_FATAL("Unable starting motors, the EPOS2 are not enabled. (Maybe Power-off or Unplugged?)");

		return 0;
	}//end of state is enable
}

void TankSteering::initLaserscanner(int range, double low, double high)
{
	//sicklmsCounter=0;	//testing laser-scanner
	int n=3;

	LaserDistance.low = low;
	LaserDistance.high = high;
	LaserDistance.range = range;

	LaserDistance.a = 2 * tankSettings.maxMPS * (n - 2) / ( n * pow((LaserDistance.low - LaserDistance.high),2) );
	LaserDistance.b = tankSettings.maxMPS / (LaserDistance.high - LaserDistance.low) - LaserDistance.a * (LaserDistance.low + LaserDistance.high);
	LaserDistance.c = -1 * (LaserDistance.a * pow(LaserDistance.low,2) + LaserDistance.b * LaserDistance.low);

	tankSettings.sickLMS = tankSettings.roshandle.subscribe("scan", 100, &TankSteering::LaserScanCallback, this);

	//ROS_ERROR("Laser-Scanner successfully initialized (Parameter-> a: %.4lf, b: %.4lf, c: %.4lf)", LaserDistance.a, LaserDistance.b, LaserDistance.c);
}

void TankSteering::resetEPOS2Fault()
{
	for (int i=left; i<=right;i++) {tankSettings.epos[i]->resetDevice();}
	ROS_INFO("reset EPOS2 successfully");
	for (int i=left; i<=right;i++) {tankSettings.epos[i]->clearFault();}
	ROS_INFO("clear EPOS2 fault successfully");
}

int TankSteering::sign(double number)
{
	return (number>0) ? 1 : (number<0) ? -1 : 0;
}

