/*
 * TankSteering.cpp
 *
 *  Created on: 25.04.2013
 *      Author: eos
 *      modified by Jannik Flessner
 */

#include "TankSteering.h"
#include <iostream>
#include <fstream>

static	double txtPosition = -200.0f;
static FILE *fp;

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
	//double xTarget = 0.5;
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
		tankSettings.targetVelocityMPS[flow] = 22.0 * z_flow;
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

//@author Jannik Flessner
void TankSteering::flowCallback(const std_msgs::String::ConstPtr& msg)
{
	
  	std::string messageGoal = msg->data.c_str();
	std::string hubDown1 = "down110";
	std::string hubDown2 = "down120";
 	std::string hubUp1 = "up110";
	std::string hubUp2 = "up120";
  	std::string rampeAusgang = "load";
	std::string rampeEingang = "unload";

  	if (messageGoal == rampeAusgang){
		tankSettings.flowControl = 1.0;       
	}    
	else if (messageGoal == rampeEingang){
		tankSettings.flowControl = 2.0;
	}
	else if (messageGoal == hubDown1){
		tankSettings.hubControl = 1.1;
	}
	else if (messageGoal == hubDown2){
		tankSettings.hubControl = 1.2;
	}
	else if (messageGoal == hubUp1){
		tankSettings.hubControl = 2.1;
	}
	else if (messageGoal == hubUp2){
		tankSettings.hubControl = 2.2;
	}
        //ROS_ERROR("Callback: %f", tankSettings.flowControl);	
}

//@author Jannik Flessner
void TankSteering::flowControl(const ros::TimerEvent& event)
{

	int lightValue;
	lightValue = tankSettings.epos[flow]->getLightSensorsValue();
	
	int sensorBack = 0x4000;
	int sensorNo = 0x0000;
	//ROS_ERROR("Lichtschrankenwert: %x", lightValue);
	//tankSettings.flowControl = 1.0;
	std::string loadStatus;
	//ROS_ERROR("Control: %f",tankSettings.flowControl);

	if (tankSettings.flowControl == 1.0 && tankSettings.hubControl == 0.0){
		if (lightValue == sensorBack && loadStatus != "PackageLoaded"){
			//ROS_ERROR("Stop Flow Motor");
			tankSettings.targetVelocityMPS[flow] = 0.0;
			loadStatus = "PackageLoaded";
			tankSettings.flowPub.publish(loadStatus);
			tankSettings.flowControl = 0.0;
		}
		else if (lightValue != sensorBack){
			//ROS_ERROR("Start Flow Motor");
			tankSettings.targetVelocityMPS[flow] = 20.0;
		}
	}


	if (tankSettings.flowControl == 2.0 && tankSettings.hubControl == 0.0){
		if (lightValue == sensorNo && loadStatus != "PackageUnloaded"){
			//ROS_ERROR("Stop Flow Motor");
			tankSettings.targetVelocityMPS[flow] = 0.0;
			loadStatus = "PackageUnloaded";
			tankSettings.flowPub.publish(loadStatus);
			tankSettings.flowControl = 0.0;
		}
		else if (lightValue != sensorNo){
			//ROS_ERROR("Start Flow Motor");
			tankSettings.targetVelocityMPS[flow] = -20.0;
		}
	}
	
}

//@author Jannik Flessner
void TankSteering::hubControl(const ros::TimerEvent& event){

	double stopDown = 0.0;
	double stopUp = 10.0;
	uint8_t micaz[5];
  	std::string out;

	if (tankSettings.hubControl == 1.1){
		if (tankSettings.hubPosition < stopDown){
			ROS_ERROR("Stop Hub Motor");
			tankSettings.targetVelocityMPS[hub] = 0.0;
			tankSettings.hubControl = 0.0;
			micaz[0] = 0x55;
			micaz[1] = 0x01;
			micaz[2] = 0x10;
			micaz[3] = 0x0e;
			micaz[4] = 0x0f;
			char buf[100];
  			sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  			out = buf;
			tankSettings.micazPub.publish(out);
		}
		else if (tankSettings.hubPosition > stopDown){
			tankSettings.targetVelocityMPS[hub] = -tankSettings.maxMPS * 0.75;
		}
	}
	else if (tankSettings.hubControl == 1.2){
		if (tankSettings.hubPosition < stopDown){
			ROS_ERROR("Stop Hub Motor");
			tankSettings.targetVelocityMPS[hub] = 0.0;
			tankSettings.hubControl = 0.0;
			micaz[0] = 0x55;
			micaz[1] = 0x01;
			micaz[2] = 0x20;
			micaz[3] = 0x0e;
			micaz[4] = 0x0f;
			char buf[100];
  			sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  			out = buf;
			tankSettings.micazPub.publish(out);
		}
		else if (tankSettings.hubPosition > stopDown){
			tankSettings.targetVelocityMPS[hub] = -tankSettings.maxMPS * 0.75;
		}
	}

	if (tankSettings.hubControl == 2.1){
		if (tankSettings.hubPosition >= stopUp){
			ROS_ERROR("Stop Hub Motor");
			tankSettings.targetVelocityMPS[hub] = 0.0;
			tankSettings.hubControl = 0.0;
			micaz[0] = 0x50;
			micaz[1] = 0x01;
			micaz[2] = 0x10;
			micaz[3] = 0x0e;
			micaz[4] = 0x0f;
			char buf[100];
  			sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  			out = buf;
			tankSettings.micazPub.publish(out);
		}
		else if (tankSettings.hubPosition < stopUp){
			tankSettings.targetVelocityMPS[hub] = tankSettings.maxMPS * 0.75;
		}
	}
	else if (tankSettings.hubControl == 2.2){
		if (tankSettings.hubPosition >= stopUp){
			ROS_ERROR("Stop Hub Motor");
			tankSettings.targetVelocityMPS[hub] = 0.0;
			tankSettings.hubControl = 0.0;
			micaz[0] = 0x50;
			micaz[1] = 0x01;
			micaz[2] = 0x20;
			micaz[3] = 0x0e;
			micaz[4] = 0x0f;
			char buf[100];
  			sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  			out = buf;
			tankSettings.micazPub.publish(out);
		}
		else if (tankSettings.hubPosition < stopUp){
			tankSettings.targetVelocityMPS[hub] = tankSettings.maxMPS * 0.75;
		}
	}

}

//modified by @author Jannik Flessner, Raschid Alkhatib
void TankSteering::setVelocityCallback(const ros::TimerEvent& event)
{
        
	double targetVelocityRPM[4];
	double absolutePosition;
	double diffPosition;
	char strInput[256];
	
	//ROS_ERROR("Data test");
	/*FILE * fileTest;
	fileTest = fopen("/home/pg/Position.txt","r");
	
	if(fileTest==NULL){
		std::fstream file;
		file.open("/home/pg/Position.txt", std::ios::out);
		file << "0";
		file.close();
	}
		else {
		//ROS_ERROR("File is available");
	}*/
	
	
	for (int i=left; i <=flow; i++) {targetVelocityRPM[i] = tankSettings.factorMPSToRPM * tankSettings.targetVelocityMPS[i];}

	ROS_DEBUG("Try to set Velocity left to %.3f rpm and right to %.3f rpm", targetVelocityRPM[left], targetVelocityRPM[right]);
	ROS_INFO("Try to set Velocity left to %.4f m/s and right to %.4f m/s", tankSettings.targetVelocityMPS[left], tankSettings.targetVelocityMPS[right]);
	
	std::fstream file;
	if(txtPosition == -200.0f){	
		file.open("/home/pg/Position.txt", std::ios::in);
		file.getline(strInput, sizeof(strInput));
		file.close();

		txtPosition = strtod(strInput, NULL);
	}

	//ROS_ERROR("%f", targetVelocityRPM[flow]);
	for (int i=left; i <=flow; i++) {
		
		absolutePosition = tankSettings.epos[hub]-> getAbsolutePosition();
		
		tankSettings.epos[i]->changeRotationPerMinute(targetVelocityRPM[i]);
		diffPosition = (tankSettings.epos[hub]-> getAbsolutePosition()) - absolutePosition;
		//ROS_ERROR("Diff: %f", diffPosition);

		if (i == hub && diffPosition != 0){
			txtPosition = txtPosition + diffPosition;
			
			fp = fopen("/home/pg/Position.txt", "w");
			char out[256];
			
			
			if(fp != NULL){
				sprintf(out, "%f", txtPosition);
				fputs(out, fp);
				fclose(fp);
				tankSettings.hubPosition = txtPosition;
				ROS_ERROR("%f", tankSettings.hubPosition);
			} else {
				ROS_ERROR("Data not written");
			}
		}
		
		
	}
	
	tankSettings.epos[flow]->testForErrorsAndPrint();
	//double abposi = tankSettings.epos[hub] ->getAbsolutePosition();
	//ROS_ERROR("%f", abposi);
	
		

}

//modified by @author Jannik Flessner
void TankSteering::odomCallback(const ros::TimerEvent& event)
{
	double pos_linear_z=0;
	double pos_delta[2], temp_pos[2];
	tf::StampedTransform readTransform;
	geometry_msgs::TransformStamped transform;
	nav_msgs::Odometry nav_odom;

	for (int i=left; i<=right; i++) { 
	  temp_pos[i] = tankSettings.epos[i]->getAbsolutePosition() * tankSettings.wheelPerimeter; //Bestimmung der zurueckgelegten Strecke beider Raeder
	  //ROS_ERROR("%f",temp_pos[i]);
	}
	for (int i=left; i<=right; i++) {
		pos_delta[i] = temp_pos[i] - pos.lastPosition[i]; // Unterschied von jetziger zur vorheriger Position
		pos.lastPosition[i] = temp_pos[i];
	}

	float timeDelta = (event.current_real - event.last_real).toSec(); // Vergangene Zeit zwischen der Messung
	double polar_s  =(pos_delta[right] + pos_delta[left] ) * 0.5; // Berechnung der Weglaenge
	double polar_theta=(pos_delta[right] - pos_delta[left]) / tankSettings.axisLength; // Berechnung der Drehung
	
	try {
		tankSettings.tfListener.lookupTransform("/odom", "/base_link", ros::Time(0), readTransform);
		pos.last.x = readTransform.getOrigin().x();
		pos.last.y = readTransform.getOrigin().y();
		pos.last.theta = tf::getYaw(readTransform.getRotation());
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
	
	pos.now.x = pos.last.x + polar_s * cos(pos.last.theta + polar_theta);
	pos.now.y = pos.last.y + polar_s * sin(pos.last.theta + polar_theta);
	
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
// Test
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


//modified by @author Jannik Flessner
int TankSteering::init()
{
	int j = 0;
	int active[4] = {0,0,0,0};
	Epos2MotorController::EposState state = Epos2MotorController::disabled;

	for (int i = left; i <= flow; i++) { tankSettings.epos[i]->testForErrorsAndPrint(); }
	resetEPOS2Fault();
	for (int i=left; i<=hub;i++) {tankSettings.epos[i]->init(tankSettings.maxRPM, tankSettings.wheelPerimeter);}

	tankSettings.epos[flow]->initFlow(tankSettings.maxRPM, tankSettings.wheelPerimeter);
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

		if ( active[left] == 1 && active[right] == 1  && active[hub] == 1 && active[flow] == 1) {
			// Abhoeren von SteuerNachrichten
			tankSettings.epos2MPS = tankSettings.roshandle.subscribe("epos2_MPS_left_right", 1000, &TankSteering::driveCallbackMPS, this);
			tankSettings.epos2Twist = tankSettings.roshandle.subscribe("cmd_vel", 1000, &TankSteering::driveCallback, this);
			tankSettings.flowMessage = tankSettings.roshandle.subscribe("flow", 10, &TankSteering::flowCallback, this);

			for (int i=left; i<=right; i++) { pos.lastPosition[i]=tankSettings.epos[i]->getAbsolutePosition() * tankSettings.wheelPerimeter; }

			tankSettings.odomTimer = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.odomDuration), &TankSteering::odomCallback, this);
			tankSettings.flowTimer = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.velocityDuration), &TankSteering::flowControl, this);
			tankSettings.hubTimer = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.velocityDuration), &TankSteering::hubControl, this);
			tankSettings.setVelocity = tankSettings.roshandle.createTimer(ros::Duration(tankSettings.velocityDuration), &TankSteering::setVelocityCallback, this);
			tankSettings.odomPub = tankSettings.roshandle.advertise<nav_msgs::Odometry>("odom", 50);
			tankSettings.flowPub = tankSettings.roshandle.advertise<std_msgs::String>("flow", 10);
			tankSettings.micazPub = tankSettings.roshandle.advertise<std_msgs::String>("uc1Command", 1000);

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

