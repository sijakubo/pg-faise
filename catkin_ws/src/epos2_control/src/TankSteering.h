/*
 * TankSteering.h
 *
 *  Created on: 25.04.2013
 *      Author: eos
 *      modified by Jannik Flessner
 */

/**
 * TankSteering-Class.
 * It's used to control two EPOS2 while each one drive on side of a robot. Steering will be done by a difference between the velocities for both sides.
 */

#ifndef TANKSTEERING_H_
#define TANKSTEERING_H_

#include "Epos2MotorController.h"
#include "epos2_control/velocity.h"		//structure for callback
#include "geometry_msgs/Twist.h"	//ros-structure for publishing velocity information
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TankSteering {

		/**
		 * Settings Structure with all relevant values and ROS.
		 **/
		struct tankSet {
			double maxRPM;	/**< [rev/min] highest allowed angular velocity of the robot */
			double maxMPS;	/**< [m/s] highest allowed velocity of the robot */
			double maxSafetyMPS;	/**< [m/s] highest allowed velocity if a connected laser scanner detect obstacles in sight. it's equal or less maxMPS. */
			double factorMPSToRPM;	/**< [-] factor to convert a translational velocity (m/s) to an angular velocity (rev/min) depending on the wheelPerimeter */
			double wheelPerimeter;	/**< [m] the perimeter of one wheel - all motor-driven wheels should have the same perimeter and radius */
			double axisLength;	/**< [m] the distance between two wheels with the same axis of rotation (on opposite sites of the robot) */
			Epos2MotorController* epos[4];	/**< two instances of the Epos2MotorController class - one for each side (left/right) */

			ros::NodeHandle roshandle; /**< the handle on a ROSNode to open Timers, Subscribers, Publishers... */

			ros::Subscriber flowMessage;    /**< object for listing to goal messages (flow) */
			ros::Subscriber epos2Twist;	/**< object for listing to twist messages (cmd_vel) */
			ros::Subscriber epos2MPS;	/**< object for listing to speed messages (one value for each side of the robot) */
			ros::Subscriber sickLMS;	/**< a object for listing to point-cloud-messages from the laser scanner */
			ros::Timer odomTimer;	/**< a timer object to call the function which calculates the current position */
			ros::Timer flowTimer;
			ros::Timer hubTimer;
			ros::Timer setVelocity;	/**< a timer object to call the function which sets the current velocity value to the EPOS2 */
			ros::Publisher odomPub;	/**< object for sending the current position to the ROS NavStack */
			ros::Publisher flowPub;	/**< object for sending to goal messages (uc1Response) */		

			ros::Timer getErrorLoop;	/**< a timer object to call a function which reads out both EPOS2 error historys */

			tf::TransformListener tfListener;	/**< object to get coordinate transformations from tf-framework */
			tf::TransformBroadcaster tfBroad;	/**< object to send coordinates with respective to a given system to the tf-frameork */

			double targetVelocityMPS[4];	/**< [m/s] the target velocity for both sides */
			double odomDuration;	/**< [s] the time between to calls of the odomCallback() */
			double velocityDuration;	/**< [s] the time between to calls of the setVelocityCallback() */
			double flowControl;
			double hubControl;
		}tankSettings;	/**< Main Settings for tank-steering a robot with two EPOS2 */

		/**
		 * structure with all values to restrict the velocity depending on the laser-scanner-data.
		 * the velocity decrease quadratic (v=a*x^2+b*x+c) depending on the minimum distance (x)
		 */
		struct distanceStruct {
			double a;	/**< the factor before the x^2 */
			double b;	/**< the factor before the x*/
			double c;	/**< the absolute value of the equation */
			double high;	/**< [m] the distance with velocity 0 m/s */
			double low;	/**< [m] the distance where the restriction of velocity starts */
			int range;	/**< [degree] the angle on which laser-scanner-data will be checked on obstacles */
		}LaserDistance;	/**< Settings to restrict the velocity in front of obstacles */

		/**
		 * describing the orientation of the robot in a 2D-case depending to a parent-frame
		 */
		struct orientation {
			double x;	/**< the x-coordinate */
			double y;	/**< the y-coordinate */
			double theta;	/**< the rotation of the robot */
		};

		/**
		 * structure with all data to calculate the current position depending on odometry-data
		 */
		struct odom {
			double lastPosition[2];	/**< the last position of both motors connected to the EPOS2 */
			orientation now;	/**< the current position */
			orientation last;	/**< the position one step before */
		}pos;	/**< Temporary memory for the odometry-calculation */

	public:

		/**
		 * enumeration for naming the EPOS2s
		 */
		enum {left, right, hub, flow};

		/**
		 * constructor of TankSteering-Class. Initialized all values without the laser-scanner-data.
		 *
		 * @param roshandle handle from ROS to open Timer, Subscriber and Publisher
		 * @param epos two instances of the Epos2MotorController-class. One for each side of the robot with the following order: left, right.
		 * @param wheelPerimeter [m] the perimeter of one wheel - all motor-driven wheels should have the same perimeter and radius
		 * @param axisLength [m] the distance between two wheels with the same axis of rotation (on opposite sites of the robot)
		 * @param maxMPS [m/s] highest allowed velocity of the robot
		 * @param odomDuration [s] the time between to calls of the odomCallback()
		 * @param velocityDuration [s] the time between to calls of the setVelocityCallback()
		 */
		TankSteering(ros::NodeHandle roshandle, Epos2MotorController* epos[4], double wheelPerimeter, double axisLength, double maxMPS, double odomDuration = 0.05, double velocityDuration = 0.07);
		/**
		 * Destructor of the TankSteering-class.
		 * Nothing to do by now.
		 */
		~TankSteering();

	private:

		/**
		 * velocity-Callback-function - will be called if a speed-message (velocity left and right) was sent
		 *
		 * @param speed contains a velocity for each side of the robot
		 */
		void driveCallbackMPS(const epos2_control::velocity speed);
		/**
		 * velocity-Callback-function - will be called if a twist message was sent
		 *
		 * @param velocityVector contains velocities in each direction of space and angular velocities around each axis
		 */
		void driveCallback(const geometry_msgs::Twist velocityVector);
		/**
		 * flowControl-callback-function - Control flow to load/unload robot
		 *
		 * @param string contains stop/load/unload command
		 */
		void flowCallback(const std_msgs::String::ConstPtr& msg);
		/**
		 * velocity-callback-function - every time called it will set the targetVelocity to both EPOS2
		 *
		 * @param event contains some time values according to the event
		 */

		void flowControl(const ros::TimerEvent& event);

		void hubControl(const ros::TimerEvent& event);

		void setVelocityCallback(const ros::TimerEvent& event);
		/**
		 * odom-callback-function - reads out the current position of both motors connected to the EPOS2s and calculate the odometry-data
		 *
		 * @param event contains some time values according to the event
		 */
		void odomCallback(const ros::TimerEvent& event);
		/**
		 * laser-scanner-callback - will be called if the laser-scanner-node send new point-cloud-data
		 *
		 * @param sensorData contains all points measured by the laser-scanner and the distance
		 */
		void LaserScanCallback(const sensor_msgs::LaserScan sensorData);
		/**
		 * error-callback - will be called continuously and check both EPOS2 on Errors.
		 * if an error is detected both EPOS2 will be disable and afterwards reseted
		 *
		 * @param event contains some time values according to the event
		 */
		void getDeviceErrorCallback(const ros::TimerEvent& event);
		/**
		 * Initialized both EPOS2 and setup all ROS components.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int init();
		/**
		 * bring the EPOS2 into a drive-able state. this function first reset both EPOS2 and clear afterwards the error history.
		 */
		void resetEPOS2Fault();

	public:

		/**
		 * Initialized the parameters for using the laser-scanner to avoid crashing into obstacles in front of the robot.
		 *
		 * @param range [degree] the angle on which laser-scanner-data will be checked on obstacles
		 * @param low [m] the distance with velocity 0 m/s
		 * @param high [m] the distance where the restriction of velocity starts
		 */
		void initLaserscanner(int range = 180, double low = 0.15, double high = 0.6);
		/**
		 * return if a value is positive, zero or negative
		 *
		 * @param number the value which should be analyzed
		 * @return "-1" if negative, "0" if zero and "1" if positive
		 */
		int sign(double number);

};

#endif /* TANKSTEERING_H_ */












