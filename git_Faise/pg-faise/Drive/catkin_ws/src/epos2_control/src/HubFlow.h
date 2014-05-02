/*
 * HubFlow.h
 */

/**
 * HUBFLOW-Class.
 * It's used to control two EPOS2 (Hub/Flow)
 */

#ifndef HUBFLOW_H_
#define HUBFLOW_H_

#include "Epos2MotorController.h"
#include "epos2_control/velocity.h"		//structure for callback
#include "geometry_msgs/Twist.h"	//ros-structure for publishing velocity information

class HubFlow {

		/**
		 * Settings Structure with all relevant values and ROS.
		 **/
		struct hubflowSet {
			double maxRPM;	/**< [rev/min] highest allowed angular velocity of the robot */
			double maxMPS;	/**< [m/s] highest allowed velocity of the robot */
			double maxSafetyMPS;	/**< [m/s] highest allowed velocity if a connected laser scanner detect obstacles in sight. it's equal or less maxMPS. */
			double factorMPSToRPM;	/**< [-] factor to convert a translational velocity (m/s) to an angular velocity (rev/min) depending on the wheelPerimeter */
			Epos2MotorController* epos[4];	/**< two instances of the Epos2MotorController class (hub/flow) */

			ros::NodeHandle roshandle; /**< the handle on a ROSNode to open Timers, Subscribers, Publishers... */

			//ros::Subscriber epos2Twist;	/**< object for listing to twist messages (cmd_vel) */
			ros::Subscriber epos2hubflow;
									/**< object for listing to speed messages (one value for each side of the robot) */
			ros::Timer setVelocity;	/**< a timer object to call the function which sets the current velocity value to the EPOS2 */



			double targetVelocityMPS[2];	/**< [m/s] the target velocity for both sides */
			double velocityDuration;	/**< [s] the time between to calls of the setVelocityCallback() */
		}hubflowSettings;	/**< Main Settings for HubFlow */


	public:

		/**
		 * enumeration for naming the EPOS2s
		 */
		enum {left, right, hub, flow};

		/**
		 * constructor of HubFlow-Class.
		 *
		 * @param roshandle handle from ROS to open Timer, Subscriber and Publisher
		 * @param epos two instances of the Epos2MotorController-class. One for each side of the robot with the following order: hub, flow.
		 * @param maxMPS [m/s] highest allowed velocity of the robot
		 * @param velocityDuration [s] the time between to calls of the setVelocityCallback()
		 */
		HubFlow(ros::NodeHandle roshandle, Epos2MotorController* epos[4], double maxMPS, double velocityDuration = 0.07);
		/**
		 * Destructor of the HubFlow-class.
		 * Nothing to do by now.
		 */
		~HubFlow();

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
		//void driveCallback(const geometry_msgs::Twist velocityVector);
		/**
		 * velocity-callback-function - every time called it will set the targetVelocity to both EPOS2
		 *
		 * @param event contains some time values according to the event
		 */
		void setVelocityCallback(const ros::TimerEvent& event);
		/**
		 * Initialized both EPOS2 and setup all ROS components.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int init();

		void resetEPOS2Fault();


};

#endif /* HUBFLOW_H_ */













