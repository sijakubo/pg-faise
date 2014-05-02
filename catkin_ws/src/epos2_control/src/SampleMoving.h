/*
 * SampleMoving.h
 *
 *  Created on: Apr 11, 2012
 *      Author: Martin Seidel
 */

#ifndef SAMPLEMOVING_H_
#define SAMPLEMOVING_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "epos2_control/velocity.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <tf/transform_broadcaster.h>
//#include <move_base_msgs/MoveBaseAction.h>			//actionlib for sending goals to the navStack
#include <actionlib/client/simple_action_client.h>


#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define pi 3.14159265

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ActionClient;

/**
 * SampleMoving-Class.
 * It's used to send velocitys by twist-messages or speed-messages and the wanted position by tf-framework.
 * The messages will be generated from a gamepad, a steering wheel, the keyboard or automatically.
 */

class SampleMoving {

		/**
		 * enumeration for naming the EPOS2s
		 */
		enum {left, right};
		/**
		 * internal state which kind of humanInterface was detected (gamepads / wheels)
		 */
		enum interfaceType
		{
			noJoy, /**< no joy_node-Tool (wheel or gamepad) was detected - only in this mode keyboard input is allowed */
			gamePad,	/**< a gamepad was detected */
			wheel	/**< steering wheel with two pedals, 2 or 3 axis in joy_msg */
		};

		/**
		 * state which indicates the current connected gamepad-mode
		 */
		enum gamePadMode
		{
			twoStick,	/**< left Stick back- and forward; right stick left and right */
			oneStick,	/**< like twoStick but both axis (left - right & back - forward) are on the left Stick */
			tankStick,	/**< left stick = left side; right stick = right side */
		};

		ros::NodeHandle roshandle; /**< the handle on a ROSNode to open Timers, Subscribers, Publishers... */

		//ActionClient action;	/**< for the navStack - until now not under ROS::groovy */

		/**
		 * describing the orientation of the robot in a 2D-case depending to a parent-frame
		 */
		struct orientation {
			double x;	/**< the x-coordinate */
			double y;	/**< the y-coordinate */
			double theta;	/**< the rotation of the robot */
		};
		/**
		 * structure with orientation-data calculated with the velocity-data sent to the TankSteering-class by ROS-messages
		 */
		struct odom {
			orientation now;	/**< the current position */
			orientation last;	/**< the position one step before */
		}pos;	/**< Temporary memory for the calculation of the wanted position */

		/**
		 * structure with all relevant driving informations of a wheel or gamepad. Used because of the different types of gamepads and wheels.
		 */
		struct axesStruct {
			double leftx;	/**< leftStick: left - right */
			double lefty;	/**< leftStick: up - down */
			double rightx;	/**< rightStick: left - right */
			double righty;	/**< rightStick: up - down */
			int switchButton;	/**< if set to one and a gamepad was detected, the gamepad-mode will be switched */
			int quickStopButton;	/**< if set, all velocity's will be set to zero */
		};

	public:
		/**
		 * Enumeration for the input parameter getInfoFrom.
		 */
		enum inputDevice
		{
			nodrive,	/**< nothing chosen - nothing will happen */
			sinus,	/**< the velocity of the motors connected to the EPOS2 will change in this way: v=sin(t) */
			humanInterface	/**< you can either use a gamepad or the keyboard of your PC to control the EPOS2s */
		};
		/**
		 * Enumeration to describe the type of sending messages over ROS to the Epos2MotorController-Class-Instances.
		 * The position will be send in each case additional by the tf-framework.
		 */
		enum msgTypes
		{
			noMsg,	/**< no message type was chosen - it will automatically switch to "SI" */
			cmdVel,	/**< twist-messages will be send */
			SI,	/**< velocitys separated for left and right EPOS2 will be send */
			navStack,	/**< velocity-messages will be send with the actionlib-MoveBaseGoal - not working by now on ros::groovy */
			onlyTF	/**< only shows the wanted position of the robot with respective to the world-coordinate-frame */
		};

	private:

		/**
		 * Settings Structure with all relevant values and ROS.
		 */
		struct sampleMovingSettingStruct {
			double SpeedStepSize; /**< [1] with respective to the maximum speed */
			double refreshRate;	/**< [1/s] with this rate a new speed-message will be send */
			int sineAngle;	/**< [degree] Angular offset between right and left EPOS in input-mode = sinus */
			int sineCurrentAngle;	/**< [degree] this value will be increased with every time a new velocity was set. only in sinus-mode. should be every time between 0 and 360 */
			interfaceType joyControll;	/**< indicates if a gamepad, steering wheel or no humanInterface is connected */
			int gamepadMode;	/**< mode of connected gamepad. normally is twoStick */
			int joyInUse;	/**< indicates if the gamepad or wheel is used currently. Blocks the Keyboard if set to 1. */
			int ableToMove;	/**< for initializing in humanInterface-mode. used for steering wheel with 3 axis - first both pedals must be pressed down for starting */
			msgTypes msgType;	/**< the type of a published message */

			double wheelBase;	/**< [m] distance between the middle of the last and the first wheel on one side of the robot. will be read from the parameter-server. */
			double axisLength;	/**< [m] distance between both wheels on one axis. will be read from the parameter-server. */
			double wheelPerimeter;	/**< [m] perimeter of one wheel. will be read from the parameter-server. */
			double maxMPS;	/**< [m/s] highest allowed speed, which could be send. will be read from the parameter-server. */

			double targetVelocity[2];	/**< [1] the velocity-value (with respect to the maximum allowed velocity) which should be reached. will be sent by velocity-messages. */

			struct timeval timeForKeys;	/**< time-value the terminal is waiting for a key-press. afterwards the velocity will be set. */
			struct termios oldKeyboardIO;	/**< the saved settings for the terminal. will be reset in the destructor */

			ros::Timer sineTimer;	/**< the ros::timer-object for calling the sine-function */
			ros::Timer keyboardTimer;	/**< a ros::timer-object, calls the function for getting keyboard-inputs */
			ros::Subscriber gamepad_sub;	/**< a ros::subscriber, which will call the gamepad-callback if a joy-messages will be received */

			ros::Publisher epos2Twist;	/**< ros::publisher for sending the velocity in a twist-messages */
			ros::Publisher epos2MPS;	/**< ros::publisher for sending the velocity in a speed-messages, that means only one velocity for both sides of a tankSterringRobot */
			ros::Timer epos2SetSpeedTimer;	/**< ros::timer which calls the function for sending the current velocity-values */
			tf::TransformBroadcaster tfBroad;	/**< a object for sending the current position to the tf-server */
			tf::Transform transform;	/**< Structure with all necessary informations for the tf-server */
		} sampleMovingSet;	/**< the main settings-structure for the SampleMoving-Class */

	public:
		/**
		 * Constructor of a SampleMoving-object. initialize all variables by callling init()
		 *
		 * @param sampleMovingRoshandle the ros::nodehandle on which all timers, publishers and subscribers will be set up
		 * @param getInfoFrom from which source the velocity-messages should be generated (humanInterface or sine)
		 * @param msgType the way a velocity-message should be published (SI, cmd_vel, ...)
		 * @param refreshRate [1/s] with this frequency all timers will be called
		 * @param SpeedStepSize [1] the highest allowed change of velocity in one step. with respective to the maximum velocity
		 * @param angle [degree] Angular offset between right and left EPOS in input-mode = sinus
		 */
		SampleMoving(ros::NodeHandle sampleMovingRoshandle, inputDevice getInfoFrom, msgTypes msgType = noMsg, double refreshRate = 5.0, double SpeedStepSize = 0.2, int angle = 0);

		/**
		 * Release the SampleMoving-object. Reset the keyboard-settings for the terminal
		 */
		~SampleMoving();

		/**
		 * return the input-number with the smallest absolute value
		 *
		 * @param arg1 first input-number
		 * @param arg2 second input-number
		 */
		double nextToZero(double arg1, double arg2);

		/**
		 * return 1 for positive values and -1 for negative values. if the input is 0 the return value will be 0 or 1 depending on the zeroIsPositiv-Parameter
		 *
		 * @param number the input which should be check to his sign
		 * @param zeroIsPositiv if it is set to 1 a input-zero will be become a output-one otherwise a output-zero
		 */
		int sign(double number, int zeroIsPositiv = 0);

	private:
		/**
		 * set all starting values - Including some values from the Parameter-Server and all ROS-Components (timer, publisher and subscriber.)
		 *
		 * @param msgType the way a velocity-message should be published (SI, cmd_vel, ...)
		 * @param getInfoFrom from which source the velocity-messages should be generated (humanInterface or sine)
		 */
		void init(msgTypes msgType, inputDevice getInfoFrom);
		/**
		 * publish with the defined refreshRate new velocity-messages.
		 * It's independent from the setting-functions - only the values in sampleMovingSet.targetVelocity will be put in the right message-type.
		 *
		 * @param event contains some time values according to the event
		 */
		void publishSpeed(const ros::TimerEvent& event);
		/**
		 * run every time, a new joy-message arrives from the connected gamepad or steering wheel. As long as the velocity on both sides of the robot
		 * is unequal to zero the input with the keyboard is blocked.
		 *
		 * @param joymsg include all axes and button informations from the joy-input (gamepad or steering wheel)
		 */
		void gamepadCallback (const sensor_msgs::Joy joymsg);
		/**
		 * publish with the defined refreshRate new velocity-messages. The velocity will change their values like a sine-function.
		 * An offset between left and right is possible with the value in sampleMovingSet.sineAngle .
		 *
		 * @param event contains some time values according to the event
		 */
		void sineCallback(const ros::TimerEvent& event);
		/**
		 * Set the terminal to catch the keys for driving. Also calculate the time-values (seconds and microsecond) from the refreshRate.
		 */
		void keyboardInit();
		/**
		 * try to catch a key from the input-terminal and set the velocity values according to this key (zero if no key is pressed).
		 * Only the keys W, S, A, D, Q and E will set a velocity unequal to zero. Only one key coudl't be recognized at once.
		 * The velocity will only be set when no input from a joy-input (gamepad or steering wheel) exist.
		 * If a velocity is set, smoothVelocity() will be called to prevent from (too) big velocity-steps.
		 *
		 * @param event contains some time values according to the event
		 */
		void keyboardCallback(const ros::TimerEvent& event);
		/**
		 * Prevent the velocity from higher steps than the speedStepSize according to the current velocity.
		 *
		 * @param inputVelocity the wanted velocity for the left and right side
		 * @param outputVelocityLeft the smoothed velocity for the left side
		 * @param outputVelocityRight the smoothed velocity for the right side
		 * @param speedStepSize the highest allowed difference for a new velocity according to the current velocity
		 * @param useDelta if zero the velocity of both side will not depending each other. Only for keyboard-input it should be set to one.
		 */
		void smoothVelocity(double inputVelocity[2], double *outputVelocityLeft, double *outputVelocityRight, double speedStepSize = 0.2, int useDelta = 0);

};

#endif /* SAMPLEMOVING_H_ */
