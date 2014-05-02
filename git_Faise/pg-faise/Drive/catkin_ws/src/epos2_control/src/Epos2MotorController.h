/*
 * Epos2MotorController.h
 *
 *  Created on: Apr 11, 2012
 *      Author: Martin Seidel
 */

/**
 * EPOS2-MotorController-Class.
 * It's used to control one EPOS2.
 */

#ifndef EPOS2MOTORCONTROLLER_H_
#define EPOS2MOTORCONTROLLER_H_

#include <iostream>			//text in-&out-put; std::string declaration
#include "ros/ros.h"		//communication with ros
#include "Definitions.h"	//declaration of functions to control the epos2
#include <math.h>			//it gives abs()
#include "std_msgs/String.h" //text output within ros

class Epos2MotorController {

		/**
		 * Settings Structure with all relevant values.
		 **/
		struct epos2Settings {
			int node;	/**< internal NodeNumber of the EPOS2 */
			int master;	/**< indicates if this EPOS2 has build up the communication or just get a handle */
			int reverse;	/**< defines the positive direction of the rotation */
			double maxRotationPerMinute;	/**< [rev/min] highest allowed angular velocity */
			double wheelPerimeter;	/**< [m] the Perimeter of one wheel */
			double gearRatio;	/**< [-] the Ratio of the Gear between Motor and wheel */
			long int gearNumerator;	/**< [-] the numerator from the Ratio of the gear */
			int gearDenomiantor;	/**< [-] the denominator from the ratio of the gear */
		}epos2MotorSet;	/**< Settings for the EPOS2 */

		/**
		 * all possible internal Errorlevel for a published message
		 **/
		enum {debug, info, warn, error, fatal};

	public:

		void* devhandle;	/**< the device handle for the communication with the EPOS2 */
		char* nodeStr;	/**< this appears in front of a message published with ROS. If not set it's "EPOS2". */

		/**
		 * possible Types of Data in the Parameter structures. Equation in c++ data Types
		 */
		enum Type
		{
			uShortInt,	/**< unsigned short int */
			uInt,	/**< unsigned int */
			uLongInt,	/**< unsigned long int */
			doubleT	/**< double */
		};

		/**
		 * internal state of the MotorController-Object with respect to the state of the EPOS2
		 */
		enum EposState
		{
			enabled,	/**< EPOS2 is ready for driving */
			disabled,	/**< EPOS2 is started but not ready for driving. Changing of the most Settings in the EPOS2 only possible in this mode. */
			quickstop,	/**< Safety brake. EPOS2 is started and stop the motor. */
			reset,	/**< before deleting Faults you had to reset all EPOS2. no corresponding state in EPOS2. */
			fault,	/**< no movement is possible. A text message with detail infos will be generated @see testForErrorsAndPrint() */
			unknown	/**< couldn't read state. Maybe no connection to the EPOS2. */
		};

		/**
		 * Gain values for a PI regulator
		 */
		struct regulatorPI {
			short unsigned int p;	/**< P Gain */
			static const Type pType = uShortInt;	/**< Type of Data from P Gain */
			short unsigned int i;	/**< I Gain */
			static const Type iType = uShortInt;	/**< Type of Data from I Gain */
		};

		/**
		 * Gain value for a PID regulator
		 */
		struct regulatorPID {
			short unsigned int p;	/**< P Gain */
			static const Type pType = uShortInt;	/**< Type of Data from P Gain */
			short unsigned int i;	/**< I Gain */
			static const Type iType = uShortInt;	/**< Type of Data from I Gain */
			short unsigned int d;	/**< D Gain */
			static const Type dType = uShortInt;	/**< Type of Data from D Gain */
		};

		/**
		 * Feedforward values for a regulator
		 */
		struct feedForward {
			short unsigned int Velocity;	/**< velocity value */
			static const Type VelocityType = uShortInt;	/**< Type of Data from velocity value */
			short unsigned int Acceleration;	/**< acceleration value */
			static const Type AccelerationType = uShortInt;	/**< Type of Data from acceleration value */
		};

		/**
		 * Structure of all regulator values used by the EPOS2
		 */
		struct regulator {
			regulatorPID positionGain;	/**< PID Gain values for the position regulator */
			feedForward positionFF;	/**< feedforward values for the position regulator */
			regulatorPI velocityGain;	/**< PI Gain values for the (angular) velocity regulator */
			feedForward velocityFF;	/**< feedforward values for the (angular) velocity regulator */
			regulatorPI currentGain;	/**< PI Gain values for the current regulator */
		};

		/**
		 * all relevant Parameter to setup the motor
		 */
		struct motorData {
			unsigned short continuousCurrent;	/**< [mA] the current which is allowed without a pause */
			static const Type continousCurrentType = uShortInt;	/**< Type of Data from the continuous current value */
			unsigned short maxPeakCurrent;	/**< [mA] maximum allowed current for a short Time. Normally it's twice the continuous current value */
			static const Type maxPeakCurrentType = uShortInt;	/**< Type of Data from the maximum current value */
			unsigned short thermalTimeConstantWinding;	/**< [e-1 s] this time constant has influence on how long the maximum current is allowed. It's not directly the Time the maximum current is allowed. */
			static const Type thermalTimeConstantWindingType = uShortInt;	/**< Type of Data from thermalTimeConstantWinding */
			unsigned long maxRPM;	/**< [rev/min] highest allowed angular velocity of the motor*/
			static const Type maxRPMType = uLongInt;	/**< Type of Data from the maxRPM */
		};

		/**
		 * all values of the gear between motor and wheel. Until now this values are not saved in the EPOS2
		 */
		struct gearData {
			unsigned long maxRPM;	/**< [rev/min] highest allowed angular velocity of the gear */
			static const Type maxRPMType = uLongInt;	/**< Type of Data from maxRPM */
			unsigned long numerator;	/**< [-] the numerator of gear ratio */
			static const Type numeratorType = uLongInt;	/**< Type of Data from the numerator */
			unsigned int denominator;	/**< [-] the denominator of the gear ratio */
			static const Type denominatorType = uInt;	/**< Type of Data from the denominator */
		};

		/**
		 * all relevant acceleration values for the motor
		 */
		struct accelerationData {//all in [m/(s^2)] units
			double profileAcceleration;	/**< [m/(s^2)] acceleration value for the velocity ramp in the Profile Velocity Mode. had to be positive and nonzero */
			static const Type profileAccelerationType = doubleT;	/**< Type of Data from profileAcceleration */
			double profileDeceleration;	/**< [m/(s^2)] deceleration value for the velocity ramp in the Profile Velocity Mode. had to be positivie and nonzero */
			static const Type profileDecelerationType = doubleT;	/**< Type of Data from profileDeceleration */
			double quickstopDeceleration;	/**< [m/(s^2)] deceleration value for the quickstopp command. had to be positive and nonzero */
			static const Type quickstopDecelerationType = doubleT;	/**< Type of Data from quickstoppDeceleration */
			double maxAcceleration;	/**< [m/(s^2)] maximum allowed acceleration and deceleration value. had to be positive and nonzero */
			static const Type maxAccelerationType = doubleT;	/**< Type of Data from maxAcceleration */
		};

		/**
		 * Constructor of a MotorController Object to build up a new communication.
		 *
		 * Only use once for a bunch of a EPOS2 connected among themselves by CAN.
		 *
		 * @param node the internal node number of the EPOS2 you want to connect to
		 * @param reverseNode if it's 1 the positive direction of movement will be switched @see setSpin()
		 * @param port the name of the port where the EPOS2 is connected (USB0, USB1, ... or /dev/ttyS0, /dev/ttyS1, ...)
		 * @param timeout [ms] If there is no response from the EPOS2 within this time, you will get a communication error. If it's zero or not set, a value of 750 will be used.
		 * @param baudrate [bit/s] the frequency for the communication. If it's not set or zero, it will be set to 1000000 (USB) or 115200 (RS232 - Serial)
		 */
		Epos2MotorController(int node, int reverseNode, std::string port, unsigned long timeout = 0, unsigned long baudrate = 0);
		/**
		 * Constructor of a MotorController Object with a given communication handler.
		 *
		 * @param node the internal node number of the EPOS2 you want to connect to
		 * @param reverseNode if it's 1 the positive direction of movement will be switched @see setSpin()
		 * @param newDevhandle a device handle from another EPOS2 connected on the same wire (USB or RS232; among themselves the EPOS2 should be connected over CAN)
		 */
		Epos2MotorController(int node, int reverseNode, void* newDevhandle);
		/**
		 * Release the EPOS2.
		 *
		 * All EPOS2 will be set into disable state. Only the EPOS2 which build up the communication ("master") release also the device.
		 * Maybe it's not a good idea to release the master before the other EPOS2(s) connected over CAN to this master.
		 */
		~Epos2MotorController();

		/**
		 * Delete all errors in the Error history of this EPOS2.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int clearFault();
		/**
		 * Reset the State Machine inside the EPOS2.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int resetDevice();
		/**
		 * calls a few set-functions, store the settings in the EPOS2 and enable the Device.
		 *
		 * @param maxRPM [rev/min] highest allowed angular velocity for this EPOS2
		 * @param wheelPerimeter [m] Perimeter of one wheel connected to this EPOS2
		 *
		 * @see setSpin()
		 * @see setDimensionAndNotation()
		 * @see setMotorData()
		 * @see setGearData()
		 * @see setAccelerationData()
		 * @see setMaxRPM()
		 * @see storeSettings()
		 */
		void init(double maxRPM, double wheelPerimeter);

		/**
		 * read out the error bit and the error history. If an error was detected a message will be send to rosOUT.
		 *
		 * @return 0 if a DeviceError was detect, else return 1
		 */
		int testForErrorsAndPrint();

		/**
		 * set the EPOS2 into enable state
		 *
		 * @return 1 if successful otherwise 0
		 */
		int enableEpos2();
		/**
		 * set the EPOS2 into disable state
		 *
		 * @return 1 if successful otherwise 0
		 */
		int disableEpos2();
		/**
		 * stores all changed Settings in the EPOS2.
		 * this should be done after changing settings and before enabling the EPOS2.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int storeSettings();
		/**
		 * change the OperationMode of the EPOS2 into ProfileVelocityMode and set the MotionProfile to sin^2-ramp
		 *
		 * @return 1 if successful otherwise 0
		 */
		int activateProfileVelocity();
		/**
		 * set a target angular velocity for this EPOS2.
		 *
		 * @param targetVelocityRPM [rev/min] angular velocity target of the wheel (not on motor side of the gear)
		 * @return 1 if successful otherwise 0
		 */
		int changeRotationPerMinute(double targetVelocityRPM);

		/**
		 * read out the state of the EPOS2 state machine
		 *
		 * @return internal state in the Epos2MotorControllerClass
		 */
		EposState getEpos2State();
		/**
		 * read out the current position value depending on the start position.
		 * convert the encoder ticks into [rev/min] of the wheel (after gear)
		 *
		 * @return [rev/min] Rotations count from start position
		 */
		double getAbsolutePosition();
		/**
		 * set all Data of the gear into the EPOS2.
		 * changes are only allowed in disabled state.
		 *
		 * @param *parameter structure with all relevant gear data
		 * @return 1 if successful otherwise 0
		 */
		int setGearData(gearData *parameter);
		/**
		 * read out all Data of the gear from the EPOS2
		 *
		 * @param *parameter structure for all relevant gear data
		 * @return 1 if successful otherwise 0
		 */
		int getGearData(gearData *parameter);
		/**
		 * set all Data of the motor into the EPOS2.
		 * changes are only allowed in disabled state.
		 * maxPeakCurrent will be the double value of continousCurrent, if continousCurrent is higher than maxPeakCurrent
		 *
		 * @param *parameter structure with all relevant motor data
		 * @return 1 if successful otherwise 0
		 */
		int setMotorData(motorData *parameter);
		/**
		 * read out all Data of the motor from the EPOS2
		 *
		 * @param *parameter structure for all relevant motor data
		 * @return 1 if successful otherwise 0
		 */
		int getMotorData(motorData *parameter);
		/**
		 * set all Data restricting the acceleration into the EPOS2.
		 * changes are only allowed in disabled state.
		 *
		 * @param *parameter structure with all relevant acceleration data
		 * @return 1 if successful otherwise 0
		 */
		int setAccelerationData(accelerationData *parameter);
		/**
		 * read out all Data restricting the acceleration from the EPOS2
		 *
		 * @param *parameter structure for all relevant acceleration data
		 * @return 1 if successful otherwise 0
		 */
		int getAccelerationData(accelerationData *parameter);
		/**
		 * set the highest allowed angular velocity into the EPOS2.
		 * changes are only allowed in disabled state.
		 *
		 * @param maxRPM highest allowed angular velocity
		 * @return 1 if successful otherwise 0
		 */
		int setMaxRPM(double maxRPM);
		/**
		 * read out the currently set highest allowed angular velocity from the EPOS2.
		 *
		 * @param *maxRPM current highest allowed angular velocity
		 * @return 1 if successful otherwise 0
		 */
		int getMaxRPM(double *maxRPM);

		/**
		 * read out all parameter of the regulators from the EPOS2
		 *
		 * @param *parameter structure for all regulator gain and feedforward values
		 * @return 1 if successful otherwise 0
		 */
		int getRegulatorParameter(regulator *parameter);
		/**
		 * set all parameter of the regulators into the EPOS2.
		 * changes are only allowed in disabled state.
		 *
		 * @param *parameter structure with all regulator gain and feedforward values
		 * @return 1 if successful otherwise 0
		 */
		int setRegulatorParameter(regulator *parameter);

	private:

		EposState state; /**< the state inside this class depending on the state of the EPOS2 @see getEpos2State() */

		/**
		 * set the positive direction of movement into the EPOS2.
		 * changes are only allowed in disabled state.
		 *
		 * @param reverse if it's 1 the positive direction is CW otherwise CCW
		 * @return 1 if successful otherwise 0
		 */
		int setSpin(int reverse);
		/**
		 * set how the numbers sending to the EPOS2 will be interpreted.
		 * it's set for position/way, velocity and acceleration.
		 * The velocity Notation index is set to e-3 because the velocity will be set as integers it increases the accuracy.
		 * changes are only allowed in disabled state.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int setDimensionAndNotation();

		/**
		 * puts out a text message into ros standard output.
		 * the pattern is like this: [\%nodeStr\%] \%errorText\%\%errorStr\%
		 * while \%errorText\% will be replaced by the given function parameter, \%nodeStr\% is a global member of this class with the name of the Node
		 * and \%errorStr\% is a text message generated by the EPOS library when a errorBit unequal 1 or an errorCode unequal 0 is given
		 *
		 * @see nodeStr
		 *
		 * @param errorText the message which should be printed
		 * @param errorLevel the ROS ErrorLevel on which the message should be published
		 * @param errorBit just an integer. most functions of the EPOS library give back an integer value.
		 * @param errorCode an errorCode returned by a function of the EPOS library.
		 */
		void errorOutput(std::string errorText, int errorLevel = info, long int errorBit = 1, unsigned int errorCode = 0);

		/**
		 * build up the communication to a plugged EPOS2.
		 *
		 * @param port the name of the port you want to connect to (USB0, USB1, ... or /dev/ttyS0, /dev/ttyS1, ...)
		 * @param baudrate [bit/s] the frequency for the communication. If it's not set or zero, it will be set to 1000000 (USB) or 115200 (RS232 - Serial)
		 * @param timeout [ms] If there is no response from the EPOS2 within this time, you will get a communication error. If it's zero or not set, a value of 750 will be used.
		 *
		 * @return 1 if successful otherwise 0
		 */
		int initCommunication(std::string port, unsigned long baudrate, unsigned long timeout);
		/**
		 * checks if a given number is positive and nonzero.
		 * if not an error will be send to ros output
		 *
		 * @param number the number which should be checked
		 * @param varName this name will be published in the Error message if the number is negative or zero
		 *
		 * @return 1 if  not negative or zero otherwise 0
		 */
		int errorIfNotBiggerZero(double number, std::string varName);

};

#endif /*EPOS2MOTORCONTROLLER_H_*/

