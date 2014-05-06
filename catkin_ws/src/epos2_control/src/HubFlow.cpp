
//Ansteuerung HUB, Flow
#include "HubFlow.h"


HubFlow::HubFlow(ros::NodeHandle roshandle, Epos2MotorController* epos[4], double maxMPS, double velocityDuration)
{
   hubflowSettings.maxMPS = maxMPS;
   hubflowSettings.velocityDuration = velocityDuration;

   hubflowSettings.roshandle = roshandle;
   
   for (int i=hub; i<=flow;i++) {
		hubflowSettings.epos[i] = epos[i];
   }

   hubflowSettings.maxSafetyMPS =hubflowSettings.maxMPS;

   init();
}

HubFlow::~HubFlow()
{

}

int HubFlow::init()
{
   int j = 0;
   int active[2] = {0,0};
   Epos2MotorController::EposState state = Epos2MotorController::disabled;

   for (int i = hub; i <= flow; i++) { hubflowSettings.epos[i]->testForErrorsAndPrint(); }
   resetEPOS2Fault();
   for (int i=hub; i<=flow;i++) {hubflowSettings.epos[i]->init(hubflowSettings.maxRPM);}
   ROS_INFO("initialize both EPOS2 successfully");

   j = 0;
   do {
	if (hubflowSettings.epos[hub]->getEpos2State() == Epos2MotorController::enabled && hubflowSettings.epos[flow]->getEpos2State() == Epos2MotorController::enabled) {
		state = Epos2MotorController::enabled;
		if (j == 0) 
		{ 
			for (int i=hub; i<=flow;i++) {hubflowSettings.epos[i]->enableEpos2();} }
		}
		j++;
   } while ( state != Epos2MotorController::enabled && j < 6);

   if (state == Epos2MotorController::enabled) {

      for (int i=hub; i<=flow; i++) {
	   active[i] = hubflowSettings.epos[i]->activateProfileVelocity();
      }

		if ( active[hub] == 1 && active[flow] == 1 ) {
			// Abhoeren von SteuerNachrichten
			hubflowSettings.epos2hubflow = hubflowSettings.roshandle.subscribe("epos2_MPS_hub_flow", 1000, &HubFlow::driveCallbackMPS, this);
			//hubflowSettings.epos2Twist = hubflowSettings.roshandle.subscribe("cmd_vel_hub_flow", 1000, &HubFlow::driveCallback, this);

			hubflowSettings.setVelocity = hubflowSettings.roshandle.createTimer(ros::Duration(hubflowSettings.velocityDuration), &HubFlow::setVelocityCallback, this);

			ROS_INFO("Epos2-HubFlow is now listing for velocity messages");
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


void HubFlow::resetEPOS2Fault()
{
	for (int i=hub; i<=flow;i++) {hubflowSettings.epos[i]->resetDevice();}
	ROS_INFO("reset EPOS2 successfully");
	for (int i=hub; i<=flow;i++) {hubflowSettings.epos[i]->clearFault();}
	ROS_INFO("clear EPOS2 fault successfully");
}


void HubFlow::driveCallbackMPS(const epos2_control::velocity speed)
{
	hubflowSettings.targetVelocityMPS[left] = speed.left;
	hubflowSettings.targetVelocityMPS[right] = speed.right;
}



void HubFlow::setVelocityCallback(const ros::TimerEvent& event)
{
	double targetVelocityRPM[2];

	for (int i=hub; i<=flow; i++) {targetVelocityRPM[i] = (hubflowSettings.targetVelocityMPS[i];}

	for (int i=hub; i<=flow; i++) {hubflowSettings.epos[i]->changeRotationPerMinute(targetVelocityRPM[i]);}
}

