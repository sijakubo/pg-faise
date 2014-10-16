//@author Jannik Flessner

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher pub;
ros::Publisher micazPub;
static int targetFlag = 0;
static std::string messageFlowLoad;
static int goalArrived;


void GoalCallback(const std_msgs::String::ConstPtr& msg){

  ROS_ERROR("MessageCallback started");
  std::string messageGoal = msg->data.c_str();
  
  
  uint8_t micaz[5];

  std::string rampeAusgang1 = "2 1 10 0 0"; // Rampe wand unten
  std::string rampeAusgang2 = "2 1 20 0 0"; // Rampe vorne unten
  std::string rampeEingang1 = "1 1 10 0 0"; // Rampe wand oben
  std::string rampeEingang2 = "1 1 20 0 0"; // Rampe vorne oben
  std::string cost1 = "10 1 10 1 20";
  std::string cost2 = "10 1 20 1 10";
  std::string out;  

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_ERROR("Action Server ready");
  if (messageGoal == rampeAusgang1){
	ROS_ERROR("Rampe Ausgang Wand unten");        

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = 3.135;
  	goal.target_pose.pose.position.y = -0.850;
  	goal.target_pose.pose.position.z = 0.0;
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.999;
  	goal.target_pose.pose.orientation.w = -0.043;

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
  	ac.waitForResult();

        messageFlowLoad = "down110";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending down");

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Goal reached");
		goalArrived = 1;
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}
	
  }
  else if (messageGoal == rampeAusgang2){
	ROS_ERROR("Rampe Ausgang vorne unten");        

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = -0.564;
  	goal.target_pose.pose.position.y = 1.043;
  	goal.target_pose.pose.position.z = 0.0;
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.044;
  	goal.target_pose.pose.orientation.w = 0.999;

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
  	ac.waitForResult();

        messageFlowLoad = "down120";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending down");

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Goal reached");
		goalArrived = 1;
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}  

  }
  else if (messageGoal == cost1){
	micaz[0] = 0x11;
	micaz[1] = 0x01;
	micaz[2] = 0x10;
	micaz[3] = 0x0e;
	micaz[4] = 0x0f;
	char buf[100];
  	sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  	out = buf;
	micazPub.publish(out);
  }
  else if (messageGoal == cost2){
	micaz[0] = 0x11;
	micaz[1] = 0x01;
	micaz[2] = 0x20;
	micaz[3] = 0x0e;
	micaz[4] = 0x0f;
	char buf[100];
  	sprintf(buf, "%c%c%c%c%c", micaz[0], micaz[1], micaz[2], micaz[3], micaz[4]);
  	out = buf;
	micazPub.publish(out);
  }
  else if (messageGoal == rampeEingang1){
	targetFlag = 0x0110;
	goalArrived = 0;
	ROS_ERROR("Rampe Eingang Wand");
  }
  else if (messageGoal == rampeEingang2){
	targetFlag = 0x0120;
	goalArrived = 0;
	ROS_ERROR("Rampe Eingang vorne");         
  }

  if (goalArrived == 1 && messageFlowLoad != "load"){
	messageFlowLoad = "load";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending load");
        goalArrived = 0;
  }
  else if (goalArrived == 2 && messageFlowLoad != "unload"){
	messageFlowLoad = "unload";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending unload");
        goalArrived = 0;
  }
  else if (goalArrived == 0){
	ROS_ERROR("Load Status not send");
  }
}


void LoadCallback(const std_msgs::String::ConstPtr& msg){

  ROS_ERROR("Navigate to second Goal");
  std::string messageLoad = msg->data.c_str();
  std::string packageLoaded = "PackageLoaded";
  uint8_t micaz[5];
  std::string out;

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_ERROR("Action Server ready");

  if (messageLoad == packageLoaded && targetFlag == 0x0110){

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = -0.334;
  	goal.target_pose.pose.position.y = -1.204;
  	goal.target_pose.pose.position.z = 0.0;
  
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.043;
  	goal.target_pose.pose.orientation.w = 0.999;


  	ROS_INFO("Rampe anfahren");
  	ac.sendGoal(goal);

  	ac.waitForResult();

	messageFlowLoad = "up110";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sende UP");

  
  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Goal reached");
		goalArrived = 2;
		targetFlag = 0x0000;
			
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}
  }
  else if (messageLoad == packageLoaded && targetFlag == 0x0120){
	
    	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = 2.924;
  	goal.target_pose.pose.position.y = 1.378;
  	goal.target_pose.pose.position.z = 0.0;
  
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.998;
  	goal.target_pose.pose.orientation.w = -0.068;

  	ROS_INFO("Rampe anfahren");
  	ac.sendGoal(goal);

  	ac.waitForResult();

	messageFlowLoad = "up120";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sende UP");

  
  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Goal reached");
		goalArrived = 2;
                targetFlag = 0x0000;
			
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}		
	
  }
  if (goalArrived == 1 && messageFlowLoad != "load"){
	messageFlowLoad = "load";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending load");
        goalArrived = 0;
  }
  else if (goalArrived == 2 && messageFlowLoad != "unload"){
	messageFlowLoad = "unload";
	pub.publish(messageFlowLoad);
	ROS_ERROR("Sending unload");
        goalArrived = 0;
  }
  else if (goalArrived == 0){
	ROS_ERROR("Load Status not send");
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle n;

  pub = n.advertise<std_msgs::String>("flow", 10);
  micazPub = n.advertise<std_msgs::String>("uc1Command", 1000);
  ros::Subscriber sub = n.subscribe("uc1Response", 1000, GoalCallback);
  ros::Subscriber flowSub = n.subscribe("flow", 10, LoadCallback);

  ros::spin();
  return 0;
}
