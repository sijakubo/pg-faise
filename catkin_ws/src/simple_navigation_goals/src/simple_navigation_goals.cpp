//@author Jannik Flessner

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher pub;


void GoalCallback(const std_msgs::String::ConstPtr& msg){

  ROS_ERROR("MessageCallback started");
  std::string messageGoal = msg->data.c_str();
  std::string messageFlowLoad;
  int goalArrived;
  //ROS_ERROR("Message: %x", messageGoal);
  std::string rampeAusgang = "61 1 1 0 0";
  

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_ERROR("Action Server ready");
  if (messageGoal == rampeAusgang){
	ROS_ERROR("First Goal Received");        

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = 0.220;
  	goal.target_pose.pose.position.y = 0.119;
  	goal.target_pose.pose.position.z = 0.0;
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.767;
  	goal.target_pose.pose.orientation.w = 0.642;

	messageFlowLoad = "down";
	pub.publish(messageFlowUnload);

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);
  	ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Goal reached");
		goalArrived = 1;
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}
	
   }  

  if (goalArrived == 1){
	ROS_INFO("Sending Load Message");
	messageFlowLoad = "load";
	pub.publish(messageFlowLoad);
  }


}

void LoadCallback(const std_msgs::String::ConstPtr& msg){

  std::string messageLoad = msg->data.c_str();
  std::string packageLoaded = "PackageLoaded";
  std::string messageFlowUnload;
  int goalArrived;

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  if (messageLoad == packageLoaded){
	ROS_ERROR("Second Goal Received");        
	
	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
  	goal.target_pose.header.stamp = ros::Time::now();

  	goal.target_pose.pose.position.x = 1.404;
  	goal.target_pose.pose.position.y = 0.662;
  	goal.target_pose.pose.position.z = 0.0;
  
  	goal.target_pose.pose.orientation.x = 0.0;
  	goal.target_pose.pose.orientation.y = 0.0;
  	goal.target_pose.pose.orientation.z = 0.780;
  	goal.target_pose.pose.orientation.w = 0.625;

	messageFlowUnload = "up";
	pub.publish(messageFlowUnload);

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);

  	ac.waitForResult();

  
  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Goal reached");
		goalArrived = 2;		
	}
  	else {
    		ROS_INFO("Goal not reached");
		goalArrived = 0;
	}
  }

  if (goalArrived == 2){
	messageFlowUnload = "unload";
	pub.publish(messageFlowUnload);
  }


}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");


  ros::NodeHandle n;

  pub = n.advertise<std_msgs::String>("flow", 1000);
  ros::Subscriber sub = n.subscribe("uc1Response", 1000, GoalCallback);
  ros::Subscriber flowSub = n.subscribe("flow", 1000, LoadCallback);


  ros::spin();
  return 0;
}
