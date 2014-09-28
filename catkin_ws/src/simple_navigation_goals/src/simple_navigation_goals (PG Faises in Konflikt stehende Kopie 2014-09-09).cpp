#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//void GoalCallback(const std_msgs::String::ConstPtr& msg){
  //ROS_ERROR("Message: %s", msg->data.c_str());
//we'll send a goal to the robot to move 1 meter forward

  //if (msg->data.c_str() == "xxx")
	//goal.target_pose.header.frame_id = "map";
  	//goal.target_pose.header.stamp = ros::Time::now();

  	//goal.target_pose.pose.position.x = 0.543;
  	//goal.target_pose.pose.position.y = 1.176;
  	//goal.target_pose.pose.orientation.w = -0.741;

  	//ROS_INFO("Sending goal");
  	//ac.sendGoal(goal);

  	//ac.waitForResult();

  	//if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	//ROS_INFO("Hooray, the base moved 1 meter forward");
  	//else
    	//ROS_INFO("The base failed to move forward 1 meter for some reason");
//}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");


  //ros::NodeHandle n;
  //ros::Subscribe sub = n.subscribe("MessageTopic", 1000, GoalCallback);


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.543;
  goal.target_pose.pose.position.y = 1.176;
  goal.target_pose.pose.orientation.w = -0.741;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}
