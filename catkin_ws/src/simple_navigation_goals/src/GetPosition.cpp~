#include "ros/ros.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "GetPosition");


  ros::NodeHandle n;

  tf::TransformListener listener;

  ros::Rate rate(1.0);

  while (n.ok())
  {
    tf::StampedTransform transform;
    try
    {
        //ROS_INFO("Attempting to read pose...");
        listener.lookupTransform("/map","/base_link",ros::Time(0), transform);
	
	double x = transform.getOrigin().x();
	double y = transform.getOrigin().y();

        ROS_INFO("x = ", x);
	ROS_INFO("y = ", y);
        
	double yaw = tf::getYaw(transform.getRotation());

	ROS_INFO("yaw =", yaw);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Nope! %s", ex.what());
    } 


    rate.sleep();

  }
  return 0;
}
