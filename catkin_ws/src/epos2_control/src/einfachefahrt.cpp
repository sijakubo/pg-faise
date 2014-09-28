#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>



int main(int argc, char **argv) {
	
	ros::Publisher epos2Twist;

    double speedMPS[2];
	geometry_msgs::Twist speedVector;

	speedVector.linear.x = 0;
	speedVector.linear.y= 0.5;
	speedVector.linear.z= 0;
	speedVector.angular.x=0;
	speedVector.angular.y=0;
	speedVector.angular.z=0;
    epos2Twist.publish(speedVector);
			
	
}