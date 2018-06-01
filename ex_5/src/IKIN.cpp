#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <sstream>

//współrzędne końcówki
double x=0;
double y=0;
double z=0;

//odebranie wiadomości ze współrzędnymi końcówki
void callbackEndState(const geometry_msgs::PoseStamped::ConstPtr& state)
{
	x = state->pose.position.x;
	y = state->pose.position.y;
	z = state->pose.position.z;

	std::cout<< "I heard x: " << x <<" y: " << y << " z: "<< z <<std::endl;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "IKIN");
	ros::NodeHandle n;

	ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
	ros::Subscriber joint_state = n.subscribe("pose_stamped", 1, callbackEndState);

	ros::Rate loop_rate(50);
	
	//rozwiązywanie OZK i wysłanie do RobotStatePublisher
	while(ros::ok())
	{
		sensor_msgs::JointState msg;
		msg.header.stamp = ros::Time::now();
		
		
		
		/**
			Rozwiazywanie zadania kinematyki odwrotnej
		*/


		msg.name.push_back("rotation_joint"); //msg.position.push_back();
		msg.name.push_back("shoulder");
		msg.name.push_back("elbow");

		JointStatePub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();

	}

	ros::spin();

	return 0;
}
