#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <cmath>

//nowe współrzędne końcówki
double x = 1;
double y = 0;
double z = 0.4;

//stare współrzędne końcówki
double x_old = 1;
double y_old = 0;
double z_old = 0.4;

//długości członów
double link_1;
double link_2;

//długości podstaw
double base = 0.2;
double rot_base = 0.1;
double a0 = 0.3;
//poszczególne kąty w stawach
double rotate;
double flex1;
double flex2;
double flex_end;

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
	ros::Subscriber PoseStateSub = n.subscribe("pose_stamped", 1, callbackEndState);

	ros::Rate loop_rate(50);
	
	//pobranie z serwera parametrów
	bool ok0 = n.getParamCached("a2_length", link_1);
	bool ok1 = n.getParamCached("a3_length", link_2);

	if ( !ok0 || !ok1 )
	{
			puts("ERROR OCCURED"); 
			exit(1);
	}
	
	//rozwiązywanie OZK i wysłanie do RobotStatePublisher
	while(ros::ok())
	{

		sensor_msgs::JointState msg;
		msg.header.stamp = ros::Time::now();
		
		
		
		/**
			Rozwiazywanie zadania kinematyki odwrotnej
		

		//wyliczenie ąta dla bazy obrotowej <-----------------------------------NIE DZIAŁA XD
		double a = sqrt( pow(x,2) + pow(y,2) );
		double b = sqrt( pow(x_old,2) + pow(y_old,2) );
		double c = sqrt( pow(abs(x-x_old),2) + pow(abs(y-y_old),2) );

		rotate = acos( ( (pow(a,2) + pow(b,2) - pow(c,2) ) / ( 2*a*b ) ) );
		*/
		rotate = atan(y/x);
		//if(rotate>1 && rotate<-1) std::cout<<"zle katy bazy"<<std::endl;

		std::cout<<"arg atan: "<<(y/x)<<" "<<std::endl;
		//KONIEC###########################################


		std::cout<<"kat baza: "<<rotate<<" "<<std::endl;
		msg.name.push_back("rotation_joint"); //msg.position.push_back();
		msg.name.push_back("shoulder");
		msg.name.push_back("elbow");

		msg.position.push_back(rotate);
		msg.position.push_back(-0.5);
		msg.position.push_back(0.3);

		JointStatePub.publish(msg);
		
		
		
		ros::spinOnce();

		loop_rate.sleep();

	}

	ros::spin();

	return 0;
}
