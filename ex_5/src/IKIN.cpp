#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include <cmath>

#define PI 3.14159265

//nowe współrzędne końcówki
double x = 1;
double y = 0;
double z = -0.4;

/*//stare współrzędne końcówki
double x_old = 1;
double y_old = 0;
double z_old = 0.4;*/

//długości członów
double a1;
double a2;

/*//długości podstaw
double base = 0.2;
double rot_base = 0.1;*/
double a0 = 0.3;
//poszczególne kąty w stawach
/*double rotate;
double flex1=-0.25;
double flex2=-0.25;*/
//double flex_end;

//--------------------------------------------
double teta1, teta2, teta3;
double p, c, a, h, alfa;
//--------------------------------------------


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

	teta1 = 0;
	teta2 = -1;
	teta3 = 1;
	
	ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
	ros::Subscriber PoseStateSub = n.subscribe("pose_stamped", 1, callbackEndState);

	ros::Rate loop_rate(50);
	
	//pobranie z serwera parametrów
	bool ok0 = n.getParamCached("a2_length", a1);
	bool ok1 = n.getParamCached("a3_length", a2);

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
		
		*/
		//wyliczenie kąta dla bazy obrotowej <-----------------------------------NIE DZIAŁA XD
/*		double a = sqrt( pow(x,2) + pow(y,2) );
		double b = sqrt( pow(a,2) + pow(z-a0,2) );
		double c = sqrt( pow(abs(x-x_old),2) + pow(abs(y-y_old),2) );
*/
		z = z - a0;
		p = sqrt(x*x + y*y);
		c = sqrt(p + z*z);
		a = a1*c/(a1 + a2);
		h = sqrt(a1*a1 - a*a);
		alfa = atan(h/a);
		
		if(x>0)
			teta1 = atan(y/x);
		else
			teta1 = atan(y/x) - PI;
		
		teta2 = (alfa + atan(z/p)) - PI/2;
		teta3 = alfa + atan(h/(c-a));
		
/*		teta2 = PI/2 - (acos(a2/(c-a)) + atan(z/p));
		teta3 = acos(a2/(c-a)) + acos(a/a1);
*/		
/*		rotate = atan(y/x);
		
		flex1 = atan( a/(z - a0) );
		flex2 = acos( ( pow(link_1,2) +pow(b,2) - pow(a2,2) )/(2*a1*b) );;
*/		
		
		
//		std::cout<<"arg atan: "<<(y/x)<<std::endl;
//		std::cout<<"arg acos: "<<(y/x)<<std::endl;

/*		std::cout<<"kat baza: "<<rotate<<" "<<std::endl;
		std::cout<<"kat flex1: "<<flex1<<" "<<std::endl;
*/
		msg.name.push_back("rotation_joint"); //msg.position.push_back();
		msg.name.push_back("shoulder");
		msg.name.push_back("elbow");

/*		msg.position.push_back(rotate);
		msg.position.push_back(flex1);
		msg.position.push_back(0.3);
*/
		msg.position.push_back(teta1);
		msg.position.push_back(teta2);
		msg.position.push_back(teta3);
		
		JointStatePub.publish(msg);
		
		ros::spinOnce();

		loop_rate.sleep();

	}

	ros::spin();

	return 0;
}
