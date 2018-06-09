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
double a2;
double a3;

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
double p, c, a, h, alfa, hyp;
//--------------------------------------------
ros::Publisher JointStatePub1;

//odebranie wiadomości ze współrzędnymi końcówki
void callbackEndState(const geometry_msgs::PoseStamped::ConstPtr& state)
{

	x = state->pose.position.x;
	y = state->pose.position.y;
	z = state->pose.position.z;

	if(sqrt(pow(z-a0,2)+x*x+y*y)>(a2+a3) || z<(a0-a3))
	{
		ROS_ERROR("Calculations not possible, selected point too far");
		return;

	} 

	sensor_msgs::JointState msg;
	msg.header.stamp = ros::Time::now();

	z = z - a0;
	teta1 = atan2(y,x);


	hyp = pow((x/cos(teta1)),2) + pow(z,2);
	teta2 = -atan2(z*cos(teta1),x)-acos((pow(a2,2)-pow(a3,2)+hyp)/(2*a2*pow(hyp,0.5)));
	if(x<0)
		teta2=teta2-PI;
	teta3 = acos((-pow(a2,2)-pow(a3,2) + hyp)/(2*a2*a3));
		
	std::cout<<"teta1: "<<teta1<<" a1: "<<a2<<" a: "<<a<<std::endl;
	std::cout<<"teta2: "<<teta2<<" c: "<<c<<std::endl;
	std::cout<<"teta3: "<<teta3<<std::endl;

	msg.name.push_back("rotation_joint"); //msg.position.push_back();
	msg.name.push_back("shoulder");
	msg.name.push_back("elbow");

	msg.position.push_back(teta1);
	msg.position.push_back(teta2);
	msg.position.push_back(teta3);

	JointStatePub1.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "IKIN");
	ros::NodeHandle n;

	teta1 = 0;
	teta2 = -1;
	teta3 = 1;
	
	
	//pobranie z serwera parametrów
	bool ok0 = n.getParamCached("a2_length", a2);
	bool ok1 = n.getParamCached("a3_length", a3);

	if ( !ok0 || !ok1 )
	{
			puts("ERROR OCCURED"); 
			exit(1);
	}
	
	ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
	JointStatePub1=JointStatePub;
	ros::Subscriber PoseStateSub = n.subscribe("pose_stamped", 1, callbackEndState);
	//rozwiązywanie OZK i wysłanie do RobotStatePublisher
/*
	while(ros::ok())
	{
		sensor_msgs::JointState msg;
		msg.header.stamp = ros::Time::now();
		
		

		z = z - a0;
		p = sqrt(x*x + y*y);
		c = sqrt(p*p + z*z);
		a = a1*c/(a1 + a2);
		h = sqrt(abs(a1*a1 - a*a));
		alfa = atan2(h,a);
	
		
		teta1 = atan2(y,x);
		
		teta2 = (alfa + atan2(z,p)) - PI/2;
		teta3 = alfa + atan2(h,(c-a));
		
		
		
		
		std::cout<<"teta1: "<<teta1<<" "<<a1<<" "<<a<<std::endl;
		std::cout<<"teta2: "<<teta2<<std::endl;

		msg.name.push_back("rotation_joint"); //msg.position.push_back();
		msg.name.push_back("shoulder");
		msg.name.push_back("elbow");


		msg.position.push_back(teta1);
		msg.position.push_back(teta2);
		msg.position.push_back(teta3);
		
		JointStatePub.publish(msg);
		
		ros::spinOnce();

		loop_rate.sleep();

	}
*/
	ros::spin();

	return 0;
}
