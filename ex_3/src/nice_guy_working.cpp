#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

double angle[3];// angles from joint_state_publisher
double link_bombel[2];// length of links
double qaternion[3];//values of quaternions to send

void callbackJointState(const sensor_msgs::JointState::ConstPtr& state)
{
  for(int i; i<3; i++)
  {
   	angle[i]=state->position[i];
   	//std::cout<<"I heard : "<<state->name[i]<<" , value: "<<angle[i]<<std::endl;
  }
}

int main(int argc, char **argv)
{


	double alpha[3];
	
	for (int i=0; i<3; i++) alpha[i]=0;

	alpha[1]=-1.57;
	
	
  ros::init(argc, argv, "NONKDL_DKIN");
  
  geometry_msgs::PoseStamped msg;
  ros::NodeHandle n;

  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("KDL_DKIN", 1); 

  ros::Subscriber joint_state = n.subscribe("joint_states", 10, callbackJointState);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {

	bool ok0 = n.getParamCached("a2_length", link_bombel[0]);// get from parameter server
	bool ok1 = n.getParamCached("a3_length", link_bombel[1]);// get from parameter server

	if ( !ok0 || !ok1 )
	{
			puts("ERROR OCCURED"); 
			exit(1);
	}

    geometry_msgs::PoseStamped msg;

    msg.header.frame_id="/base_link";
    msg.header.stamp = ros::Time::now();

//---------Matrix----------------
//macierz 2 układu względem 0

KDL::Vector v1(cos(angle[0])*cos(angle[1]) ,sin(angle[0])*cos(angle[1]),-sin(angle[1]) );


KDL::Vector v2(-cos(angle[0])*sin(angle[1]) ,-sin(angle[0])*sin(angle[1]),-cos(angle[1])  );

KDL::Vector v3(-sin(angle[0]),cos(angle[0]),0);
    
v1.Normalize();
v2.Normalize();
v3.Normalize();

KDL::Rotation r1(v1,v2,v3);

//macierz 3 układu wzgledem 2

KDL::Rotation r2(KDL::Vector(cos(angle[2]),sin(angle[2])*cos(alpha[2]),sin(angle[2])*sin(alpha[2])),
                     KDL::Vector(-sin(angle[2]),cos(angle[2])*cos(alpha[2]),cos(angle[2])*sin(alpha[2])),
                     KDL::Vector(0,-sin(alpha[2]),cos(alpha[2])));

//złozenie macierzy    
KDL::Rotation r3=r1*r2;

//--------Getting_Quaternions-------------
    r3.GetQuaternion(qaternion[0],qaternion[1],qaternion[2],qaternion[3]);
//-----------------------------------------    
    msg.pose.position.x =0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;

    msg.pose.orientation.x = qaternion[0];
    msg.pose.orientation.y = qaternion[1];
    msg.pose.orientation.z = qaternion[2];
    msg.pose.orientation.w = qaternion[3];



    poseStampedPub.publish(msg); //sending geometry_msgs
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
