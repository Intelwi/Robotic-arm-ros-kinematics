#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

double angle[2];// angles from joint_state_publisher
double link_bombel[1];// length of links

void callbackJointState(const sensor_msgs::JointState::ConstPtr& state)
{
  for(int i; i<3; i++)
  {
   	angle[i]=state->position[i];
   	std::cout<<"I heard : "<<state->name[i]<<" , value: "<<angle[i]<<std::endl;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "NONKDL_DKIN");
  
  geometry_msgs::PoseStamped msg;
  ros::NodeHandle n;

  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 1); 

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


    msg.pose.position.x =0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;

    msg.pose.orientation.x = -4.88070518543e-05;
    msg.pose.orientation.y = -0.70080730609;
    msg.pose.orientation.z = 7.50503570579e-05;
    msg.pose.orientation.w = 0.713350613677;



    poseStampedPub.publish(msg); //sending geometry_msgs
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
