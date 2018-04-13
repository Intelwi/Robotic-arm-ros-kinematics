#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

//void callbackJointState(const JointStateConstPtr& state)
//{
 // ROS_INFO("I heard: [%s]", msg->data.c_str());
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  
  geometry_msgs::PoseStamped msg; //obiekt PoseStamped
  ros::NodeHandle n;

  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("poseStampedDsr", 1); 

  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  //ros::Subscriber joint_state = n_.subscribe("joint_states", 10, callbackJointState);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
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

ROS_INFO("poseStamped orientation: %lf, %lf, %lf, %lf",
msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w);

    poseStampedPub.publish(msg);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
