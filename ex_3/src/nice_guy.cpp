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

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::PoseStamped msg;

    msg.pose.position.x = 0.2;
    msg.pose.position.y = 0.3;
    msg.pose.position.z = 0.4;
    msg.pose.orientation.w = 1;

    poseStampedPub.publish(msg);
    std::cout<<"hello"<<std::endl;
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
