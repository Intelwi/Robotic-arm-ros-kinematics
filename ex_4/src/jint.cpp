#include "ros/ros.h"
#include "ex_4/JintControlSrv.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

ros::Publisher JointStatePub1;

bool add(ex_4::JintControlSrv::Request  &req,
         ex_4::JintControlSrv::Response &res)
{
  res.status = 2; // na razie stała odp
  ROS_INFO("time=%f, rotation=%f, shoulder=%f, elbow=%f", req.time, req.teta1, req.teta2, req.teta3);
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();//czas ruchu - bez tego ani rusz!
  msg.position.push_back(req.teta1);
  msg.position.push_back(req.teta2);
  msg.position.push_back(req.teta3);


  msg.name.push_back("rotation_joint");
  msg.name.push_back("shoulder");
  msg.name.push_back("elbow");

  JointStatePub1.publish(msg);
  ROS_INFO("sending back response: [%d]", res.status);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint_control_srv_server");
  ros::NodeHandle n;
  ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
  JointStatePub1 = JointStatePub;
  ros::ServiceServer service = n.advertiseService("jint_control_srv", add);
  ROS_INFO("Ready to do a job.");
  ros::spin();

  return 0;
}
