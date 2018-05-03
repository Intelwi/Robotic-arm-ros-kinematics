#include "ros/ros.h"
#include "ex_4/JintControlSrv.h"

bool add(ex_4::JintControlSrv::Request  &req,
         ex_4::JintControlSrv::Response &res)
{
  res.status = 2; // na razie stała odp
  ROS_INFO("request: x=%f, y=%f, z=%f, q=%f", req.time, req.teta1, req.teta2, req.teta3);
  ROS_INFO("sending back response: [%d]", res.status);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint_control_srv_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("jint_control_srv", add);
  ROS_INFO("Ready to do a job.");
  ros::spin();

  return 0;
}
