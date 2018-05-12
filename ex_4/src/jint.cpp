#include "ros/ros.h"
#include "ex_4/JintControlSrv.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

ros::Publisher JointStatePub1;
float teta_0[3]; // y(0)
float teta_solv[3]; // rozwazania
int sampling = 60;
int k=0;

// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
void linear_inter(float time, float teta1, float teta2, float teta3)
{
  k=0;
  ros::Rate loop_rate(50);
  //-----sending-----
  while(k <= sampling*time)
  {

	sensor_msgs::JointState msg;
  	msg.header.stamp = ros::Time::now();//czas ruchu - bez tego ani rusz!

   	k++;

  	teta_solv[0] = teta_0[0] + (teta1 - teta_0[0])*k/(time*sampling);
	teta_solv[1] = teta_0[1] + (teta2 - teta_0[1])*k/(time*sampling);
	teta_solv[2] = teta_0[2] + (teta3 - teta_0[2])*k/(time*sampling);
  	  
  	for(int i=0;i<3;i++) msg.position.push_back(teta_solv[i]);
  		
  	msg.name.push_back("rotation_joint");
  	msg.name.push_back("shoulder");
  	msg.name.push_back("elbow");

  	JointStatePub1.publish(msg);
  	ros::spinOnce();

    	loop_rate.sleep();
  }
  for(int i=0;i<3;i++) teta_0[i] = teta_solv[i];
}

bool add(ex_4::JintControlSrv::Request  &req,
         ex_4::JintControlSrv::Response &res)
{
  res.status = 2; // na razie stała odp
  ROS_INFO("time=%f, rotation=%f, shoulder=%f, elbow=%f", req.time, req.teta1, req.teta2, req.teta3);
  linear_inter(req.time,req.teta1, req.teta2, req.teta3);

  ROS_INFO("sending back response: [%d]", res.status);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint_control_srv_server");
  ros::NodeHandle n;

  for(int i=0; i<3; i++) teta_0[i]=0;

  ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
  JointStatePub1 = JointStatePub;
  ros::ServiceServer service = n.advertiseService("jint_control_srv", add);
  ROS_INFO("Ready to do a job.");
  ros::spin();

  return 0;
}
