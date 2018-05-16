#include "ros/ros.h"
#include "ex_4/JintControlSrv.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

ros::Publisher JointStatePub1;
float teta_0[3]; // y(0)
float teta_solv[3]; // rozwiazania
int sampling = 60;
int k=0;
bool rj=true, sh=true, el=true; // jak true to licz
int  rj_ok, sh_ok, el_ok; // ostatnia teta z zakresu dozwolonego
int status;

// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
int linear_inter(float time, float teta1, float teta2, float teta3)
{
  k=0;
  status=0;
  rj=true, sh=true, el=true;

  ros::Rate loop_rate(50);

	if(time<=0)
	 {
		ROS_WARN("\nCzas mniejszy lub rowny od zera: %s\n", "time");
		return -1;
	}

	if((teta_0[0] + teta1) > 2.14 || (teta_0[0] + teta1) < -2.14)
	 {
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "rotation_joint");
		return -1;
	}

	if((teta_0[1] + teta2) > -0.1 || (teta_0[1] + teta2) < -1.50) 
	{
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "shoulder");
		return -1;
	}

	if((teta_0[2] + teta3) > 1.50 || (teta_0[2] + teta3) < 0.1) 
	{
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "elbow");
		return -1;
	}


  //-----sending-----
  while(k <= sampling*time)
  {
	sensor_msgs::JointState msg;
  	msg.header.stamp = ros::Time::now(); //czas ruchu - bez tego ani rusz!

   	k++;

//-----------------------next tetas:
	
		
		teta_solv[0] = teta_0[0] + (teta1 - teta_0[0])*k/(time*sampling);
	
		;
		teta_solv[1] = teta_0[1] + (teta2 - teta_0[1])*k/(time*sampling);
	
		
		teta_solv[2] = teta_0[2] + (teta3 - teta_0[2])*k/(time*sampling);

//-----------------------next tetas slow:


//-----------------------examining new tetas:



  	for(int i=0;i<3;i++) msg.position.push_back(teta_solv[i]);
  		
  	msg.name.push_back("rotation_joint");
  	msg.name.push_back("shoulder");
  	msg.name.push_back("elbow");

  	JointStatePub1.publish(msg);
  	ros::spinOnce();

    	loop_rate.sleep();
  }

  for(int i=0;i<3;i++) teta_0[i] = teta_solv[i];

  return 1;
}

bool doAJob(ex_4::JintControlSrv::Request  &req,
         ex_4::JintControlSrv::Response &res)
{
  ROS_INFO("time=%f, rotation=%f, shoulder=%f, elbow=%f", req.time, req.teta1, req.teta2, req.teta3);

  res.status = linear_inter(req.time,req.teta1, req.teta2, req.teta3);
  ROS_INFO("sending back response: [%d]", res.status);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint_control_srv_server");
  ros::NodeHandle n;

  for(int i=0; i<3; i++) teta_solv[i]=0;

  teta_0[0] = 0;
  teta_0[1] = -1;
  teta_0[2] = 1;

  ros::Publisher JointStatePub = n.advertise<sensor_msgs::JointState>("joint_states", 1); 
  JointStatePub1 = JointStatePub;
  ros::ServiceServer service = n.advertiseService("jint_control_srv", doAJob);
  ROS_INFO("Ready to do a job.");
  ros::spin();

  return 0;
}
