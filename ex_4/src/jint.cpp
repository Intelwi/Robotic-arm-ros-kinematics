#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Float64.h>
#include <sstream>
#include "ex_4/JintControlSrv.h"

ros::Publisher JointStatePub1;
ros::Publisher poseStatePubPath1;
float teta_0[3]; // y(0)
float teta_solv[3]; // rozwiazania
int sampling = 60;
int k=0;
int status;
float ttime, teta1, teta2, teta3;


// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
int linear_inter(int mode)
{
	k=0;
	status=0;
	ros::Rate loop_rate(50);
/** Do zadania 4, zakomentowane bo uzywamy do piatego gdzie nie moze byc ograniczen
	if(ttime <= 0)
	{
		ROS_WARN("\nCzas musi byc wiekszy od zera!\n");
		return -1;
	}
	if(teta1 > 2.14 || teta1 < -2.14)
	{
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "rotation_joint");
		status++;
	}
	if(teta2 > -0.1 || teta2 < -1.50) 
	{
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "shoulder");
		status++;
	}
	if(teta3 > 1.50 || teta3 < 0.1) 
	{
		ROS_WARN("\n---Wykroczenie detected---: %s\n", "elbow");
		status++;
	}
*/
	if(status > 0) return status;


	float a1 = -(teta1 - teta_0[0]);
	float b1 = teta1 - teta_0[0];

	float a2 = -(teta2 - teta_0[1]);
	float b2 = teta2 - teta_0[1];

	float a3 = -(teta3 - teta_0[2]);
	float b3 = teta3 - teta_0[2];

//-----------------------counting loop:--------------->>>

	while(k <= sampling*ttime)
	{
		sensor_msgs::JointState msg;
		msg.header.stamp = ros::Time::now(); //czas ruchu - bez tego ani rusz!

		k++;

//---------1st mode next tetas:

		if(mode == 1)
		{
			teta_solv[0] = teta_0[0] + (teta1 - teta_0[0])*k/(ttime*sampling);

			teta_solv[1] = teta_0[1] + (teta2 - teta_0[1])*k/(ttime*sampling);

			teta_solv[2] = teta_0[2] + (teta3 - teta_0[2])*k/(ttime*sampling);
		}

//---------2nd mode next tetas:

// y1 = teta_0[X];
// y2 = tetaX;

		else if(mode == 2)
		{
			float tx = k/(ttime*sampling);

			teta_solv[0] = (1-tx)*teta_0[0] + tx*teta1 + tx*(1-tx)*(a1*(1-tx) + b1*tx);

			teta_solv[1] = (1-tx)*teta_0[1] + tx*teta2 + tx*(1-tx)*(a2*(1-tx) + b2*tx);

			teta_solv[2] = (1-tx)*teta_0[2] + tx*teta3 + tx*(1-tx)*(a3*(1-tx) + b3*tx);
		}

		else {
			ROS_WARN("\nNie ma takiego trybu.\n");
			return -2;
		}

//---------sending:

		for(int i=0;i<3;i++) msg.position.push_back(teta_solv[i]);
	
		msg.name.push_back("rotation_joint");
		msg.name.push_back("shoulder");
		msg.name.push_back("elbow");

		JointStatePub1.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

//-----------------------counting loop----------------<<<

	for(int i=0;i<3;i++) teta_0[i] = teta_solv[i]; // nowe tety jako tety początkowe

	return 0;
}



bool doAJob(ex_4::JintControlSrv::Request &req,  ex_4::JintControlSrv::Response &res)
{
	ROS_INFO("mode: %d, time=%f, rotation=%f, shoulder=%f, elbow=%f", req.mode, req.ttime, req.teta1, req.teta2, req.teta3);

	ttime = req.ttime;
	teta1 = req.teta1;
	teta2 = req.teta2;
	teta3 = req.teta3;

	res.status = linear_inter(req.mode);
  
	ROS_INFO("sending back response: [%d]", res.status);
	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "jint");
	ros::NodeHandle n;

// tety początkowe:
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
