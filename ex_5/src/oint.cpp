#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "ex_4/OintControlSrv.h"

ros::Publisher poseStatePub1;
ros::Publisher poseStatePubPath1;
double teta_0[3]; // y(0)
double teta_solv[3]; // rozwiazania
double position_0[3]; //współrzędne początkowe xyz 
double position_solv[3]; //współrzędne wyliczone
double qaternion[4];
int sampling = 60;
int k=0;
int status;
double ttime, teta1, teta2, teta3,x,y,z;


// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
int linear_inter(int mode)
{
	k=0;
	status=0;
	nav_msgs::Path path; // ścieżka
	
	ros::Rate loop_rate(50);

	if(ttime <= 0)
	{
		ROS_WARN("\nCzas musi byc wiekszy od zera!\n");
		return -1;
	}

	//------Obliczenia dla metody trapezowej-------------->>>
		//-----pozycje
	double c1 = -(x - position_0[0]);
	double d1 = x - position_0[0];

	double c2 = -(y - position_0[1]);
	double d2 = y - position_0[1];

	double c3 = -(z - position_0[2]);
	double d3 = z - position_0[2];

	//-----------------------counting loop:--------------->>>

	while(k <= sampling*ttime)
	{

		geometry_msgs::PoseStamped msg;

		msg.header.frame_id="/base_link";
		msg.header.stamp = ros::Time::now();

		path.header.stamp = ros::Time::now();
		path.header.frame_id = "/base_link";
		

		k++;

	//---------1st mode next tetas:

		if(mode == 1)
		{
			position_solv[0] = position_0[0] + (x - position_0[0])*k/(ttime*sampling);

			position_solv[1] = position_0[1] + (y - position_0[1])*k/(ttime*sampling);

			position_solv[2] = position_0[2] + (z - position_0[2])*k/(ttime*sampling);

		}

	//---------2nd mode next tetas:


		else if(mode == 2)
		{
			float tx = k/(ttime*sampling);
			//---------pozycja--------------
			position_solv[0] = (1-tx)*position_0[0] + tx*x + tx*(1-tx)*(c1*(1-tx) + d1*tx);

			position_solv[1] = (1-tx)*position_0[1] + tx*y + tx*(1-tx)*(c2*(1-tx) + d2*tx);

			position_solv[2] = (1-tx)*position_0[2] + tx*z + tx*(1-tx)*(c3*(1-tx) + d3*tx);
		
		}

		else {
			ROS_WARN("\nNie ma takiego trybu.\n");
			return -2;
		}

//---------sending:

		    msg.pose.position.x = position_solv[0];
		    msg.pose.position.y = position_solv[1];
		    msg.pose.position.z = position_solv[2];

		poseStatePub1.publish(msg);

		path.poses.push_back(msg); // ładowanie kolejnego elementu ścieżki
		
		poseStatePubPath1.publish(path); //do wyświetlania śieżki
		ros::spinOnce();

		loop_rate.sleep();
	}

//-----------------------counting loop----------------<<<

	for(int i=0;i<3;i++) position_0[i] = position_solv[i]; // nowe pozycje jako pozycje poczatkowe

	return 0;
}



bool doAJob(ex_4::OintControlSrv::Request &req,  ex_4::OintControlSrv::Response &res)
{
	ROS_INFO("mode: %d, time=%f, x=%f, y=%f, z=%f", req.mode, req.ttime, req.x, req.y, req.z);

	ttime = req.ttime;

	x = req.x;
	y = req.y;
	z = req.z;

	res.status = linear_inter(req.mode);
  
	ROS_INFO("sending back response: [%d]", res.status);
	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "oint");
	ros::NodeHandle n;

// pozycje początkowe:
	for(int i=0; i<3; i++) position_solv[i]=0;

	position_0[0] = 0;
	position_0[1] = 0;
	position_0[2] = 0;

	ros::Publisher poseStatePub = n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1); 
	poseStatePub1 = poseStatePub;
	
	ros::Publisher poseStatePubPath = n.advertise<nav_msgs::Path>("path", 1); //do publikowania ścieżek
	poseStatePubPath1 = poseStatePubPath;// do publikowania ścieżek
	
	ros::ServiceServer service = n.advertiseService("oint_control_srv", doAJob);
	ROS_INFO("Ready to do a job.");

	ros::spin();

	return 0;
}
