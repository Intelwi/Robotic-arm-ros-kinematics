#include "ros/ros.h"
#include "ex_4/OintControlSrv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "oint_client");
	if (argc != 9)
	{
		ROS_INFO("usage: mode, time, roll, pitch, yaw, x, y, z");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<ex_4::OintControlSrv>("oint_control_srv");

	ex_4::OintControlSrv srv;

	srv.request.mode = atoi(argv[1]);
	srv.request.ttime = atof(argv[2]);
	srv.request.roll = atof(argv[3]);
	srv.request.pitch = atof(argv[4]);
	srv.request.yaw = atof(argv[5]);
	srv.request.x = atof(argv[6]);
	srv.request.y = atof(argv[7]);
	srv.request.z = atof(argv[8]);

	if (client.call(srv))
	{
		if(srv.response.status==0)
		{		
			ROS_INFO("Obliczenia wykonano poprawnie. Status: %d", srv.response.status);
		}	
		else
		{
			ROS_INFO("ERROR: Obliczenia nie sa mozliwe do wykonania. Przekroczono limity stawow. Status: %d", srv.response.status);
		}
	}
	else
	{
		ROS_ERROR("Failed to call service jint");
		return 1;
	}

	return 0;
}
