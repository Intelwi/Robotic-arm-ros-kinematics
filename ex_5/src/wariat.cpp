#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "ex_5/WariatControlSrv.h"
#include <math.h>

#define PI 3.14159265358

ros::Publisher poseStatePub1;
ros::Publisher poseStatePubPath1;
double position_0[3]; //współrzędne początkowe xyz 
double position_solv[3]; //współrzędne wyliczone
int sampling = 60;
int k=0;
int status;
double ttime, x, y, z, a, b, t, help;
//------------
bool finished;
int count;
std::string shape;


// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
int linear_inter(int mode)
{
	//--------------
	finished = false;
	int count = 0;
	double mo, tt, wys;
	bool back = true;
	//--------------
	status=0;
	nav_msgs::Path path; // ścieżka
	
	ros::Rate loop_rate(50);

	if(ttime <= 0)
	{
		ROS_WARN("\nCzas musi byc wiekszy od zera!\n");
		return -1;
	}
	if(mode != 1 && mode != 2)
	{
		ROS_WARN("\nNie ma takiego trybu.\n");
		return -2;
	}

	if(shape == "ellipse")
	{
		t=0;
		a=x; // wielka półoś
		b=y; // mała półoś
		// x = a*cos(t) = a = x
		y=0; // y = b*sin(t)
	}
	else if(shape == "ellipse2")
	{
		wys = position_0[2];
		t=0;
		a=y; // wielka półoś
		b=z; // mała półoś
		// y = a*cos(t) = a = y
		z=wys; // y = b*sin(t)
	}
	
	// powolne docieranie do punktu początkowego kształtu 
	tt = ttime;
	mo = mode;
	ttime = 2;
	mode = 2;

	while(ros::ok()) //!finished
	{
		k=0;
			
		if(finished)
		{
			finished = false;
			count = 0;
			t=0;
		}
		
		// Obliczenia dla metody trapezowej
		double c1 = -(x - position_0[0]);
		double d1 = x - position_0[0];

		double c2 = -(y - position_0[1]);
		double d2 = y - position_0[1];

		double c3 = -(z - position_0[2]);
		double d3 = z - position_0[2];
		
		
	//-----------------------interpolating loop:--------------->>>
		while(k <= sampling*ttime)
		{

			geometry_msgs::PoseStamped msg;

			msg.header.frame_id="/base_link";
			msg.header.stamp = ros::Time::now();

			path.header.stamp = ros::Time::now();
			path.header.frame_id = "/base_link";


			k++;

	//---------1st mode next position:

			if(mode == 1)
			{
				position_solv[0] = position_0[0] + (x - position_0[0])*k/(ttime*sampling);

				position_solv[1] = position_0[1] + (y - position_0[1])*k/(ttime*sampling);

				position_solv[2] = position_0[2] + (z - position_0[2])*k/(ttime*sampling);
			}

	//---------2nd mode next position:


			else if(mode == 2)
			{
				float tx = k/(ttime*sampling);

				position_solv[0] = (1-tx)*position_0[0] + tx*x + tx*(1-tx)*(c1*(1-tx) + d1*tx);

				position_solv[1] = (1-tx)*position_0[1] + tx*y + tx*(1-tx)*(c2*(1-tx) + d2*tx);

				position_solv[2] = (1-tx)*position_0[2] + tx*z + tx*(1-tx)*(c3*(1-tx) + d3*tx);
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
	//-----------------------interpolating loop----------------<<<
		
		if(back)
		{
			ttime = tt;
			mode = mo;
			back=false;
		}
		
		for(int i=0;i<3;i++) position_0[i] = position_solv[i]; // nowe pozycje jako pozycje poczatkowe
		
		/* kwadrat poziomy */
		if(shape == "square")
		{
			help=x;
			x=y;
			y=-help;
			count++;
			if(count==5)
				finished = true;
			ROS_INFO("Pentla: %d. x: %f, y: %f", count, x, y);
		}
		/* elipsa pozioma */
		else if(shape == "ellipse")
		{
			if(t < 2*PI)
			{
				t=t+PI/20; // elipsa z 40-tu elemetów
				x=a*cos(t);
				y=b*sin(t);
				
				count++;
				ROS_INFO("Pentla: %d. x: %f, y: %f", count, x, y);
			}
			else
				finished = true;
		}
		/* elipsa pionowa */
		else if(shape == "ellipse2")
		{
			if(t < 2*PI)
			{
				t=t+PI/20; // elipsa z 40-tu elemetów
				y=a*cos(t);
				z=b*sin(t)+wys;
				
				count++;
				ROS_INFO("Pentla: %d. x: %f, y: %f", count, x, y);
			}
			else
				finished = true;
		}
		else
		{
			ROS_ERROR("Bledny ksztalt", count, x, y);
			status = 1;
			finished = true;
		}
	}

	return status;
}



bool doAJob(ex_5::WariatControlSrv::Request &req,  ex_5::WariatControlSrv::Response &res)
{
	ttime = req.ttime;
	x = req.x;
	y = req.y;
	z = req.z;
	shape = req.shape;
	
	ROS_INFO("mode: %d, time=%f, x=%f, y=%f, z=%f, %s", req.mode, ttime, x, y, z, shape.c_str());

	res.status = linear_inter(req.mode);
  
	ROS_INFO("sending back response: [%d]", res.status);
	return true;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "wariat");
	ros::NodeHandle n;

// tety początkowe:
	for(int i=0; i<3; i++) position_solv[i]=0;

	position_0[0] = 0.5*0.362357754+0.3;
	position_0[1] = 0;
	position_0[2] = 0.5*0.932039086+0.3;
	
	ros::Publisher poseStatePub = n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1); 
	poseStatePub1 = poseStatePub;
	
	ros::Publisher poseStatePubPath = n.advertise<nav_msgs::Path>("path", 1); //do publikowania ścieżek
	poseStatePubPath1 = poseStatePubPath;// do publikowania ścieżek
	
	ros::ServiceServer service = n.advertiseService("wariat_control_srv", doAJob);
	ROS_INFO("Ready to do a job.");

	ros::spin();

	return 0;
}
