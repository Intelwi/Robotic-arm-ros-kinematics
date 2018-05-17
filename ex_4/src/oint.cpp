#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include "ex_4/OintControlSrv.h"

ros::Publisher poseStatePub1;
float teta_0[3]; // y(0)
float teta_solv[3]; // rozwiazania
float position_0[3]; //współrzędne początkowe xyz 
float position_solv[3]; //współrzędne wyliczone
double qaternion[3];
int sampling = 60;
int k=0;
int status;
float ttime, teta1, teta2, teta3,x,y,z;


// interpolacja liniowa, time-czas wykonania ruchu, tetaX - zadane kolejne kąty
int linear_inter(int mode)
{
	k=0;
	status=0;

	
	ros::Rate loop_rate(50);
	float a1 = -(teta1 - teta_0[0]);
	float b1 = teta1 - teta_0[0];

	float a2 = -(teta2 - teta_0[1]);
	float b2 = teta2 - teta_0[1];

	float a3 = -(teta3 - teta_0[2]);
	float b3 = teta3 - teta_0[2];

	//-----------------------counting loop:--------------->>>

	while(k <= sampling*ttime)
	{

		
		geometry_msgs::PoseStamped msg;

		msg.header.frame_id="/base_link";
		msg.header.stamp = ros::Time::now();


		k++;

	//---------1st mode next tetas:

		if(mode == 1)
		{
			position_solv[0] = position_0[0] + (x - position_0[0])*k/(ttime*sampling);

			position_solv[1] = position_0[1] + (y - position_0[1])*k/(ttime*sampling);

			position_solv[2] = position_0[2] + (z - position_0[2])*k/(ttime*sampling);

			teta_solv[0] = teta_0[0] + (teta1 - teta_0[0])*k/(ttime*sampling);

			teta_solv[1] = teta_0[1] + (teta2 - teta_0[1])*k/(ttime*sampling);

			teta_solv[2] = teta_0[2] + (teta3 - teta_0[2])*k/(ttime*sampling);
		}

	//---------2nd mode next tetas:######### NIE OBSŁUŻONE #######################


		/*else if(mode == 2)
		{
			float tx = k/(ttime*sampling);

			teta_solv[0] = (1-tx)*teta_0[0] + tx*teta1 + tx*(1-tx)*(a1*(1-tx) + b1*tx);

			teta_solv[1] = (1-tx)*teta_0[1] + tx*teta2 + tx*(1-tx)*(a2*(1-tx) + b2*tx);

			teta_solv[2] = (1-tx)*teta_0[2] + tx*teta3 + tx*(1-tx)*(a3*(1-tx) + b3*tx);
		}*/

		else {
			ROS_WARN("\nNie ma takiego trybu.\n");
			return -2;
		}

//---------sending:
		KDL::Rotation roll(KDL::Vector(1,0,0),
                     	KDL::Vector(0,cos(teta_solv[0]),sin(teta_solv[0])),
			KDL::Vector(0,-sin(teta_solv[0]),cos(teta_solv[0])));

		KDL::Rotation pitch(KDL::Vector(cos(teta_solv[1]),0,-sin(teta_solv[1])),
                     	KDL::Vector(0,1,0),
			KDL::Vector(sin(teta_solv[1]),0,cos(teta_solv[1])));

		KDL::Rotation yaw(KDL::Vector(cos(teta_solv[2]),sin(teta_solv[2]),0),
                     	KDL::Vector(-sin(teta_solv[2]),cos(teta_solv[2]),0),
			KDL::Vector(0,0,1));
		KDL::Rotation r1 = roll*pitch*yaw;

/*

		KDL::Vector v1(cos(teta_solv[2])*cos(teta_solv[1]),sin(teta_solv[2])*cos(teta_solv[1]),-sin(teta_solv[1]));
		KDL::Vector v2(cos(teta_solv[2])*sin(teta_solv[1])*sin(teta_solv[0])-sin(teta_solv[2])*cos(teta_solv[0]),sin(teta_solv[2])*sin(teta_solv[1])*sin(teta_solv[0])+cos(teta_solv[2])*cos(teta_solv[0]),cos(teta_solv[1])*sin(teta_solv[0]));
		KDL::Vector v3(cos(teta_solv[2])*sin(teta_solv[1])*cos(teta_solv[0])+sin(teta_solv[2])*sin(teta_solv[0]),sin(teta_solv[2])*sin(teta_solv[1])*cos(teta_solv[0])-cos(teta_solv[2])*sin(teta_solv[0]),cos(teta_solv[1])*cos(teta_solv[0]));

		 v1.Normalize();
		 v2.Normalize();
		 v3.Normalize();
		KDL::Rotation r1(v1,v2,v3);


		KDL::Rotation r1(KDL::Vector(cos(position_solv[2])*cos(position_solv[1]),sin(position_solv[2])*cos(position_solv[1]),-sin(position_solv[1])),
                     KDL::Vector(cos(position_solv[2])*sin(position_solv[1])*sin(position_solv[0])-sin(position_solv[2])*cos(position_solv[0]),sin(position_solv[2])*sin(position_solv[1])*sin(position_solv[0])+cos(position_solv[2])*cos(position_solv[0]),cos(position_solv[1])*sin(position_solv[0])),
                     KDL::Vector(cos(position_solv[2])*sin(position_solv[1])*cos(position_solv[0])+sin(position_solv[2])*sin(position_solv[0]),sin(position_solv[2])*sin(position_solv[1])*cos(position_solv[0])-cos(position_solv[2])*sin(position_solv[0]),cos(position_solv[1])*cos(position_solv[0])));
*/

		//std::cout<<vr[0]<<" "<<vr[1]<<" "<<vr[2]<<std::endl;
		//--------Getting_Quaternions-------------

		   //r1.GetQuaternion(qaternion[0],qaternion[1],qaternion[2],qaternion[3]);

		//-----------------------------------------    
		    msg.pose.position.x = position_solv[0];
		    msg.pose.position.y = position_solv[1];
		    msg.pose.position.z = position_solv[2];


		    //msg.pose.orientation.x = qaternion[0];
		    //msg.pose.orientation.y = qaternion[1];
		    //msg.pose.orientation.z = qaternion[2];
		    //msg.pose.orientation.w = qaternion[3];

		poseStatePub1.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
	}

//-----------------------counting loop----------------<<<

	for(int i=0;i<3;i++) teta_0[i] = teta_solv[i]; // nowe tety jako tety początkowe
	for(int i=0;i<3;i++) position_0[i] = position_solv[i]; // nowe tety jako tety początkowe

	return 0;
}



bool doAJob(ex_4::OintControlSrv::Request &req,  ex_4::OintControlSrv::Response &res)
{
	ROS_INFO("mode: %d, time=%f, roll=%f, pitch=%f, yaw=%f, x=%f, y=%f, z=%f", req.mode, req.ttime, req.roll, req.pitch, req.yaw, req.x, req.y, req.z);

	ttime = req.ttime;
	teta1 = req.roll;
	teta2 = req.pitch;
	teta3 = req.yaw;

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

// tety początkowe:
	for(int i=0; i<3; i++) teta_solv[i]=0;
	for(int i=0; i<3; i++) position_solv[i]=0;

	position_0[0] = 0;
	position_0[1] = 0;
	position_0[2] = 0;
	
	teta_0[0] = 0;
	teta_0[1] = 0;
	teta_0[2] = 0;

	ros::Publisher poseStatePub = n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1); 
	poseStatePub1 = poseStatePub;

	ros::ServiceServer service = n.advertiseService("oint_control_srv", doAJob);
	ROS_INFO("Ready to do a job.");

	ros::spin();

	return 0;
}
