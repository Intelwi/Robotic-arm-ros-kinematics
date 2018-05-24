#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

double angle[3];// angles from joint_state_publisher
double link_bombel[2];// length of links
double qaternion[3];//values of quaternions to send

void callbackJointState(const sensor_msgs::JointState::ConstPtr& state)
{
  for(int i; i<3; i++)
  {
   	angle[i]=state->position[i];
   	//std::cout<<"I heard : "<<state->name[i]<<" , value: "<<angle[i]<<std::endl;
	
  }
  /*ROS_ERROR("\njeden\n");
	if(-3.14> angle[0] || angle[0]>3.14 || -1.40>angle[1] || angle[1]>0 || 0> angle[2] || angle[2]>1.57)
	{	
		//ROS_ERROR("\nWRONG ANGLE!\n");
		for(int i; i<3; i++)
		{
   			angle[i]=1;
  		}
		ROS_ERROR("\nWRONG ANGLE!\n");
		
	}
ROS_ERROR("\ndwa\n");*/
	
}

int main(int argc, char **argv)
{
	
   double alpha[3];
	
   for (int i=0; i<3; i++) alpha[i]=0;

  alpha[1]=-1.57;
  ros::init(argc, argv, "NONKDL_DKIN");
  
  geometry_msgs::PoseStamped msg;
  ros::NodeHandle n;

  ros::Publisher poseStampedPub = n.advertise<geometry_msgs::PoseStamped>("NONKDL_DKIN", 1); 

  ros::Subscriber joint_state = n.subscribe("joint_states", 10, callbackJointState);

  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
	ros::Rate loop_rate(10);
	bool ok0 = n.getParamCached("a2_length", link_bombel[0]);// get from parameter server
	bool ok1 = n.getParamCached("a3_length", link_bombel[1]);// get from parameter server

	if ( !ok0 || !ok1 )
	{
			puts("ERROR OCCURED"); 
			exit(1);
	}

    geometry_msgs::PoseStamped msg;

    msg.header.frame_id="/base_link";
    msg.header.stamp = ros::Time::now();

//-------NON_KDL_Matrix
//-------------------------------
    double r0[3][3], r1[3][3], r2[3][3],r10[3][3], r3[3][3], vr[3], vh[3];
    r0[0][0]=cos(angle[0]);
    r0[0][1]=-sin(angle[0]);
    r0[0][2]=0;
    r0[1][0]=sin(angle[0]);
    r0[1][1]=cos(angle[0]);
    r0[1][2]=0;
    r0[2][0]=0;
    r0[2][1]=0;
    r0[2][2]=1;

    r1[0][0]=cos(angle[1]);
    r1[0][1]=-sin(angle[1]);
    r1[0][2]=0;
    r1[1][0]=0;
    r1[1][1]=0;
    r1[1][2]=1;
    r1[2][0]=-sin(angle[1]);
    r1[2][1]=-cos(angle[1]);
    r1[2][2]=0;

    r2[0][0]=cos(angle[2]);
    r2[0][1]=-sin(angle[2]);
    r2[0][2]=0;
    r2[1][0]=sin(angle[2])*cos(alpha[2]);
    r2[1][1]=cos(angle[2])*cos(alpha[2]);
    r2[1][2]=-sin(alpha[2]);
    r2[2][0]=sin(angle[2])*sin(alpha[2]);
    r2[2][1]=cos(angle[2])*sin(alpha[2]);
    r2[2][2]=cos(alpha[2]);

    vr[0] = link_bombel[1];
    vr[1] = 0;
    vr[2] = 0;

    // Initializing elements of matrix mult to 0.
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        {
            r3[i][j]=0;
	    r10[i][j]=0;
        }

    // Multiplying matrix r1 and r2 and storing in array mult.
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            for(int k = 0; k < 3; ++k)
            {
                r10[i][j] += r0[i][k] * r1[k][j];
            }

   // Multiplying matrix r1 and r2 and storing in array mult.
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
            for(int k = 0; k < 3; ++k)
            {
                r3[i][j] += r10[i][k] * r2[k][j];
            }

   KDL::Vector v1(r3[0][0] ,r3[1][0],r3[2][0]);


   KDL::Vector v2(r3[0][1] ,r3[1][1],r3[2][1]);

   KDL::Vector v3(r3[0][2] ,r3[1][2],r3[2][2]); 
v1.Normalize();
v2.Normalize();
v3.Normalize();  
   KDL::Rotation r3_1(v1,v2,v3);


// Multiplying matrix r1 and r2 and storing in array mult.
    for(int i = 0; i < 3; ++i)
            for(int k = 0; k < 3; ++k)
            {
                vh[i] += r3[i][k] * vr[k];
            }
//std::cout<<vh[0]<<" "<<vh[1]<<" "<<vh[2]<<std::endl;

KDL::Vector v4(link_bombel[1],0,0);

KDL::Vector vl=r3_1*v4;

//--------Getting_Quaternions-------------
    r3_1.GetQuaternion(qaternion[0],qaternion[1],qaternion[2],qaternion[3]);
//-----------------------------------------    
    msg.pose.position.x =cos(angle[0])*cos(angle[1])*link_bombel[0]+vl[0];
    msg.pose.position.y = sin(angle[0])*cos(angle[1])*link_bombel[0]+vl[1];
    msg.pose.position.z = -sin(angle[1])*link_bombel[0]+0.3+vl[2];


    msg.pose.orientation.x = qaternion[0];
    msg.pose.orientation.y = qaternion[1];
    msg.pose.orientation.z = qaternion[2];
    msg.pose.orientation.w = qaternion[3];



    poseStampedPub.publish(msg); //sending geometry_msgs
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

//---------Matrix----------------
//macierz 2 układu względem 1
/**
KDL::Vector v1(cos(angle[0])*cos(angle[1]) ,sin(angle[0])*cos(angle[1]),-sin(angle[1]) );


KDL::Vector v2(-cos(angle[0])*sin(angle[1]) ,-sin(angle[0])*sin(angle[1]),-cos(angle[1])  );

KDL::Vector v3(-sin(angle[0]),cos(angle[0]),0);
    
v1.Normalize();
v2.Normalize();
v3.Normalize();

KDL::Rotation r1(v1,v2,v3);

//macierz 3 układu wzgledem 2

KDL::Rotation r2(KDL::Vector(cos(angle[2]),sin(angle[2])*cos(alpha[2]),sin(angle[2])*sin(alpha[2])),
                     KDL::Vector(-sin(angle[2]),cos(angle[2])*cos(alpha[2]),cos(angle[2])*sin(alpha[2])),
                     KDL::Vector(0,-sin(alpha[2]),cos(alpha[2])));

//złozenie macierzy    
KDL::Rotation r3=r1*r2;
*/

