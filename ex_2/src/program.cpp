#include <iostream>
#include <cmath>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

int main(int argc, char** argv)
{
	double alpha[3];
	double tetha[3];
	double a[3];//niepotrzebne
	double d[3];//niepotrzebne
	double alfa, beta, gamma;
	
	for (int i=0; i<3; i++)
	{
		d[i]=0;
		tetha[i]=0;
		a[i]=0;
		alpha[i]=0;
	}

	//example values
	tetha[0]=0;//zmienne
	
	alpha[1]=-1.57;
	tetha[1]=-1;//zmienne
	
	a[2]=0.5;//niepotrzbne
	tetha[2]=0.3;//zmienne

	//KDL::Rotation r1;
    	//Creating a rotation matrix out of three unit vectors Vx, Vy,
    	//Vz. Be careful, these vectors should be normalised and
    	//orthogonal. Otherwise this can result in an inconsistent
    	//rotation matrix

	KDL::Rotation r0(KDL::Vector(cos(tetha[0]),sin(tetha[0])*cos(alpha[0]),sin(tetha[0])*sin(alpha[0])),
                     KDL::Vector(-sin(tetha[0]),cos(tetha[0])*cos(alpha[0]),cos(tetha[0])*sin(alpha[0])),
                     KDL::Vector(0,-sin(alpha[0]),cos(alpha[0])));

	KDL::Rotation r1(KDL::Vector(cos(tetha[1]),sin(tetha[1])*cos(alpha[1]),sin(tetha[1])*sin(alpha[1])),
                     KDL::Vector(-sin(tetha[1]),cos(tetha[1])*cos(alpha[1]),cos(tetha[1])*sin(alpha[1])),
                     KDL::Vector(1,-sin(alpha[1]),cos(alpha[1])));

	KDL::Rotation r2(KDL::Vector(cos(tetha[2]),sin(tetha[2])*cos(alpha[2]),sin(tetha[2])*sin(alpha[2])),
                     KDL::Vector(-sin(tetha[2]),cos(tetha[2])*cos(alpha[2]),cos(tetha[2])*sin(alpha[2])),
                     KDL::Vector(0,-sin(alpha[2]),cos(alpha[2])));

	r0.GetRPY(alfa,beta,gamma);
	std::cout<<"Rotation_joint - Roll-Pitch-Yaw: "<<alfa<<", "<<beta<<", "<<gamma<<std::endl;

	r1.GetRPY(alfa,beta,gamma);
	std::cout<<"Shoulder - Roll-Pitch-Yaw: "<<alfa<<", "<<beta<<", "<<gamma<<std::endl;

	r2.GetRPY(alfa,beta,gamma);
	std::cout<<"Elbow - Roll-Pitch-Yaw: "<<alfa<<", "<<beta<<", "<<gamma<<std::endl;

	return(0);
}
