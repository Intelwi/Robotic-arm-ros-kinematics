#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>


class TeleopTurtle
{
public:
	TeleopTurtle();
	void keyLoop();
	void getKeys();
	void keysChanged();
	void setPrevKeys();

private:
	ros::NodeHandle nh;
	ros::Publisher twist_pub_;
	double linear_, angular_, l_scale_, a_scale_;
	std::string up , down , left , right;
	std::string p_up , p_down , p_left , p_right; // previous keys
};


TeleopTurtle::TeleopTurtle():
	linear_(0),
		angular_(0),
			l_scale_(2.0),
				a_scale_(2.0)
{
	getKeys();
	twist_pub_ = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
}


int kfd = 0;
struct termios cooked, raw;


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_turtle");

	TeleopTurtle teleop_turtle;

	teleop_turtle.getKeys();
	teleop_turtle.setPrevKeys();
	teleop_turtle.keyLoop();

	return(0);
}


void TeleopTurtle::getKeys()
{
	bool ok0 = nh.getParamCached("my_teleop/up", up);
	bool ok1 = nh.getParamCached("my_teleop/down", down);
	bool ok2 = nh.getParamCached("my_teleop/left", left);
	bool ok3 = nh.getParamCached("my_teleop/right", right);

	if ( !ok0 || !ok1 || !ok2 || !ok3)
	{
		puts("ERROR OCCURED"); 
		exit(1);
	}
}


void TeleopTurtle::keysChanged()
{
	puts("Use following keys to move the turtle:");
	printf("Up - %c\n", up[0]);
	printf("Down - %c\n", down[0]);
	printf("Left - %c\n", left[0]);
	printf("Right - %c\n", right[0]);
	puts("Or set your own.");
}


void TeleopTurtle::setPrevKeys()
{
	p_up = up;
	p_down = down;
	p_left = left;
	p_right = right;
}


void TeleopTurtle::keyLoop()
{
	ros::Rate loop_rate(10);

	// get the console in the raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                         
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	//-----------------------------

	puts("Reading from keyboard");
	puts("---------------------------");
	keysChanged();

	while (ros::ok())
	{
		getKeys();

		if(p_up != up || p_down != down || p_left != left || p_right != right)
		{
			puts("---------------------------");
			puts("Control keys changed!");
			keysChanged();
			setPrevKeys();
		}

		char c;
		bool send = false;

		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			exit(-1);
		}

		linear_=angular_=0;

		if(c == left[0])
		{
			angular_ = 1.0;
			send = true;
		}
		else if (c == right[0])
		{
			angular_ = -1.0;
			send = true;
		}
		else if (c == up[0])
		{
			linear_ = 1.0;
			send = true;
		}
		else if (c == down[0])
		{
			linear_ = -1.0;
			send = true;
		}

		geometry_msgs::Twist twist;

		twist.angular.z = a_scale_*angular_;
		twist.linear.x = l_scale_*linear_;

		if(send ==true)
		{
			twist_pub_.publish(twist);    
			send=false;
		}
	}

	ros::spinOnce();
	loop_rate.sleep();

	return;
}
