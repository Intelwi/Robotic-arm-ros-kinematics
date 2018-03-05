#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string>


int main(int argc, char **argv)
{
   double linear_=0, angular_=0, l_scale_=2, a_scale_=2;
   int kfd = 0;
   struct termios cooked, raw;
   std::string up , down , left , right;
  
  ros::init(argc, argv, "teleop_turtle");

  
  ros::NodeHandle n;
  
  bool ok0 = n.getParamCached("my_teleop/up", up);
  bool ok1 = n.getParamCached("my_teleop/down", down);
  bool ok2 = n.getParamCached("my_teleop/left", left);
  bool ok3 = n.getParamCached("my_teleop/right", right);
  
  if ( !ok0 || !ok1 || !ok2 || !ok3) {
  	puts("ERROR OCCURED"); 
  	exit (1) ;
  }

  ros::Publisher twist_pub_ = n.advertise<geometry_msgs::Twist> ("turtle1/cmd_vel", 1);

    ros::Rate loop_rate(10);

    //--------------------------                                                          
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
    puts("Use default keys to move the turtle.");
    printf("Up - %c\n", up[0]);
    printf("Down - %c\n", down[0]);
    printf("Left - %c\n", left[0]);
    printf("Right - %c\n", right[0]);
    puts("Or set your own.");

    while (ros::ok())
    {
  	bool ok0 = n.getParamCached("my_teleop/up", up);
  	bool ok1 = n.getParamCached("my_teleop/down", down);
  	bool ok2 = n.getParamCached("my_teleop/left", left);
  	bool ok3 = n.getParamCached("my_teleop/right", right);
  
   if ( !ok0 || !ok1 || !ok2 || !ok3) {
  	puts("ERROR OCCURED"); 
  	exit (1) ;
   }

	char c;
	bool send = false;

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


  return 0;
}

