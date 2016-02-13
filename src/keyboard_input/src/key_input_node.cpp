#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <unistd.h>
#include <termios.h>

#define KEY_w 119
#define KEY_s 115
#define KEY_a 97
#define KEY_d 100
#define KEY_space 32

/*
char getch()
{
	char buf = 0;
	struct termios old = {0};
	if (tcgetattr(0, &old) < 0)
		perror("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(0, TCSANOW, &old) < 0)
		perror("tcsetattr ICANON");
//	lseek(0, -1L, SEEK_END);
	if (read(0, &buf, 1) < 0)
		perror ("read()");
	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(0, TCSADRAIN, &old) < 0)
		perror ("tcsetattr ~ICANON");
	return (buf);
}
*/

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "key_input_node");
	ros::NodeHandle n;
	ros::Publisher pwm_pub = n.advertise<geometry_msgs::Vector3>("vector_pwm_arduino", 1);

	ros::Rate loop_rate(10);
	int pwm = 0;
	while (ros::ok())
	{
		geometry_msgs::Vector3 vector;
		int c = 0;
		switch((c=getch()))
		{
			case KEY_w:
				vector.x=(+1)*pwm;	//left motor pwm
				vector.y=(+1)*pwm;	//right motor pwm
				vector.z=1;	//forward both
				ROS_INFO("Up");//cout << endl << "Up" << endl;//key up
				break;
			case KEY_s:
				vector.x=(-1)*pwm;
				vector.y=(-1)*pwm;
				vector.z=1;	//backward both
				ROS_INFO("Down");//cout << endl << "Down" << endl;   // key down
				break;
			case KEY_d:
				vector.x=(+1)*pwm;
				vector.y=(-1)*pwm;
				vector.z=1;	// left forward right backward
				ROS_INFO("Right");//cout << endl << "Right" << endl;  // key right
				break;
			case KEY_a:
				vector.x=(-1)*pwm;
				vector.y=(+1)*pwm;
				vector.z=1;	// left backward right forward
				ROS_INFO("Left");//cout << endl << "Left" << endl;  // key left
				break;
			case KEY_space:
				vector.x=0;
				vector.y=0;
				vector.z=-1;	// both motor stop
				ROS_INFO("Brake");//cout << endl << "Left" << endl;  // key left
				break;
			default:
				if(c>=48&&c<=57)
				{
					pwm=(255*(c-48))/(57-48);
					ROS_INFO("pwm = %d", pwm);
				}
				else
					ROS_INFO("%d", c);
				break;
		}
		pwm_pub.publish(vector);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
