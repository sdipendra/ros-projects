#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <signal.h>

#define pi 3.141592653589793

void signint_handler(int sig)
{
	ros::NodeHandle nh;
	geometry_msgs::Vector3 pwm_vector;
	pwm_vector.x=0;
	pwm_vector.y=0;
	pwm_vector.z=-1;
	ros::Publisher vector_pub;
	vector_pub = nh.advertise<geometry_msgs::Vector3>("vector_pwm_arduino", 1000);
	ROS_INFO("sending kill signal to arduino...!!!");
	vector_pub.publish(pwm_vector);
	ROS_INFO("shutting down...!!!");
	ros::shutdown();
}

//pid control class
class pid_control
{
	private:
		//setpoint parameters
		double setpt;		//regulated level to maintain
		//gain parameters
		double kprop;		//proportionality coefficient
		double ki;		//integrator coefficient
		double kd;		//differentiator coefficient
		double kgain;		//loop gain coefficient
		//controller state parameters
		double integ;		//integral of setpoint errors
		double deriv;		//previous setpoint error
		double time;		//previous time

		//default parameters
		static const double csetpt = 0.0;
		static const double ckprop = 1.0;
		static const double cki = 1.0;
		static const double ckd = 1.0;
		static const double ckgain =1.0;
		static const double cinteg = 0.0;
		static const double cderiv = 0.0;

		//differentiator
		double derivative(double err, double delt)
		{
			double pid_deriv = (kd*(err-deriv))/delt;
			deriv = err;
			ROS_INFO("pid_deriv: %lf", pid_deriv);
			return pid_deriv;
		}
		//integrator
		double integral(double err, double delt)
		{
			integ += err*delt;
			ROS_INFO("ki*integ: %lf", ki*integ);
			return ki*integ;
		}
		//proportional
		double proportional(double err)
		{
			ROS_INFO("kprop*err: %lf", kprop*err);
			return kprop*err;
		}
	public:
		pid_control(double setpt=csetpt, double kprop=ckprop, double ki=cki, double kd=ckd, double kgain=ckgain, double integ=cinteg, double deriv=cderiv, double time=ros::Time::now().toSec()-0.1)
		{
			setpt=this->setpt;
			kprop=this->kprop;
			ki=this->ki;
			kd=this->kd;
			kgain=this->kgain;
			integ=this->integ;
			deriv=this->deriv;
			time=this->time;
		}

		double pid_out(double curr_feedback, double curr_time)
		{
			double err = curr_feedback - setpt;
			double delt = curr_time - time;
			double time = curr_time;
			ROS_INFO("kgain*(derivative(err, delt)+integral(err, delt)+proportional(err)): %lf", kgain*(derivative(err, delt)+integral(err, delt)+proportional(err)));
			return kgain*(derivative(err, delt)+integral(err, delt)+proportional(err));
		}
};

//class magnetometer_heading direction
class direction_follower
{
	private:
		ros::NodeHandle n;
		ros::Subscriber compass_hdg_sub;
		ros::Publisher vector_pub;
		pid_control ground_bot;
	public:
		direction_follower()
		{
			vector_pub = n.advertise<geometry_msgs::Vector3>("vector_pwm_arduino", 1000);
			compass_hdg_sub = n.subscribe("/mavros/global_position/compass_hdg", 1000, &direction_follower::scan_callback, this);
			signal(SIGINT, signint_handler);
		}
		
		void scan_callback(const std_msgs::Float64::ConstPtr& hdg_in);
};

void direction_follower::scan_callback(const std_msgs::Float64::ConstPtr& hdg_in)
{
	ROS_INFO("hdg_in->data: %lf", hdg_in->data);
	double omega = ground_bot.pid_out(hdg_in->data-180.0, ros::Time::now().toSec());
	double omega_highlim = 100000000000.0;
	double omega_lowlim = -100000000000.0;
	geometry_msgs::Vector3 pwm_vector;

	pwm_vector.x=(255*omega)/omega_highlim;
	pwm_vector.y=(255*omega)/omega_lowlim;
	pwm_vector.z = 1;

	if(pwm_vector.x<-255)
		pwm_vector.x=-255;
	else if(pwm_vector.x>255)
		pwm_vector.x=255;

	if(pwm_vector.y<-255)
		pwm_vector.y=-255;
	else if(pwm_vector.y>255)
		pwm_vector.y=255;

	vector_pub.publish(pwm_vector);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "direction_follower_node");
	direction_follower obj;
	ros::spin();
	return 0;
}
