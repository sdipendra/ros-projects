//1.1
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#define pi 3.141592653589793
#define width 0.3
#define length 0.6

float angle_sweep = pi/2;
float radius_sweep = 0.5;
int index_sweep = (int)((725/4.448544)*angle_sweep);

//class laserscan_to_pointcloud
class laserscan_to_clearance_vector
{
	private:
		ros::NodeHandle n;

		ros::Subscriber laser_sub;

		ros::Publisher vector_pub, marker_pub;


	public:
		laserscan_to_clearance_vector();
		void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
		float index_to_angle(int index);
		float dimension_fit(const sensor_msgs::LaserScan::ConstPtr& scan, int index);
};

laserscan_to_clearance_vector::laserscan_to_clearance_vector()
{
	vector_pub = n.advertise<geometry_msgs::Vector3>("vector_pwm_arduino", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

	laser_sub = n.subscribe("/scan", 1000, &laserscan_to_clearance_vector::scan_callback, this);
}

void laserscan_to_clearance_vector::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	int head_index=-1;
	float min_distance=1.0, distance=0.0;
/*
	for(int i=44; i<=725; i++)
	{
		if(scan_in->ranges[i]>3.0)
		{
			head_index=i;
			break;
		}
		if(scan_in->ranges[i]>scan_in->ranges[head_index])
			head_index=i;
	}
*/
	for(int i=0; i<341; i++)
	{
		if((scan_in->ranges[384-i]>3.0)||(scan_in->ranges[384-i]!=scan_in->ranges[384-i]))
		{
			distance=this->dimension_fit(scan_in, 384-i+(index_sweep/2));
			if(distance!=-1.0)
			{
				head_index=384-i;
				min_distance=distance;
				break;
			}
		}
		if((scan_in->ranges[384+i+1]>3.0)||(scan_in->ranges[384+i+1]!=scan_in->ranges[384+i+1]))
		{
			distance=this->dimension_fit(scan_in, 384+i+1+(index_sweep/2));
			if(distance!=-1.0)
			{
				head_index=384+i+1;
				min_distance=distance;
				break;
			}
		}
		if(head_index==-1||scan_in->ranges[384-i]>scan_in->ranges[head_index])
		{
			distance=this->dimension_fit(scan_in, 384-i+(index_sweep/2));
			if(distance!=-1.0)
			{
				head_index=384-i;
				min_distance=distance;
			}
		}
		else if(head_index==-1||scan_in->ranges[384+i+1]>scan_in->ranges[head_index])
		{
			distance=this->dimension_fit(scan_in, 384+i+1+(index_sweep/2));
			if(distance!=-1.0)
			{
				head_index=384+i+1;
				min_distance=distance;
			}
		}
	}
	float head_angle = this->index_to_angle(head_index);
	ROS_ERROR("%lf %d", head_angle, head_index);
	float head_radius;
	if(min_distance>3.0)
		head_radius=1.0;
	else
	{
		head_radius=min_distance/3.0;
	}
	geometry_msgs::Vector3 clearance_vector;
	if(head_index==-1)
	{
		clearance_vector.x=pi;
		clearance_vector.y=1.0;
		clearance_vector.z=0.0;
	}
	else
	{	
		clearance_vector.x=head_angle;
		clearance_vector.y=1.0;
		clearance_vector.z=0.0;
	}

	uint32_t marker_shape = visualization_msgs::Marker::ARROW;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/laser";
	marker.header.stamp = ros::Time::now();
	marker.ns = "clearance_vector";
	marker.id = 0;
	marker.type = marker_shape;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;

	float e_a, e_b, e_c;
	e_a = clearance_vector.x;
	e_b=0.0;
	e_c=0.0;
	float q_x, q_y, q_z, q_w;
	q_x = cos(e_a/2)*cos(e_b/2)*cos(e_c/2) + sin(e_a/2)*sin(e_b/2)*sin(e_c/2);
	q_y = sin(e_a/2)*cos(e_b/2)*cos(e_c/2) - cos(e_a/2)*sin(e_b/2)*sin(e_c/2);
	q_z = cos(e_a/2)*sin(e_b/2)*cos(e_c/2) + sin(e_a/2)*cos(e_b/2)*sin(e_c/2);
	q_w = cos(e_a/2)*cos(e_b/2)*sin(e_c/2) - sin(e_a/2)*sin(e_b/2)*cos(e_c/2);

	marker.pose.orientation.x = q_x;
	marker.pose.orientation.y = q_y;
	marker.pose.orientation.z = q_z;
	marker.pose.orientation.w = q_w;

	marker.scale.x = min_distance;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();

	vector_pub.publish(clearance_vector);
	marker_pub.publish(marker);
}

float laserscan_to_clearance_vector::index_to_angle(int index)
{
	float x, x1, x2, y, y1, y2;
	x = index;
	x1 = 0;
	x2 = 725;
	y1 = -2.356194;
	y2 = 2.092350;
	y = y1 + ( ( (y2 - y1) / (x2 - x1) ) * (x - x1) );
	return y;
}

float laserscan_to_clearance_vector::dimension_fit(const sensor_msgs::LaserScan::ConstPtr& scan, int index)
{
	float min=scan->ranges[index-(index_sweep/2)];
/*
	for(int i=0; i<index_sweep; i++)
	{
		if(scan->ranges[index-(index_sweep/2)+i]<0.5)
			return -1.0;
		if(scan->ranges[index-(index_sweep/2)+i]<min)
			min=scan->ranges[index-(index_sweep/2)+i];
	}
*/
	for(int i=0; i<=index_sweep; i++)
	{
		if((scan->ranges[index-(index_sweep/2)+i]==scan->ranges[index-(index_sweep/2)+i])&&(fabs(scan->ranges[index-(index_sweep/2)+i]*sin(index_to_angle(index-(index_sweep/2)+i)-index_to_angle(index)))<width)&&(scan->ranges[index-(index_sweep/2)+i]*cos(index_to_angle(index-(index_sweep/2)+i)-index_to_angle(index))>0.1)&&(scan->ranges[index-(index_sweep/2)+i]*cos(index_to_angle(index-(index_sweep/2)+i)-index_to_angle(index))<length))
			return -1.0;
		if((scan->ranges[index-(index_sweep/2)+i]==scan->ranges[index-(index_sweep/2)+i])&&(scan->ranges[index-(index_sweep/2)+i]<min))
			min=scan->ranges[index-(index_sweep/2)+i];
	}
	if(min!=min)
		return 3.0;
	else
		return min;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laserscan_to_clearance_vector_node");
	laserscan_to_clearance_vector obj;
	ros::spin();
	return 0;
}
