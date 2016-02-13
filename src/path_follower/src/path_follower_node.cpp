#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>

#define pi 3.141592653589793

const int array_size=1000;	// Size of the arrays used to store way points i.e way and snap points i.e snap
int way_start=0;	// Starting index of unexecuted way point
int way_end=6;	// Ending index of way points in buffer
double way[array_size][2] = {{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}};	// way point array
int snap_start=0;	// Index of snap point corresponding to the way_start
int snap_end=6;	// Index of snap point corresponding to the way_end
double snap[array_size][2];	// snap point array
int path_start=0;	// Index of path point corresponding to the way_start
int path_end=10;	// Index of path point corresponding to the way_end
const int path_size=2*array_size-1;	// Relation between no. of way points and path points
double path[path_size][2];	// path point array

/*
double angle_sweep = pi/4;
double radius_sweep = 0.5;
int index_sweep = (int)((725/4.448544)*angle_sweep);
*/

//class laserscan_to_pointcloud
class path_follower
{
	private:
		ros::NodeHandle n;
		ros::Subscriber waypoint_sub;
		ros::Publisher vector_pub, marker_pub;

	public:
		path_follower();
		double distance(double x1, double y1, double x2, double y2);
		void scan_callback(const geometry_msgs::Vector3::ConstPtr& scan_in);
};

path_follower::path_follower()
{
	vector_pub = n.advertise<geometry_msgs::Vector3>("clearance_vector_bfcs", 1000);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
	waypoint_sub = n.subscribe("/waypoint_add", 1000, &path_follower::scan_callback, this);

	for(int i=0; i<(array_size+way_end-way_start)%array_size; i++)
	{
		if(i==0)
		{
			snap[(snap_start+i)%array_size][0]=way[(way_start+i)%array_size][0];
			snap[(snap_start+i)%array_size][1]=way[(way_start+i)%array_size][1];
			snap_end=(snap_start+i+1)%array_size;
		}
		else
		{
			snap[(snap_start+i)%array_size][0]=2*way[(way_start+i)%array_size][0]-snap[(way_start+i-1)%array_size][0];
			snap[(snap_start+i)%array_size][1]=2*way[(way_start+i)%array_size][1]-snap[(way_start+i-1)%array_size][1];
			snap_end=(snap_start+i+1)%array_size;
		}
	}

	for(int i=0; i<(array_size+snap_end-snap_start)%array_size; i++)
	{
		if(i==0)
		{
			path[(path_start+i)%path_size][0]=way[(way_start+i)%array_size][0];
			path[(path_start+i)%path_size][1]=way[(way_start+i)%array_size][1];
		}
		else if(i==(array_size+snap_end-snap_start)%array_size-1)
		{
			path[(path_start+2*i-1)%path_size][0]=way[(way_start+i)%array_size][0];
			path[(path_start+2*i-1)%path_size][1]=way[(way_start+i)%array_size][1];
		}
		else
		{
			if(distance(way[(way_start+i)%array_size][0], way[(way_start+i)%array_size][1], snap[(snap_start+i)%array_size][0], snap[(snap_start+i)%array_size][1])<distance(way[(way_start+i+1)%array_size][0], way[(way_start+i+1)%array_size][1], snap[(snap_start+i)%array_size][0], snap[(snap_start+i)%array_size][1]))
			{
				path[(2*i-1)%path_size][0]=way[(way_start+i)%array_size][0];
				path[(2*i-1)%path_size][1]=way[(way_start+i)%array_size][1];
				double dy=snap[(snap_start+i+1)%array_size][1]-snap[(snap_start+i)%array_size][1];
				double dx=snap[(snap_start+i+1)%array_size][0]-snap[(snap_start+i)%array_size][0];
				double dl=sqrt(dy*dy+dx*dx);
				double s=dy/dl;
				double c=dx/dl;
				double t=dy/dx;
				double d=distance(way[(way_start+i)%array_size][0], way[(way_start+i)%array_size][1], snap[(snap_start+i)%array_size][0], snap[(snap_start+i)%array_size][1]);
				path[(path_start+2*i)%path_size][0]=snap[(snap_start+i)%array_size][0]+d*c;
				path[(path_start+2*i)%path_size][1]=snap[(snap_start+i)%array_size][1]+d*s;
			}
			else
			{
				path[(path_start+2*i)%path_size][0]=way[(way_start+i+1)%array_size][0];
				path[(path_start+2*i)%path_size][1]=way[(way_start+i+1)%array_size][1];
				double dy=snap[(snap_start+i-1)%array_size][1]-snap[(snap_start+i)%array_size][1];
				double dx=snap[(snap_start+i-1)%array_size][0]-snap[(snap_start+i)%array_size][0];
				double dl=sqrt(dy*dy+dx*dx);
				double s=dy/dl;
				double c=dx/dl;
				double t=dy/dx;
				double d=distance(way[(way_start+i+1)%array_size][0], way[(way_start+i+1)%array_size][1], snap[(snap_start+i)%array_size][0], snap[(snap_start+i)%array_size][1]);
				path[(path_start+2*i-1)%path_size][0]=snap[(snap_start+i)%array_size][0]+d*c;
				path[(path_start+2*i-1)%path_size][1]=snap[(snap_start+i)%array_size][1]+d*s;
			}
		}
	}

	for(int i=0; i<2*(array_size-1); i++)
		ROS_ERROR("%d : p=( %f, %f)", i, path[i][0], path[i][1]);
}

double path_follower::distance(double x1, double y1, double x2, double y2)
{
	return(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)));
}

void path_follower::scan_callback(const geometry_msgs::Vector3::ConstPtr& scan_in)
{
	way[way_end][0]=scan_in->x;
	way[way_end][1]=scan_in->y;
	way_end=(way_end+1)%array_size;

	geometry_msgs::Vector3 clearance_vector;
	clearance_vector.x=0.0;
	clearance_vector.y=0.0;
	clearance_vector.z=0.0;

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

	double e_a, e_b, e_c;
	e_a = 0.0;
	e_b=0.0;
	e_c=0.0;
	double q_x, q_y, q_z, q_w;
	q_x = cos(e_a/2)*cos(e_b/2)*cos(e_c/2) + sin(e_a/2)*sin(e_b/2)*sin(e_c/2);
	q_y = sin(e_a/2)*cos(e_b/2)*cos(e_c/2) - cos(e_a/2)*sin(e_b/2)*sin(e_c/2);
	q_z = cos(e_a/2)*sin(e_b/2)*cos(e_c/2) + sin(e_a/2)*cos(e_b/2)*sin(e_c/2);
	q_w = cos(e_a/2)*cos(e_b/2)*sin(e_c/2) - sin(e_a/2)*sin(e_b/2)*cos(e_c/2);

	marker.pose.orientation.x = q_x;
	marker.pose.orientation.y = q_y;
	marker.pose.orientation.z = q_z;
	marker.pose.orientation.w = q_w;

	marker.scale.x = sqrt(clearance_vector.x*clearance_vector.x+clearance_vector.y*clearance_vector.y+clearance_vector.z*clearance_vector.z);
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "path_follower_node");
	path_follower obj;
	ros::spin();
	return 0;
}
