#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TransformStamped.h>

ros::Publisher pub;


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_cloud");

	ROS_INFO("Hallo");

	ros::NodeHandle nh_p("~");

	pub = nh_p.advertise<geometry_msgs::TransformStamped>("cloud", 1);

	ros::Subscriber sub = nh_p.subscribe("cloud", 1, cloudCallback);

	ros::spin();
}