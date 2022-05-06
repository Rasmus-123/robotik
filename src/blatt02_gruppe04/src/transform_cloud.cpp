#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


ros::Publisher pub;

std::string target_frame;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer.lookupTransform(target_frame, "base_link", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    sensor_msgs::PointCloud2 res;

    tf2::doTransform(*cloud, res, transformStamped);

    pub.publish(res);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "transform_cloud");

	ROS_INFO("Hallo");

	ros::NodeHandle nh_p("~");

	pub = nh_p.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    nh_p.param<std::string>("target_frame", target_frame, "base_link");

    ROS_INFO_STREAM("Target_Frame: " << target_frame);

	ros::Subscriber sub = nh_p.subscribe("/convert_scan_to_cloud/cloud", 1, cloudCallback);

	ros::spin();
}