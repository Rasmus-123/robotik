#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>

ros::Publisher pub;

void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
	sensor_msgs::PointCloud2 cloud;
	cloud.header.frame_id = "laser";
	cloud.header.stamp = scan->header.stamp;
	cloud.header.seq = scan->header.seq; //?


	sensor_msgs::PointField field_x;
	field_x.name = "x";
	field_x.offset = 0 * sizeof(float);
	field_x.datatype = sensor_msgs::PointField::FLOAT32;
	field_x.count = 1;

	sensor_msgs::PointField field_y;
	field_y.name = "y";
	field_y.offset = 1 * sizeof(float);
	field_y.datatype = sensor_msgs::PointField::FLOAT32;
	field_y.count = 1;

	sensor_msgs::PointField field_z;
	field_z.name = "z";
	field_z.offset = 2 * sizeof(float);
	field_z.datatype = sensor_msgs::PointField::FLOAT32;
	field_z.count = 1;

	cloud.fields.push_back(field_x);
	cloud.fields.push_back(field_y);
	cloud.fields.push_back(field_z);


	cloud.point_step = 3 * sizeof(float);

	cloud.width = scan->ranges.size();
	cloud.height = 1;

	cloud.row_step = cloud.width * cloud.point_step;

	cloud.data.resize(cloud.row_step * cloud.height);

	for (int i = 0; i < scan->ranges.size(); i++)
	{
		float range = scan->ranges.at(i);

		if (range > scan->range_max || range < scan->range_min)
		{
			continue;
		}

		float angle = scan->angle_min + (i * scan->angle_increment);

		float x = range * std::cos(angle);
		float y = range * std::sin(angle);
		float z = 0;

		int offset = i * 3;

		float* ptr = (float*) &cloud.data[0];

		ptr[offset] = x;
		ptr[offset+1] = y;
		ptr[offset+2] = z;
	}
	
	pub.publish(cloud);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "convert_scan_to_cloud");

	ROS_INFO("Hallo");

	ros::NodeHandle nh_p("~");

	pub = nh_p.advertise<sensor_msgs::PointCloud2>("cloud", 1);

	ros::Subscriber sub = nh_p.subscribe("scan", 1, scanCallback);

	ros::spin();
}