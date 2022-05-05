#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void cloudCallback(const sensor_msgs::LaserScanConstPtr& scan)
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
    cloud.height = 1; // Was bedeutet die height?

    cloud.row_step = cloud.width * cloud.point_step;

    // cloud.data.resize(cloud.point_step * cloud.width);


    // TODO: Wie in 2.2b die Koordinaten berechnen


    //pub.publish(cloud);
}

int main()
{
    ros::NodeHandle nh;

    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    ros::Subscriber sub = nh.subscribe("scan", 1, cloudCallback);

    ros::spin();
}