#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <visualization_msgs/Marker.h>


#include <cmath>

ros::Publisher pub;

float epsilon;

struct Point
{
    float x;
    float y;
    float z;
};


inline float euklidDistance(const Point& a, const Point& b)
{
    return std::sqrt(std::exp2f(a.x - b.x) + std::exp2f(a.y - b.y) + std::exp2f(a.z - b.z));
}


void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    visualization_msgs::Marker marker;

    marker.header = cloud->header;
    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.color.a = 1; // Ansonsten unsichtbar
    marker.color.g = 1;

    std::vector<Point> points;
    points.reserve(cloud->data.size() / sizeof(Point));

    for (int i = 0; i < cloud->data.size() / sizeof(Point); i++)
    {
        Point* ptr = (Point*) cloud->data[0];
        points.push_back(ptr[i]);
    }

    for (int i = 0; i < points.size(); i++)
    {
        const int line_begin = i;

        
    }

    /*
    std::list<std::list<Point>> lines;
    for (int i = 0; i < cloud->data.size() / sizeof(Point); i++)
    {
        if (lines.empty())
        {
            lines.push_back({p});
            continue;
        }

        std::list<Point>& line = lines.back();

        if(line.size() == 1)
        {
            line.push_back(p);
            continue;
        }
        
        euklidDistance(line.back(), p);

        float sum = 0;

        for (int i = )
    }
    */
    ros::shutdown();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "online_line_finder");

	ROS_INFO("Hallo");

	ros::NodeHandle nh_p("~");

	pub = nh_p.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    nh_p.param<float>("epsilon", epsilon, 1.0);

	ros::Subscriber sub = nh_p.subscribe("/convert_scan_to_cloud/cloud", 1, cloudCallback);

	ros::spin();
}