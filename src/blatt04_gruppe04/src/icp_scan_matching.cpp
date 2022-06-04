# include <ros/ros.h>

// Subscribe
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>

// Publish

#include <visualization_msgs/Marker.h>

// Other

#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <utility>

// --- --- --- //

ros::Publisher pub_marker;

ros::Publisher pub_cloud1;
ros::Publisher pub_cloud2;

laser_geometry::LaserProjection laser_projector;

inline double PointDistance(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
}

std::vector<geometry_msgs::Point32> CloudToListOfPoints(const sensor_msgs::PointCloud2& cloud)
{
    // Je 4 Pointfields mit einem Element:
    //  x, y, z, index
    //  Offsets: 0,4,8,12
    //  Also je 32 Bits

    // Model-Data Len: 2736
    // Height: 1 (daher die ganzen mal height eigentlich unnötig)
    // Width: 171

    // Diese Funktion ist unschön * 1000

    std::vector<geometry_msgs::Point32> points;
    points.reserve(cloud.height * cloud.width); // Hier: 1 * 171


    const int offset = cloud.fields.size(); // Hier: 4
    for (int i = 0; i < cloud.width * cloud.height; i++)
    {   
        // x, y, z = x, y, z, index -> index just gets ignored
        geometry_msgs::Point32* p = (geometry_msgs::Point32*) cloud.data.data() + i * offset;

        points.push_back(*p);
    }

    return points;
}

void scanCallback(const sensor_msgs::LaserScanConstPtr& model, const sensor_msgs::LaserScanConstPtr& scan)
{
    // https://wiki.ros.org/laser_geometry
    // Simple Projection (for now?)
    // Ist in rviz identisch zum Laser-Scan, scheint also so zu reichen
    sensor_msgs::PointCloud2 cloud_model;
    sensor_msgs::PointCloud2 cloud_scan;
    laser_projector.projectLaser(*model, cloud_model);
    laser_projector.projectLaser(*scan, cloud_scan);

    std::vector<geometry_msgs::Point32> points_model = CloudToListOfPoints(cloud_model);
    std::vector<geometry_msgs::Point32> points_scan = CloudToListOfPoints(cloud_scan);

    std::vector<std::pair<geometry_msgs::Point32, geometry_msgs::Point32>> lines;

    double max_distance = 2.0; // Epsilon?
    for (const auto &p : points_scan)
    {
        double current_dist = max_distance;
        geometry_msgs::Point32 current_point;
        bool isset = false;

        for (const auto& model_p : points_model)
        {
            if (PointDistance(p, model_p) <= current_dist)
            {
                current_dist = PointDistance(p, model_p);
                current_point = model_p;
                isset = true;
            }
        }
        
        if (isset)
        {
            lines.push_back(std::make_pair(p, current_point));
        }
    }

    std::cout << lines.size() << " Pairs found!" << std::endl;
    

    visualization_msgs::Marker mark;

    mark.header = model->header;
    mark.id = 1;
    mark.type = visualization_msgs::Marker::LINE_LIST;
    mark.action = visualization_msgs::Marker::ADD;
    mark.scale.x = 0.01;
    mark.color.a = 1.0;
    mark.color.r = 1.0;
    mark.pose.orientation.w = 1.0;

    for (const auto& pair : lines)
    {
        // Kann man Point32 nicht einfacher in Point (64) umwandeln?
        geometry_msgs::Point p1;
        p1.x = pair.first.x;
        p1.y = pair.first.y;
        p1.z = pair.first.z;
        geometry_msgs::Point p2;
        p2.x = pair.second.x;
        p2.y = pair.second.y;
        p2.z = pair.second.z;

        mark.points.push_back(p1); //Linie-Start
        mark.points.push_back(p2); //Linie-Ende
    }

    pub_marker.publish(mark);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_scan_matching_node");

    ros::NodeHandle nh;

    // Publisher

    pub_marker = nh.advertise<visualization_msgs::Marker>("/icp/marker", 1000);

    pub_cloud1 = nh.advertise<sensor_msgs::PointCloud2>("/icp/cloud_model", 1000);
    pub_cloud2 = nh.advertise<sensor_msgs::PointCloud2>("/icp/cloud_scan", 1000);

    // Subscriber

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_model(nh, "/model", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan(nh, "/scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_model, sub_scan);
    sync.registerCallback(boost::bind(&scanCallback, _1, _2));

    ros::spin();

    return 0;
}