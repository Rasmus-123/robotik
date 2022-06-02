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

// --- --- --- //

ros::Publisher pub_marker;

ros::Publisher pub_cloud1;
ros::Publisher pub_cloud2;

laser_geometry::LaserProjection laser_projector;

sensor_msgs::PointCloud2 test(const std::vector<geometry_msgs::Point32>& points) 
{
    sensor_msgs::PointCloud2 payload;

    // describe the bytes
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

    payload.fields.push_back(field_x);
    payload.fields.push_back(field_y);
    payload.fields.push_back(field_z);

    payload.point_step = 3 * sizeof(float);

    payload.width = points.size();
    payload.height = 1;
    payload.row_step = payload.width * payload.point_step;
    payload.data.resize(payload.row_step * payload.height);

    // reinterpret byte memory as float memory
    float* data_raw = reinterpret_cast<float*>(&payload.data[0]);

    for(int i = 0; i < points.size(); i++) 
    {
        data_raw[i*3] = points[i].x;
        data_raw[i*3 + 1] = points[i].y;
        data_raw[i*3 + 2] = points[i].z;
    }

    return payload;
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

    

    visualization_msgs::Marker mark;

    mark.header = model->header;
    mark.id = 0;

    mark.type = visualization_msgs::Marker::LINE_LIST;
    mark.action = visualization_msgs::Marker::ADD;

    mark.scale.x = 1.0;
    mark.scale.y = 1.0;
    mark.scale.z = 1.0;
    mark.scale.z = 1.0;

    mark.color.a = 1.0;
    mark.color.r = 1.0;
    mark.color.g = 0.0;
    mark.color.b = 0.0;

    // for irgendwas
    // {
    //      mark.points.push_back() Linie-Start
    //      mark.points.push_back() Linie-Ende
    // }

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