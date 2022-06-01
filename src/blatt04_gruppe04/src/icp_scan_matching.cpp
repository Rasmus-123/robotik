# include <ros/ros.h>

// Subscribe
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/LaserScan.h>

// Publish

#include <visualization_msgs/Marker.h>

// Other

// --- --- --- //

ros::Publisher pub_marker;


void scanCallback(const sensor_msgs::LaserScanConstPtr& model, const sensor_msgs::LaserScan& scan)
{
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "icp_scan_matching_node");

    ros::NodeHandle nh;

    // Publisher

    pub_marker = nh.advertise<visualization_msgs::Marker>("/icp/marker", 1000);

    // Subscriber

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_model(nh, "/model", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan(nh, "/scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_model, sub_scan);
    sync.registerCallback(boost::bind(&scanCallback, _1, _2));
}