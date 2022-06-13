#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

ros::Publisher sim_pub;

void simCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose, const nav_msgs::OccupancyGrid::ConstPtr &map) {
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scan_simulator_test");

    ros::NodeHandle nh;

    double x = cos(10);
    
    // subscriber
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "/initialpose", 1000);
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(nh, "/map", 1000);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::OccupancyGrid> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_sub, map_sub);
    sync.registerCallback(boost::bind(&simCallback, _1, _2));

    sim_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_simulated", 1000);

    return 0;

}