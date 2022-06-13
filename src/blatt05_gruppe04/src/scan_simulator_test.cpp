#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

#include <mcl_helper/scan_simulator.h>

using namespace mcl_helper;
ScanSimulator scan_sim;

ros::Publisher sim_pub;

void simCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose, const nav_msgs::OccupancyGrid::ConstPtr &map) {
    geometry_msgs::Pose pose_sim;
    pose_sim.position.x = pose->pose.pose.position.x;
    pose_sim.position.y = pose->pose.pose.position.y;
    pose_sim.position.z = pose->pose.pose.position.z;
    pose_sim.orientation.x = pose->pose.pose.orientation.x;
    pose_sim.orientation.y = pose->pose.pose.orientation.y;
    pose_sim.orientation.z = pose->pose.pose.orientation.z;
    pose_sim.orientation.w = pose->pose.pose.orientation.w;

    
    scan_sim.setMap(*map);
    sensor_msgs::LaserScan simulated_scan;
    simulated_scan.header.frame_id = "scanner_simulated";
    simulated_scan.angle_min = -2.3570001125335693;
    simulated_scan.angle_max = 2.3570001125335693;
    simulated_scan.angle_increment = 0.017459258437156677;
    simulated_scan.angle_min = 0.05000000074505806;
    simulated_scan.range_max = 10.0;
    scan_sim.simulateScan(pose_sim, simulated_scan);

    sim_pub.publish(simulated_scan);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scan_simulator_test");

    ros::NodeHandle nh;
    
    // subscriber
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "/initialpose", 1000);
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub(nh, "/map", 1000);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::OccupancyGrid> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pose_sub, map_sub);
    sync.registerCallback(boost::bind(&simCallback, _1, _2));

    sim_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_simulated", 1000);

    ros::spin();

    return 0;

}