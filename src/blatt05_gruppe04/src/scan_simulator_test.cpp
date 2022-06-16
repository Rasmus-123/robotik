#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

#include <mcl_helper/scan_simulator.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace mcl_helper;
ScanSimulator scan_sim;

nav_msgs::OccupancyGrid latest_map;

ros::Publisher sim_pub;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    
    static tf2_ros::TransformBroadcaster tf2_br;

    geometry_msgs::Pose pose_sim;
    pose_sim.position.x = pose->pose.pose.position.x;
    pose_sim.position.y = pose->pose.pose.position.y;
    pose_sim.position.z = pose->pose.pose.position.z;
    pose_sim.orientation.x = pose->pose.pose.orientation.x;
    pose_sim.orientation.y = pose->pose.pose.orientation.y;
    pose_sim.orientation.z = pose->pose.pose.orientation.z;
    pose_sim.orientation.w = pose->pose.pose.orientation.w;
    ROS_INFO("1");
    
    scan_sim.setMap(latest_map);
    ROS_INFO("2");
    sensor_msgs::LaserScan simulated_scan;
    simulated_scan.header.stamp = ros::Time::now();
    simulated_scan.header.frame_id = "scanner_simulated";
    simulated_scan.angle_min = -2.3570001125335693;
    simulated_scan.angle_max = 2.3570001125335693;
    simulated_scan.angle_increment = 0.017459258437156677;
    simulated_scan.angle_min = 0.05000000074505806;
    simulated_scan.range_max = 10.0;
    scan_sim.simulateScan(pose_sim, simulated_scan);

    ROS_INFO("3");

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.child_frame_id = simulated_scan.header.frame_id; // Source Frame
    tf_stamped.header.frame_id = "map"; // Target Frame
    tf_stamped.header.stamp = ros::Time::now();

    tf_stamped.transform.rotation = pose_sim.orientation;
    tf_stamped.transform.translation.x = pose_sim.position.x;
    tf_stamped.transform.translation.y = pose_sim.position.y;
    tf_stamped.transform.translation.z = pose_sim.position.z;

    sim_pub.publish(simulated_scan);
    tf2_br.sendTransform(tf_stamped);
}

void simCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    latest_map = *map;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scan_simulator_test");

    ros::NodeHandle nh;
    
    // subscriber
    ros::Subscriber pose_sub = nh.subscribe("/initialpose", 1000, &poseCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1000, &simCallback);

    sim_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_simulated", 1000);

    ros::spin();

    return 0;

}