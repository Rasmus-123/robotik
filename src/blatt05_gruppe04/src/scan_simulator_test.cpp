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

using namespace mcl_helper;
ScanSimulator scan_sim;

nav_msgs::OccupancyGrid latest_map;

ros::Publisher sim_pub;

/**
 * Warum tauchen Scan+Transform ab dem zweiten durchlauf auf?
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    
    static tf2_ros::TransformBroadcaster tf2_br;

    if (latest_map.header.frame_id.empty())
    {
        std::cout << "Waiting for map to be published!" << std::endl;
        return;
    }

    geometry_msgs::Pose pose_sim = pose->pose.pose;

    scan_sim.setMap(latest_map);

    sensor_msgs::LaserScan simulated_scan;
    simulated_scan.header.stamp = ros::Time::now();
    simulated_scan.header.frame_id = "scanner_simulated";
    simulated_scan.angle_min = -2.3570001125335693;
    simulated_scan.angle_max = 2.3570001125335693;
    simulated_scan.angle_increment = 0.017459258437156677;
    simulated_scan.angle_min = 0.05000000074505806;
    simulated_scan.range_max = 10.0;
    scan_sim.simulateScan(pose_sim, simulated_scan);

    // Publish Transformation

    // Pose ist relativ zu Map (oder so transformierbar). Scan ist relativ zu Pose.
    // Dann muss die Transformation einfach nur die Pose sein.

    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.child_frame_id = simulated_scan.header.frame_id; // Source Frame: "scanner_simulated"
    tf_stamped.header.frame_id = latest_map.header.frame_id; // Target Frame: "map"
    tf_stamped.header.stamp = ros::Time::now();

    tf_stamped.transform.rotation = pose_sim.orientation;
    tf_stamped.transform.translation.x = pose_sim.position.x;
    tf_stamped.transform.translation.y = pose_sim.position.y;
    tf_stamped.transform.translation.z = pose_sim.position.z;

    sim_pub.publish(simulated_scan);
    tf2_br.sendTransform(tf_stamped);

    std::cout << "Published Scan and Transform!" << std::endl;
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