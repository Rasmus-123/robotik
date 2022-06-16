#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

#include <mcl_helper/scan_simulator.h>

using namespace mcl_helper;
ScanSimulator scan_sim;

nav_msgs::OccupancyGrid latest_map;

geometry_msgs::PoseArray pose_array;
double particle_weights[];

ros::Publisher sim_pub;

// MISSING: pose_array being generated and motion update

void realScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    sensor_msgs::LaserScan simulated_scan;
    simulated_scan.header.stamp = scan->header.stamp;
    simulated_scan.header.frame_id = "scanner_simulated";
    simulated_scan.angle_min = scan->angle_min;
    simulated_scan.angle_max = scan->angle_max;
    simulated_scan.angle_increment = scan->angle_increment;
    simulated_scan.range_max = scan->range_max;
    
    weightParticles(scan);

}

void weightParticles(const sensor_msgs::LaserScan::ConstPtr &scan) {
    for(int i = 0; i < pose_array.poses.size(); i++) {
        sensor_msgs::LaserScan simulated_scan;
        scan_sim.simulateScan(pose, simulated_scan);

        double sum;
        for(int j = 0; j < simulated_scan.ranges.size(); j++) {
            sum += ((simulated_scan.ranges[j] - scan->ranges[j]) / 0.5)^2;
        }

        particle_weights[i] = sum / simulated_scan.ranges.size();
    }
}

enum MODE {
    WEIGHTED_AVERAGE,
    HIGHEST_WEIGHT,
};

geometry_msgs::Pose getPose(MODE mode) {
    if(mode == WEIGHTED_AVERAGE) {
        // normalize weights
        double sum = 0;
        for(int i = 0; i < pose_array.poses.size(); i++) {
            sum += particle_weights[i];
        }
        for(int i = 0; i < pose_array.poses.size(); i++) {
            particle_weights[i] /= sum;
        }
        
        // get weighted average
        double x,y,z,qx,qy,qz,qw;
        for(int i = 0; i < pose_array.poses.size(); i++) {
            x += pose_array.poses[i].position.x * particle_weights[i];
            y += pose_array.poses[i].position.y * particle_weights[i];
            z += pose_array.poses[i].position.z * particle_weights[i];
            qx += pose_array.poses[i].orientation.x * particle_weights[i];
            qy += pose_array.poses[i].orientation.y * particle_weights[i];
            qz += pose_array.poses[i].orientation.z * particle_weights[i];
            qw += pose_array.poses[i].orientation.w * particle_weights[i];
        }

        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        return pose;

    } else if(mode == HIGHEST_WEIGHT) {
        int index = 0;
        for(int i = 0; i < pose_array.poses.size(); i++) {
            if(particle_weights[i] > particle_weights[index]) {
                index = i;
            }
        }
        return pose_array.poses[index];
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "scan_simulator_test");

    ros::NodeHandle nh;
    
    // subscriber
    ros::Subscriber pose_sub = nh.subscribe("/initialpose", 1000, &poseCallback);
    ros::Subscriber map_sub = nh.subscribe("/map", 1000, &simCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 1000, &realScanCallback);

    sim_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_simulated", 1000);

    ros::spin();

    return 0;

}