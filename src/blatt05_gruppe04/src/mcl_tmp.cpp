#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <tf/transform_listener.h>

ros::Publisher pose_pub;
nav_msgs::Odometry last_odom;
geometry_msgs::PoseArray pose_array;

struct euler {
    double roll;
    double pitch;
    double yaw;
};

euler quaternion_to_euler(geometry_msgs::Quaternion q) {
    euler e;
    tf::Quaternion q_tf(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(q_tf);
    m.getRPY(e.roll, e.pitch, e.yaw);
    return e;
}


void ekfCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    if(!last_odom.header.frame_id.empty() && pose_array.poses.size() != 0) {
        double delta_t = (odom->header.stamp - last_odom.header.stamp).toSec();
        for(int i = 0; i < pose_array.poses.size(); i++) {
            euler eul = quaternion_to_euler(pose_array.poses[i].orientation);
            double dist_x = odom->twist.twist.linear.x * delta_t * std::cos(eul.yaw * 0.5);
            double dist_y = -odom->twist.twist.linear.x * delta_t * std::sin(eul.yaw * 0.5);

            pose_array.poses[i].position.x += dist_x;
            pose_array.poses[i].position.y += dist_y;

            double delta_rot = odom->twist.twist.angular.z * delta_t;
            eul.yaw += delta_rot;

            // roll pitch yaw to geometry_msgs::Quaternion
            tf::Quaternion q;
            q.setRPY(eul.roll, eul.pitch, eul.yaw);
            pose_array.poses[i].orientation.x = q.x();
            pose_array.poses[i].orientation.y = q.y();
            pose_array.poses[i].orientation.z = q.z();
            pose_array.poses[i].orientation.w = q.w();

        }

    }
    pose_pub.publish(pose_array);
    last_odom = *odom;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    ROS_INFO_STREAM("Pose: " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z);
pose_pub.publish(pose_array);
    if(pose_array.poses.size() == 0) {
        pose_array.header.stamp = pose->header.stamp;
        pose_array.header.frame_id = pose->header.frame_id;

        pose_array.poses.push_back(pose->pose.pose);

        // get random poses around the current pose with a sigma of 0.5
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> dist(0, 0.5);
        std::normal_distribution<double> rot(0, 0.1);

        for (int i = 0; i < 100; i++) {
            geometry_msgs::Pose pose_random;
            pose_random.position.x = pose->pose.pose.position.x + dist(gen);
            pose_random.position.y = pose->pose.pose.position.y + dist(gen);

            tf::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            yaw += rot(gen);

            q.setRPY(roll, pitch, yaw);
            pose_random.orientation.x = q.x();
            pose_random.orientation.y = q.y();
            pose_random.orientation.z = q.z();
            pose_random.orientation.w = q.w();

            pose_array.poses.push_back(pose_random);
        }
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mcl2");

    ros::NodeHandle nh;
    
    // publisher
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("/mcl/poses", 1000);

    // subscriber
    ros::Subscriber sub = nh.subscribe("/initialpose", 1000, poseCallback);
    ros::Subscriber ekf_sub = nh.subscribe("/odom", 1000, ekfCallback);
    ros::spin();

    return 0;

}