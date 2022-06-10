#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <random>
#include <tf/transform_listener.h>

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped last_pose;
geometry_msgs::PoseArray pose_array;


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    ROS_INFO_STREAM("Pose: " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z);

    pose_array.header.stamp = pose->header.stamp;
    pose_array.header.frame_id = pose->header.frame_id;

    pose_array.poses.push_back(pose->pose.pose);

    // get random poses around the current pose with a sigma of 0.5m
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0, 0.5);

    for (int i = 0; i < 100; i++) {
        geometry_msgs::Pose pose_random;
        pose_random.position.x = pose->pose.pose.position.x + dist(gen);
        pose_random.position.y = pose->pose.pose.position.y + dist(gen);
        pose_random.orientation = pose->pose.pose.orientation;
        pose_array.poses.push_back(pose_random);
    }

    pose_pub.publish(pose_array);

    ekfCallback(pose);
}

void ekfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {

    // current pose quaternion to roll pitch yaw
    tf::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // last pose quaternion to roll pitch yaw
    tf::Quaternion q2(last_pose.pose.pose.orientation.x, last_pose.pose.pose.orientation.y, last_pose.pose.pose.orientation.z, last_pose.pose.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getRPY(roll2, pitch2, yaw2);

    // calculate difference in roll pitch yaw
    double diff_roll = roll - roll2;
    double diff_pitch = pitch - pitch2;
    double diff_yaw = yaw - yaw2;


    for(int i = 0; i < pose_array.poses.size(); i++) {
        pose_array.poses[i].position.x += pose->pose.pose.position.x - last_pose.pose.pose.position.x;
        pose_array.poses[i].position.y += pose->pose.pose.position.y - last_pose.pose.pose.position.y;
        

        // pose from pose_array to quaternion
        tf::Quaternion q3(pose_array.poses[i].orientation.x, pose_array.poses[i].orientation.y, pose_array.poses[i].orientation.z, pose_array.poses[i].orientation.w);
        tf::Matrix3x3 m3(q3);
        double roll3, pitch3, yaw3;
        m3.getRPY(roll3, pitch3, yaw3);

        // calculate new roll pitch yaw
        double new_roll = roll3 + diff_roll;
        double new_pitch = pitch3 + diff_pitch;
        double new_yaw = yaw3 + diff_yaw;

        // roll pitch yaw to quaternion
        tf::Quaternion q4;
        q4.setRPY(new_roll, new_pitch, new_yaw);

        // tf::Quaternion to geometry_msgs::Quaternion for pose from pose_array
        pose_array.poses[i].orientation.x = q4.x();
        pose_array.poses[i].orientation.y = q4.y();
        pose_array.poses[i].orientation.z = q4.z();
        pose_array.poses[i].orientation.w = q4.w();
    }

    pose_pub.publish(pose_array);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mcl2");

    ros::NodeHandle nh;
    
    // publisher
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("/mcl/poses", 1000);

    // subscriber
    ros::Subscriber sub = nh.subscribe("/initialpose", 1000, poseCallback);
    ros::Subscriber ekf_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1000, ekfCallback);
    ros::spin();

    return 0;

}