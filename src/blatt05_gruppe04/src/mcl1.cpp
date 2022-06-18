#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <random>

ros::Publisher pose_pub;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    ROS_INFO_STREAM("Pose: " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z);

    geometry_msgs::PoseArray pose_array;

    pose_array.header.stamp = pose->header.stamp;
    pose_array.header.frame_id = pose->header.frame_id;

    pose_array.poses.push_back(pose->pose.pose);

    // get random poses around the current pose with a sigma of 0.5
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0, 0.75);
    std::normal_distribution<double> rot(0, 0.2);

    for (int i = 0; i < 500; i++) {
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

    

    pose_pub.publish(pose_array);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mcl1");

    ros::NodeHandle nh;
    
    // publisher
    pose_pub = nh.advertise<geometry_msgs::PoseArray>("/mcl/poses", 1000);

    // subscriber
    ros::Subscriber sub = nh.subscribe("/initialpose", 1000, poseCallback);
    ros::spin();

    return 0;

}