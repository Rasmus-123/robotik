#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <random>
#include <tf/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

ros::Publisher pose_pub;
geometry_msgs::PoseWithCovarianceStamped last_pose;
geometry_msgs::PoseArray pose_array;

struct Pose2D {
    float x;
    float y;
    float alpha;
};

float get_yaw(const geometry_msgs::Quaternion& q)
{
    tf2::Quaternion rot;
    tf2::convert(q, rot);
    tf2::Matrix3x3 m(rot);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void convert(const geometry_msgs::Pose& from, Pose2D& to)
{
    to.x = from.position.x;
    to.y = from.position.y;
    to.alpha = get_yaw(from.orientation);
}

void convert(   const geometry_msgs::Pose& from,
                Eigen::Affine3d& to)
{
    Eigen::Vector3d translation;
    translation.x() = from.position.x;
    translation.y() = from.position.y;
    translation.z() = from.position.z;
    Eigen::Quaterniond rotation;
    rotation.x() = from.orientation.x;
    rotation.y() = from.orientation.y;
    rotation.z() = from.orientation.z;
    rotation.w() = from.orientation.w;
    to.setIdentity();
    to.linear() = rotation.matrix();
    to.translation() = translation;
}

geometry_msgs::Pose delta_pose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    Eigen::Affine3d T_p1_map;
    convert(p1, T_p1_map);

    Eigen::Affine3d T_p2_map;
    convert(p2, T_p2_map);

    // we need T_p2_p1
    
    // T_p2_p1 = T_p2->p1 
    Eigen::Affine3d T_p2_p1 = T_p1_map.inverse() * T_p2_map;

    Eigen::Vector3d t = T_p2_p1.translation();
    Eigen::Quaterniond rot;
    rot = T_p2_p1.linear();

    geometry_msgs::Pose delta_p;
    delta_p.position.x = t.x();
    delta_p.position.y = t.y();
    delta_p.position.z = t.z();

    delta_p.orientation.x = rot.x();
    delta_p.orientation.y = rot.y();
    delta_p.orientation.z = rot.z();
    delta_p.orientation.w = rot.w();

    return delta_p;
}

Pose2D apply_delta(
    const Pose2D& pose, 
    const Pose2D& delta)
{
    Pose2D pose_new;

    pose_new.x = delta.x * cos(pose.alpha) + delta.y * -sin(pose.alpha) + pose.x;
    pose_new.y = delta.x * sin(pose.alpha) + delta.y * cos(pose.alpha) + pose.y;
    pose_new.alpha = pose.alpha + delta.alpha;

    return pose_new;
}

void ekfCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {

    if(!last_pose.header.frame_id.empty()) {

            Pose2D current_ekf_2d;
            convert(pose->pose.pose, current_ekf_2d);
            
            geometry_msgs::Pose current_ekf = pose->pose.pose;
            geometry_msgs::Pose delta_ekf = delta_pose(last_pose.pose.pose, current_ekf);

            Pose2D delta_ekf_2d;
            convert(delta_ekf, delta_ekf_2d);
            
            for(size_t i=0; i < pose_array.poses.size(); i++)
            {
                Pose2D old_pose;
                old_pose.x = pose_array.poses[i].position.x;
                old_pose.y = pose_array.poses[i].position.y;
                old_pose.alpha = get_yaw(pose_array.poses[i].orientation);
                Pose2D new_pose = apply_delta(old_pose, delta_ekf_2d);
                pose_array.poses[i].position.x = new_pose.x;
                pose_array.poses[i].position.y = new_pose.y;
                pose_array.poses[i].orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), new_pose.alpha));
            }
    }

    pose_array.header.stamp = pose->header.stamp;

    pose_pub.publish(pose_array);

    last_pose = *pose;
}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    ROS_INFO_STREAM("Pose: " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z);

    pose_array = geometry_msgs::PoseArray();

    pose_array.header.stamp = pose->header.stamp;
    pose_array.header.frame_id = pose->header.frame_id;

    pose_array.poses.push_back(pose->pose.pose);

    // get random poses around the current pose with a sigma of 0.5
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0, 1.3);
    std::normal_distribution<double> rot(0, 0.2);

    for (int i = 0; i < 2000; i++) {
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