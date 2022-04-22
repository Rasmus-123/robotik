#include "nav_msgs/Odometry.h"
#include "ceres_msgs/WheelPositions.h"
#include "ros/ros.h"
#include <math.h>


#include <stdio.h>
#include <stdlib.h>


ros::Publisher wheel_pub;
ceres_msgs::WheelPositions::Ptr last_msg = NULL;
nav_msgs::Odometry *last_pos;

double last_theta = 0;


// source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
struct Quaternion {
    double w, x, y, z;
};

Quaternion toQuaternion(double yaw, double pitch, double roll) { // yaw (Z), pitch (Y), roll (X)
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


void posCallback(const ceres_msgs::WheelPositions::Ptr &data) {
  

  if(last_msg != NULL) {
    // Header der Payload
    nav_msgs::Odometry *payload = new nav_msgs::Odometry;
    payload->header.frame_id = "odom";
    payload->header.stamp = ros::Time::now();
    payload->child_frame_id = "base_footprint";
    

    // Ausrechnen der ganzen Werte
    double pos_diff_front_left = data->pos_front_left - last_msg->pos_front_left;
    double pos_diff_front_right = data->pos_front_right - last_msg->pos_front_right;
    double pos_diff_rear_left = data->pos_rear_left - last_msg->pos_rear_left;
    double pos_diff_rear_right = data->pos_rear_right - last_msg->pos_rear_right;

    double delta_s_l = (pos_diff_front_left + pos_diff_rear_left) / 2;
    double delta_s_r = (pos_diff_front_right + pos_diff_rear_right) / 2;

    double delta_s = 0.5 * (delta_s_l + delta_s_r);

    double delta_theta = (delta_s_r - delta_s_l) / 0.435;

    payload->pose.pose.position.x = last_pos->pose.pose.position.x + delta_s * cos(last_theta + delta_theta * 0.5);
    payload->pose.pose.position.y = last_pos->pose.pose.position.y + delta_s * sin(last_theta + delta_theta * 0.5);

    // Umrechnen der z-Achsen Rotation in Quaternion
    last_theta = last_theta + delta_theta;
    Quaternion q = toQuaternion(last_theta, 0, 0);
    payload->pose.pose.orientation.w = q.w;
    payload->pose.pose.orientation.x = q.x;
    payload->pose.pose.orientation.y = q.y;
    payload->pose.pose.orientation.z = q.z;

    // delta t ist die Zeit zwischen den letzten beiden Nachrichten
    double delta_t = payload->header.stamp.toSec() - last_msg->stamp.toSec();

    double v_x = delta_s / delta_t;

    double v_theta = delta_theta / delta_t;


    // Setzen der linear Geschw. in die x-Achse
    payload->twist.twist.linear.x = v_x;

    // Setzen der Rotationsgeschwindigkeit um die z-Achse
    payload->twist.twist.angular.z = v_theta;

  

    wheel_pub.publish(*payload);
    ros::spinOnce();

    delete last_pos;
    last_pos = payload;
  }

  //delete last_msg;
  last_msg = data;

  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "robot_odom_node");

  ros::NodeHandle n;

  // publisher
  wheel_pub = n.advertise<nav_msgs::Odometry>("odom_est", 1000);

  // 'Leere' Nachricht als Startzustand erstellen
  last_pos = new nav_msgs::Odometry;
  last_pos->header.frame_id = "odom";
  last_pos->header.stamp = ros::Time::now();
  last_pos->child_frame_id = "base_footprint";
  last_pos->pose.pose.position.x = 0;
  last_pos->pose.pose.position.y = 0;
  last_pos->pose.pose.position.z = 0;
  last_pos->twist.twist.angular.x = 0;
  last_pos->twist.twist.linear.z = 0;


  // subscriber
  ros::Subscriber sub = n.subscribe("wheel_positions", 1000, posCallback);
  ros::spin();

  return 0;
}


