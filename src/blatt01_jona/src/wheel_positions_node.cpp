#include "ceres_msgs/MotorRotations.h"
#include "ceres_msgs/WheelPositions.h"
#include "ros/ros.h"
#include "cmath"

ros::Publisher wheel_pub;


void posCallback(const ceres_msgs::MotorRotations::ConstPtr &data) {
  ceres_msgs::WheelPositions payload;
  payload.stamp = ros::Time::now();
  // (rot / 2pi) * 2pi * r = rot * r
  payload.pos_front_left = data->rot_front_left * 0.13;
  payload.pos_front_right = data->rot_front_right * 0.13;
  payload.pos_rear_left = data->rot_rear_left * 0.13;
  payload.pos_rear_right = data->rot_rear_right * 0.13;
  wheel_pub.publish(payload);
  ros::spinOnce();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "wheel_positions_node");

  ros::NodeHandle n;

  // publisher
  wheel_pub = n.advertise<ceres_msgs::WheelPositions>("wheel_positions", 1000);

  // subscriber
  ros::Subscriber sub = n.subscribe("wheel_rotations", 1000, posCallback);
  ros::spin();

  return 0;
}
