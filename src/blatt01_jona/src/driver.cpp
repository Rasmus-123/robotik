#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "driver");

  ros::NodeHandle n;

  // publisher
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // rate einstellen
  ros::Rate loop_rate(100);

  while (ros::ok()) {

    geometry_msgs::Twist payload;

    // payload vorbereiten
    payload.linear.x = 0.5;
    payload.linear.y = 0;
    payload.linear.z = 0;

    payload.angular.x = 0;
    payload.angular.y = 0;
    payload.angular.z = 0;


    // publish
    chatter_pub.publish(payload);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}