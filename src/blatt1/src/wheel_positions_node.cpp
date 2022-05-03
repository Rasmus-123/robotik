#include <ros/ros.h>

#include <csignal>

#include <ceres_msgs/WheelPositions.h>
#include <ceres_msgs/MotorRotations.h>

#include <cmath>

// --- //

ros::Publisher pub;


void signalHandler(int signum)
{
    ROS_INFO("Goodbye..");

    ros::shutdown();
}

void rotationsCallback(const ceres_msgs::MotorRotationsConstPtr& msg)
{
    ceres_msgs::WheelPositions toSend;

    constexpr float wheelDiameter = 0.13f;                      // Raddurchmesser 0.13m
    constexpr float pi = 3.14159265358979323846264338327950f;   // Pi

    constexpr float wheelCircumference = wheelDiameter * pi;    // Umfang = pi * d

    toSend.stamp = msg->stamp;

    // Gefahrene Distanz (in m) = Umdrehungen * Reifenumfang
    toSend.pos_front_left = msg->rot_front_left * wheelCircumference;
    toSend.pos_front_right = msg->rot_front_right * wheelCircumference;
    toSend.pos_rear_left = msg->rot_rear_left * wheelCircumference;
    toSend.pos_rear_right = msg->rot_rear_right * wheelCircumference;

    pub.publish(toSend);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "wheel_positions_node", ros::init_options::NoSigintHandler);

    std::signal(SIGINT, signalHandler);

    ros::NodeHandle nh;

    pub = nh.advertise<ceres_msgs::WheelPositions>("wheel_positions", 10);

    ros::Subscriber sub = nh.subscribe("wheel_rotations", 10, rotationsCallback);

    ros::spin();
}