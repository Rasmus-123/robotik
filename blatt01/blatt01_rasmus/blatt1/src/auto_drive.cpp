#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <csignal>

void signalHandler(int signum)
{
    ROS_INFO("Goodbye!");

    ros::shutdown();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "auto_drive", ros::init_options::NoSigintHandler);


    std::signal(SIGINT, signalHandler);

    ROS_INFO("Hallo!");

    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Linear-Geschwindigkeit von 0.5
    geometry_msgs::Twist msg;
    msg.linear.x = 0.5;

    while(true)
    {
        pub.publish(msg);
        // sleep?
    }

    ros::spinOnce();
}