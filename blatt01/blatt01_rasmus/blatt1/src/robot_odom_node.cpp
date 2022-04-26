#include <ros/ros.h>

#include <csignal>

#include <ceres_msgs/WheelPositions.h>

#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

// --- //

ros::Publisher pub;

void signalHandler(int signum)
{
    ROS_INFO("Goodbye..");

    ros::shutdown();
}

/**
 * WheelPositionsDelta(Nachricht1, Nachricht2)
 * 
 * Rechnet n2 - n1 für alle Variablen
 * 
 */
struct WheelPositionsDelta
{
public:

    WheelPositionsDelta(ceres_msgs::WheelPositionsConstPtr first, ceres_msgs::WheelPositionsConstPtr second)
        : duration(second->stamp - first->stamp), 
          pos_front_left(second->pos_front_left - first->pos_front_left),
          pos_front_right(second->pos_front_right - first->pos_front_right),
          pos_rear_left(second->pos_rear_left - first->pos_rear_left),
          pos_rear_right(second->pos_rear_right - first->pos_rear_right)
    {}

    ros::Duration duration;
    float pos_front_left;
    float pos_front_right;
    float pos_rear_left;
    float pos_rear_right;
};

void wheelPositionsCallback(const ceres_msgs::WheelPositionsConstPtr& msg)
{
    // Vorherige Nachricht vom letzten Callback
    static ceres_msgs::WheelPositionsConstPtr previous;

    // Aktuelle Pose
    static geometry_msgs::Pose old_pose;

    // Aktueller Winkel
    static float theta = 0;

    // Erste Nachricht
    if (!previous)
    {
        // Start mit WheelPos 0,0,0,0
        previous = ceres_msgs::WheelPositionsConstPtr(new ceres_msgs::WheelPositions());
    }

    WheelPositionsDelta delta(previous, msg); // Positionsänderung seit letzer Nachricht

    float deltasl = (delta.pos_front_left + delta.pos_rear_left) / 2;   // Mittlere Änderung Links
    float deltasr = (delta.pos_front_right + delta.pos_rear_right) / 2; // Mittlere Änderung Rechts

    constexpr float b = 0.435; // Achsenbreite

    float deltas = (deltasl+deltasr) / 2;       // Gefahrene Strecke
    float deltatheta = (deltasr - deltasl) / b; // Winkeländerung
    
    geometry_msgs::Pose pose; // Neue Pose

    // Vektorgleichungen aus dem Übungsblatt

    // x_t = x_t-1 + deltas * cos(theta_t-1 + deltatheta * 0.5)
    pose.position.x = old_pose.position.x + deltas * std::cos(theta + deltatheta / 2);
    // y_t = y_t-1 + deltas * sin(theta_t-1 + deltatheta * 0.5)
    pose.position.y = old_pose.position.y + deltas * std::sin(theta + deltatheta / 2);

    // theta_t = theta_t-1 + deltatheta
    theta += deltatheta;
    
    // Pose braucht Winkel in Quaternionform http://wiki.ros.org/tf2/Tutorials/Quaternions
    tf2::Quaternion quat;
    // Roll(um x-Achse drehen), Pitch(um y), Yaw(um z) - wir brauchen z yaw
    quat.setRPY(0, 0, theta);
    pose.orientation = tf2::toMsg(quat);

    // Linear-Geschwindigkeit vx | x-Achse
    float velocityLinear = deltas / delta.duration.toSec();         
    // Winkel-Geschwindigkeit vtheta | z-Achse(yaw)
    float velocityAngular = deltatheta / delta.duration.toSec();    

    geometry_msgs::Twist twist;
    twist.linear.x = velocityLinear;
    twist.angular.z = velocityAngular;

    // Einfach +1 bei jeder Nachricht
    static int seq = 0;

    std_msgs::Header header;
    header.frame_id = "odom";
    header.stamp = msg->stamp;
    header.seq = seq++;

    nav_msgs::Odometry toSend;

    toSend.child_frame_id = "base_footprint";   // std::string
    toSend.header = header;                     // std_msgs::header
    toSend.pose.pose = pose;                    // geometry_msgs::PoseWithCovariance // Covariance ignoriert
    toSend.twist.twist = twist;                 // geometry_msgs::TwistWithCovariance // Covariance ignoriert

    previous = msg;
    old_pose = pose;

    //std::cout << "ToSend:" << std::endl << toSend << std::endl << std::endl;
    pub.publish(toSend);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "robot_odom_node", ros::init_options::NoSigintHandler);

    std::signal(SIGINT, signalHandler);


    ros::NodeHandle nh;

    pub = nh.advertise<nav_msgs::Odometry>("odom_est", 10);

    ros::Subscriber sub = nh.subscribe("wheel_positions", 10, wheelPositionsCallback);

    ros::spin();
}