#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>



constexpr int DIMENSION_ZUSTAND = 1; // n
constexpr int DIMENSION_AKTION = 1; // m
constexpr int DIMENSION_MESSUNG = 1; // l

/**
 * Eigen
 * 
 * Eigen::Matrix<type, rows, cols>
 * 
 * 
 * Eigen::Vector-Typen sind nur typedefs für Matrix mit cols = 1
 * 
 */

// Einheitsmatrix (n X n)
typedef Eigen::Matrix<double, DIMENSION_ZUSTAND, DIMENSION_ZUSTAND> EinheitsMatrix;


// Zustandsraum x
typedef Eigen::Matrix<double, DIMENSION_ZUSTAND, 1> Zustandsraum;

// Aktionsraum u
typedef Eigen::Matrix<double, DIMENSION_AKTION, 1> Aktionsraum;

// Messraum Z
typedef Eigen::Matrix<double, DIMENSION_MESSUNG, 1> Messraum;


// Transitionsmodell A (n X n) - Entspricht Einheitsmatrix, wenn keine spontanen Transitionen erfolgen
typedef Eigen::Matrix<double, DIMENSION_ZUSTAND, DIMENSION_ZUSTAND> Transitionsmodell;

// Aktionsmodell B (n X m) - Konvertiert den Aktionsraum in den Zustandsraum
typedef Eigen::Matrix<double, DIMENSION_ZUSTAND, DIMENSION_AKTION> Aktionsmodell;

// Kovarianzmatrix Sigma_u für Aktionsmodell (n X n)
typedef Eigen::Matrix<double, DIMENSION_ZUSTAND, DIMENSION_ZUSTAND> Kovarianzaktionsmodell;

// Sensormodell H (l X n) - Konvertiert Zustand in den Messraum
typedef Eigen::Matrix<double, DIMENSION_MESSUNG, DIMENSION_ZUSTAND> Sensormodell;

// Kovarianzmatrix Sigma_Z für Sensormodell (l X l)
typedef Eigen::Matrix<double, DIMENSION_MESSUNG, DIMENSION_MESSUNG> Kovarianzsensormodell;


const EinheitsMatrix einheitsmatrix = EinheitsMatrix::Identity();

ros::Publisher pub;


/**
 * Eingabe sollten: Aktueller Zustand x_t, aktuelles Fehlermodell Sigma_t und ein Evidenzpaar(u, z) sein
 * 
 * (Fehlermodell berechnet man doch auch selbst oder nicht?)
 * 
 */



auto prediction(const Transitionsmodell& A, const Zustandsraum& x, const Aktionsmodell& B, const Aktionsraum& u, const Kovarianzaktionsmodell& Sigma_u)
{
    auto new_x_strich = A * x + B * u;
    // old_sigma der Erfahrungswert oder so?
    auto old_sigma = einheitsmatrix; // temp
    auto new_sigma_strich = A * old_sigma * A.transpose() + Sigma_u;

    return std::make_tuple(new_x_strich, new_sigma_strich);
}

auto correction(const Sensormodell& H, const Kovarianzsensormodell& Sigma_Z, const Messraum& Z)
{
    /* new_x_strich + new_sigma_strich der prediction */ 
    auto new_x_strich = einheitsmatrix; // temp 
    auto new_sigma_strich = einheitsmatrix; // temp 

    // Kalman Gewinnmatrix (Kalman Gain)
    auto new_K = new_sigma_strich * H.transpose() * (H * new_sigma_strich * H.transpose() + Sigma_Z).inverse();

    auto new_x = new_x_strich + new_K * (Z - H * new_x_strich);

    auto new_sigma = (einheitsmatrix - new_K * H) * new_sigma_strich;

    return std::make_tuple(new_x, new_sigma);
}


void callback(const sensor_msgs::ImuConstPtr& imu, const nav_msgs::OdometryConstPtr& odom)
{
    ROS_INFO("Hallo from Callback!");

    // Was gehört zum Zustandsraum, Aktionsraum und Messraum?

    // Zustandsraum Position und Orientiertung?

    // Aktionsraum vielleicht die Dreh + Fahrgeschwindigkeit?

    // Messraum?

    geometry_msgs::PoseStamped msg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_filter_node");

    ROS_INFO("Hello");

    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>("kalman_filter_localization", 1);

    // https://wiki.ros.org/message_filters

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> ApproximatePolicy;

    message_filters::Synchronizer<ApproximatePolicy> sync(ApproximatePolicy(10), imu_sub, odom_sub);

    // Exakter Synchronizer funktioniert nicht
    //message_filters::TimeSynchronizer<sensor_msgs::Imu, nav_msgs::Odometry> sync(imu_sub, odom_sub, 10);

    sync.registerCallback(callback);


    ros::spin();

    return 0;
}