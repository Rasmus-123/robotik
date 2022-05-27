#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>

ros::Publisher map_pub;

// convert laserscan to map
void mapCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose, const sensor_msgs::LaserScan::ConstPtr &laser)
{

    // transform laser with pose
    geometry_msgs::TransformStamped payload;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    while(ros::ok()) {
        try {
            transformStamped = tfBuffer.lookupTransform(pose->header.frame_id, laser->header.frame_id,
                                ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }
    
    sensor_msgs::LaserScan transformedLaser;
    //tfBuffer.transform(*laser, transformedLaser, pose->header.frame_id);
    tf2::doTransform(*laser, transformedLaser, transformStamped);


    // create map
    nav_msgs::OccupancyGrid map;
    map.info.resolution = 0.1;
    map.info.width = (transformedLaser.angle_max - transformedLaser.angle_min) / transformedLaser.angle_increment;
    map.info.height = (transformedLaser.range_max - transformedLaser.range_min) / map.info.resolution;
    map.info.origin.position.x = -transformedLaser.angle_min;
    map.info.origin.position.y = -transformedLaser.range_min;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 1;
    map.data.resize(map.info.width * map.info.height);




    // fill map
    for (int i = 0; i < transformedLaser.ranges.size(); i++)
    {
        if (transformedLaser.ranges[i] < transformedLaser.range_max && transformedLaser.ranges[i] > transformedLaser.range_min)
        {
            // adjust scan data with pose TODO
            int x = (int)((transformedLaser.ranges[i] - transformedLaser.range_min) / map.info.resolution);
            int y = (int)((transformedLaser.angle_min + transformedLaser.angle_increment * i) / map.info.resolution);
            map.data[y * map.info.width + x] = 100;
        }
    }

    // publish map
    map_pub.publish(map);
}

/* void mapCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr &pose, sensor_msgs::LaserScan::Ptr &laser) {

    // get pose
    float x = pose->pose.pose.position.x;
    float y = pose->pose.pose.position.y;
    float theta = tf::getYaw(pose->pose.pose.orientation);

    // transform laserscan
    for (int i = 0; i < ranges_size; i++)
    {
        float x_new = x + ranges[i] * cos(angle_min + angle_increment * i + theta);
        float y_new = y + ranges[i] * sin(angle_min + angle_increment * i + theta);
    }

} */

int main(int argc, char **argv) {

    ros::init(argc, argv, "fill_map");

    ros::NodeHandle nh;

    // publisher
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("odom_est", 1000);


    // subscriber
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> ekf_sub(nh, "/robot_pose_ekf/odom_combined", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ekf_sub, laser_sub);
    sync.registerCallback(boost::bind(&mapCallback, _1, _2));


    ros::spin();

    return 0;
}


