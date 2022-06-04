#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>

ros::Publisher map_pub;

nav_msgs::OccupancyGrid map;


double get_yaw(const geometry_msgs::Quaternion& q)
{
    // convert geometry_msgs::Quaternion to roll pitch yaw
    tf::Quaternion qa(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(qa);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

// convert laserscan to map
void mapCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose, const sensor_msgs::LaserScan::ConstPtr &laser)
{
    // tries to avoid some mapping while rotating
    static double yaw;
    if(std::abs(yaw - get_yaw(pose->pose.pose.orientation)) < 0.01) {
        map.header.stamp = laser->header.stamp;

        // laserscan to pointcloud
        laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud;
        projector.projectLaser(*laser, cloud);

        // transform laserscan with pose
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        while(ros::ok()) {
            try {
                transformStamped = tfBuffer.lookupTransform("odom_combined", cloud.header.frame_id, ros::Time(0));
                break;
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }

        tf2::doTransform(cloud, cloud, transformStamped);


        // pointcloud to map
        float* points = reinterpret_cast<float*>(&cloud.data[0]);
        for (int i = 0; i < cloud.width; i ++) {
            int x = (points[i*5] - map.info.origin.position.x) / map.info.resolution;
            int y = (points[i*5 + 1] - map.info.origin.position.y) / map.info.resolution;

            if (x < 0 || x > map.info.width || y < 0 || y > map.info.height) {
                ROS_INFO_STREAM("X oder Y Out of Range: x: " << x << ", y: " << y);
                continue;
            }
            map.data[y * map.info.width + x] = 100;
        }

        // publish map
        map_pub.publish(map);
    }

    yaw = get_yaw(pose->pose.pose.orientation);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "fill_map");

    ros::NodeHandle nh;

    // publisher
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1000);


    map.header.frame_id = "odom_combined";
    map.info.resolution = 0.1;
    map.info.width = 1000;
    map.info.height = 1000;
    map.info.origin.position.x = -50;
    map.info.origin.position.y = -50;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 1;
    
    map.data.resize(map.info.width * map.info.height);

    // subscriber
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> ekf_sub(nh, "/robot_pose_ekf/odom_combined", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ekf_sub, laser_sub);
    sync.registerCallback(boost::bind(&mapCallback, _1, _2));


    ros::spin();

    return 0;
}


