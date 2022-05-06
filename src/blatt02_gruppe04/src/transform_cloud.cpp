#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
// For some reason, tf2_sensor_msgs was not part of the AUR package ros-noetic-desktop-full
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>


ros::Publisher tf_pub;
std::string dest_node;



void tfCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    geometry_msgs::TransformStamped payload;
    std::string last_topic;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    while(ros::ok()) {
        try {
            transformStamped = tfBuffer.lookupTransform(dest_node, data->header.frame_id,
                                ros::Time(0));
            break;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
    }

    sensor_msgs::PointCloud2 data_out;
    tf2::doTransform(*data, data_out, transformStamped);

   
    tf_pub.publish(data_out);
    ros::spinOnce();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "transform_cloud");

    ros::NodeHandle n;
    ros::NodeHandle np("~");

    np.param<std::string>("dest_name", dest_node, "laser");

    np.getParam("dest_name", dest_node);

    // publisher
    tf_pub = np.advertise<sensor_msgs::PointCloud2>("cloud", 1000);


    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, tfCallback);
    ros::spin();

    return 0;
}
