#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/TransformStamped.h"

ros::Publisher tf_pub;
std::string dest_node;



void tfCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    geometry_msgs::TransformStamped payload;
    std::string last_topic;
/*     if(!last_topic.compare(dest_node)) {
        tf_pub = np.advertise<sensor_msgs::PointCloud2>(dest_node, 1000);

    } */

    last_topic = dest_node;
    tf_pub.publish(payload);
    ros::spinOnce();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "transform_cloud");

    ros::NodeHandle n;
    ros::NodeHandle np("~");

    np.param<std::string>("dest_name", dest_node, "base_link");

    np.getParam("my_int", dest_node);

    // publisher
    tf_pub = np.advertise<sensor_msgs::PointCloud2>(dest_node, 1000);

    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, tfCallback);
    ros::spin();

    return 0;
}
