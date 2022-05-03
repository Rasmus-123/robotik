#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>


ros::Publisher tf_pub;
std::string dest_node;



void tfCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    geometry_msgs::TransformStamped payload;
    std::string last_topic;
/*     if(!last_topic.compare(dest_node)) {
        tf_pub = np.advertise<sensor_msgs::PointCloud2>(dest_node, 1000);

    } 
    last_topic = dest_node; */

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(dest_node, "base_link",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
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

    np.param<std::string>("dest_name", dest_node, "base_link");

    np.getParam("dest_name", dest_node);

    // publisher
    tf_pub = np.advertise<sensor_msgs::PointCloud2>("cloud", 1000);


    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, tfCallback);
    ros::spin();

    return 0;
}
