#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>


ros::Publisher tf_pub;
std::string dest_node;


struct Point {
    double x,y,z;
};

double euklidDist(Point a, Point b) {
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}


void tfCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    geometry_msgs::TransformStamped payload;

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;


    //for(int i = 0; i < data->fields.front())
   
    const Point* data_raw = reinterpret_cast<const Point*>(&data->data[0]);

    int j = 0;
    for(int k = 0; k < sizeof(data_raw) / sizeof(data_raw[0]); k++) {
        double zaehler = euklidDist(data_raw[j], data_raw[k+1]);
        double nenner;
        for(int i = j; j <= k; i++) {
            nenner += euklidDist(data_raw[i], data_raw[i+1]);
        }
        if(zaehler / nenner > 1) {
            
        }
    
    }

    ros::spinOnce();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "online_line_finder");

    ros::NodeHandle n;

    // publisher
    tf_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1000);


    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, tfCallback);
    ros::spin();

    return 0;
}
