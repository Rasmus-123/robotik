#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <stdio.h>


ros::Publisher marker_pub;
std::string dest_node;


struct Point {
    float x,y,z;
};

double euklidDist(Point a, Point b) {
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

geometry_msgs::Point toGP(Point p) {
    geometry_msgs::Point tp;
    tp.x = p.x;
    tp.y = p.y;
    tp.z = p.z;
    return tp;
}


void markerCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = data->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "lines";
    marker.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::LINE_LIST;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1;
    
    const Point* data_raw = reinterpret_cast<const Point*>(&data->data[0]);    

    int k = 0;
    int j = 0;

    while(j < data->width - 2) {
        k = j + 1;

        geometry_msgs::Point start_point = toGP(data_raw[j]);
        geometry_msgs::Point end_point;
        int points_on_line = 2;


        while(k < data->width - 1) {
            
            double zaehler = euklidDist(data_raw[j], data_raw[k+1]);

            if(zaehler >= 1.5) {
                k++;
                break;
            }

            double nenner = 0.0;
            std::string s = "";
            for(int i = j; i <= k; i++) {
                nenner += euklidDist(data_raw[i], data_raw[i+1]);
                s += "(" + std::to_string(i) + ":" + std::to_string(i+1) + "): " + std::to_string(euklidDist(data_raw[i], data_raw[i+1])) + ", ";
            }

/*             if(k == j + 1) {
                ROS_INFO("---------------------------------------------------------");
                ROS_INFO("xp for j changed: %f",exp(-0.1 * (k-j)));
            } else {
                ROS_INFO("exp: %f",exp(-0.1 * (k-j)));
            }

            ROS_INFO("erg: %f",zaehler / nenner);
             */
            
            ROS_INFO("zaehler (%i, %i): %f", j,k+1, zaehler);
            ROS_INFO_STREAM(s);

            if(zaehler / nenner >= 1 - exp(-0.1 * (k-j))) {
                double a = euklidDist(data_raw[k-1], data_raw[k+1]);
                double b = euklidDist(data_raw[k-1], data_raw[k]) + euklidDist(data_raw[k], data_raw[k+1]);

                if(a / b >= 1 - 0.005) {
                    points_on_line++;
                    end_point = toGP(data_raw[k+1]);
                } else {
                    break;
                }
            } else {
                break;
            }
            k++;
        }

        j = k;
        if(points_on_line >= 3) {
            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
            j--;
        }

 
        // Continue with j from the last ended line
        //ROS_INFO("points: %i",points_on_line);

    }
    marker_pub.publish(marker);  
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "online_line_finder");

    ros::NodeHandle n;

    // publisher
    marker_pub = n.advertise<visualization_msgs::Marker>("marker_top", 1000);


    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, markerCallback);
    ros::spin();

    return 0;
}
