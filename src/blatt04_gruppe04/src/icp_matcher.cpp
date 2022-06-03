#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher marker_pub;
ros::Publisher test_pub;

// generate random float between two values
float randomFloat(float a, float b) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
}

// given two laserscans finds the nerarest point in the second laserscan to the first laserscan and publish these relations as line list markers
void matcherCallback(const sensor_msgs::LaserScan::ConstPtr &model, const sensor_msgs::LaserScan::ConstPtr &scan) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "matcher";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    //marker.scale.y = 0.1;

    marker.pose.orientation.w = 1.0;

    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // converts the model scan to a PointCloud2
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 modelCloud;
    projector.projectLaser(*model, modelCloud);

    // converts the laser scan to a PointCloud2
    sensor_msgs::PointCloud2 scanCloud;
    projector.projectLaser(*scan, scanCloud);

    test_pub.publish(scanCloud);

    // nearest neighbor search for two point clouds

    // reinterpret byte memory as float memory
    float* modelPoints = reinterpret_cast<float*>(&modelCloud.data[0]);
    float* scanPoints = reinterpret_cast<float*>(&scanCloud.data[0]);
    ROS_INFO_STREAM ("modelCloud.data.size: " << modelCloud.data.size());
    for(int i = 0; i < modelCloud.width; i++) {
        int index = -1;
        float minDist = std::numeric_limits<float>::max();
        
        for(int j = 0; j < scanCloud.width; j++) {
            // euclidean distance
            float dist = sqrt(pow(modelPoints[i*4] - scanPoints[j*4], 2) + pow(modelPoints[i*4+1] - scanPoints[j*4+1], 2) + pow(modelPoints[i*4+2] - scanPoints[j*4+2], 2));
            if(dist < minDist) {
                minDist = dist;
                index = j;
            }
        }
        ROS_INFO_STREAM ("minDist: " << minDist);
        if(index != -1 && minDist < 0.4) {
/*             marker.points.push_back(toGP(modelPoints[i*4], modelPoints[i*4+1], modelPoints[i*4+2]));
            marker.points.push_back(toGP(scanPoints[index*4], scanPoints[index*4+1], scanPoints[index*4+2])); */
        
            geometry_msgs::Point scanPoint;
            geometry_msgs::Point modelPoint;

            scanPoint.x = scanPoints[i*4];
            scanPoint.y = scanPoints[i*4+1];
            scanPoint.z = scanPoints[i*4+2];

            modelPoint.x = modelPoints[index*4];
            modelPoint.y = modelPoints[index*4+1];
            modelPoint.z = modelPoints[index*4+2];

            marker.points.push_back(scanPoint);
            marker.points.push_back(modelPoint);
        }
    }
    
    marker_pub.publish(marker);

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "icp_matcher");

    ros::NodeHandle nh;

    // publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("ref_lines", 1000);
    test_pub = nh.advertise<sensor_msgs::PointCloud2>("test", 1000);


    // subscriber
    message_filters::Subscriber<sensor_msgs::LaserScan> model_sub(nh, "/model", 1000);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), model_sub, scan_sub);
    sync.registerCallback(boost::bind(&matcherCallback, _1, _2));


    ros::spin();

    return 0;
}
