#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

ros::Publisher marker_pub;

const int neighbors = 7; 


geometry_msgs::Point toGP(float x, float y, float z) {
    geometry_msgs::Point tp;
    tp.x = x;
    tp.y = y;
    tp.z = z;
    return tp;
}

std::vector<geometry_msgs::Point> laserScanToPointVector(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // converts the laser scan to a PointCloud2
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan, cloud);

    // reinterpret byte memory as float memory
    float* point_array_ptr = reinterpret_cast<float*>(&cloud.data[0]);
    
    // Turn Points from point_array_ptr into geometry_msgs::Points
    std::vector<geometry_msgs::Point> points;
    points.reserve(cloud.width);
    for (int i = 0; i < cloud.width; i++)
    {
        points.push_back(toGP(point_array_ptr[i*4], point_array_ptr[i*4+1], point_array_ptr[i*4+2]));
    }

    return points;
}

std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> findCorrespondences(const std::vector<geometry_msgs::Point>& scan_points, const std::vector<geometry_msgs::Point>& model_points)
{
    std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> pairs;
    
    for(int j = 0; j < scan_points.size(); j++) {
        int index = -1;
        float minDist = std::numeric_limits<float>::max();
        
        for(int i = 0; i < model_points.size(); i++) {
            // euclidean distance
            float dist = sqrt(pow(model_points[i].x - scan_points[j].x, 2) + pow(model_points[i].y - scan_points[j].y, 2) + pow(model_points[i].z - scan_points[j].z, 2));
            if(dist < minDist) {
                minDist = dist;
                index = i;
            }
        }

        if(minDist < 0.4) {
            geometry_msgs::Point scan_p = scan_points[j];
            geometry_msgs::Point model_p = model_points[index];
            
            pairs.push_back(std::make_pair(scan_p, model_p));
        }
    }
    
    return pairs;
}

// given two laserscans finds the calculates the icp transformation and outputs it on the console
void icpCallback(const sensor_msgs::LaserScan::ConstPtr &model, const sensor_msgs::LaserScan::ConstPtr &scan) {
    /*
    // converts the model scan to a PointCloud2
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 modelCloud;
    projector.projectLaser(*model, modelCloud);

    // converts the laser scan to a PointCloud2
    sensor_msgs::PointCloud2 scanCloud;
    projector.projectLaser(*scan, scanCloud);

    // nearest neighbor search for two point clouds

    // reinterpret byte memory as float memory
    float* modelPointsPtrArray = reinterpret_cast<float*>(&modelCloud.data[0]);
    float* scanPointsPtrArray = reinterpret_cast<float*>(&scanCloud.data[0]);

    std::vector<geometry_msgs::Point> model_points;
    std::vector<geometry_msgs::Point> scan_points;
    model_points.reserve(modelCloud.width);
    scan_points.reserve(scanCloud.width);

    for (int i = 0; i < modelCloud.width; i++)
    {
        model_points.push_back(toGP(modelPointsPtrArray[i*4], modelPointsPtrArray[i*4+1], modelPointsPtrArray[i*4+2]));
    }
    
    for (int i = 0; i < scanCloud.width; i++)
    {
        scan_points.push_back(toGP(scanPointsPtrArray[i*4], scanPointsPtrArray[i*4+1], scanPointsPtrArray[i*4+2]));
    }
    */
   
    std::vector<geometry_msgs::Point> model_points = laserScanToPointVector(model);
    std::vector<geometry_msgs::Point> scan_points = laserScanToPointVector(scan);

    // correspondences from scan to model
    // first = scan; second = model
    std::vector<std::pair<geometry_msgs::Point, geometry_msgs::Point>> pairs = findCorrespondences(scan_points, model_points);
    /*
    for(int j = 0; j < scanCloud.width; j++) {
        int index = -1;
        float minDist = std::numeric_limits<float>::max();
        
        for(int i = 0; i < modelCloud.width; i++) {
            // euclidean distance
            float dist = sqrt(pow(model_points[i].x - scan_points[j].x, 2) + pow(model_points[i].y - scan_points[j].y, 2) + pow(model_points[i].z - scan_points[j].z, 2));
            if(dist < minDist) {
                minDist = dist;
                index = i;
            }
        }

        if(minDist < 0.4) {
            geometry_msgs::Point scan_p = scan_points[j];
            geometry_msgs::Point model_p = model_points[index];
            
            pairs.push_back(std::make_pair(scan_p, model_p));
        }
    }
    */

    // calculate center of mass of scanCloud
    float scanCenterOfMass[3] = {0, 0, 0};
    for(int i = 0; i < scan_points.size(); i++) {
        scanCenterOfMass[0] += scan_points[i].x;
        scanCenterOfMass[1] += scan_points[i].y;
        scanCenterOfMass[2] += scan_points[i].z;
    }
    scanCenterOfMass[0] /= scan_points.size();     // c'x | Scan
    scanCenterOfMass[1] /= scan_points.size();     // c'y
    scanCenterOfMass[2] /= scan_points.size();     // c'z


    // calculate center of mass of modelCloud
    float modelCenterOfMass[3] = {0, 0, 0};
    for(int i = 0; i < model_points.size(); i++) {
        modelCenterOfMass[0] += model_points[i].x;
        modelCenterOfMass[1] += model_points[i].y;
        modelCenterOfMass[2] += model_points[i].z;
    }
    modelCenterOfMass[0] /= model_points.size();   // cx | Model
    modelCenterOfMass[1] /= model_points.size();   // cy
    modelCenterOfMass[2] /= model_points.size();   // cz
    
/*     // translate center of scanCloud to center of modelCloud
    float diff[3] = {scanCenterOfMass[0] - modelCenterOfMass[0], scanCenterOfMass[1] - modelCenterOfMass[1], scanCenterOfMass[2] - modelCenterOfMass[2]};
    for(int i = 0; i < scan_points.size(); i++) {
        scanPointsPtrArray[i*4] -= diff[0];
        scanPointsPtrArray[i*4+1] -= diff[1];
        scanPointsPtrArray[i*4+2] -= diff[2];
    } */

    // calc S
    float Sxx;
    float Sxy;
    float Syx;
    float Syy;
    for(const auto& pair : pairs)
    {
        Sxx += (pair.first.x - modelCenterOfMass[0]) * (pair.second.x - scanCenterOfMass[0]);
        Sxy += (pair.first.x - modelCenterOfMass[0]) * (pair.second.y - scanCenterOfMass[1]);
        Syx += (pair.first.y - modelCenterOfMass[1]) * (pair.second.x - scanCenterOfMass[0]);
        Syy += (pair.first.y - modelCenterOfMass[1]) * (pair.second.y - scanCenterOfMass[1]);
    }

    float theta = std::atan(Syx - Sxy / Sxx - Syy);

    float tx = modelCenterOfMass[0] - (scanCenterOfMass[0] * std::cos(theta) - scanCenterOfMass[1] * std::sin(theta));
    float ty = modelCenterOfMass[1] - (scanCenterOfMass[0] * std::sin(theta) + scanCenterOfMass[1] * std::cos(theta));


    // T = T + Matrix + Delta_T
    Eigen::Matrix<double, 3, 1> T = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 3, 1> deltaT;
    deltaT << tx, ty, theta;
    Eigen::Matrix<double, 3, 3> R;
    R << std::cos(theta), -std::sin(theta), 0,
            std::sin(theta), std::cos(theta), 0,
            0, 0, 1;
    
    T = T + R * deltaT;

    // transform scan with T
    for(auto& p : model_points) {
        R(0, 2) = tx;
        R(1, 2) = ty;

        Eigen::Matrix<double, 3, 1> pM(p.x, p.y, p.z);

        pM = R * pM;

        p.x = pM(0);
        p.y = pM(1);
        p.z = pM(2);        
    }

    ROS_INFO_STREAM("T: (" << T(0) << ", " << T(1) << ", " << T(2) << ")");

    // calculate error
    float error = 0;
    for(auto& p : pairs) {
        Eigen::Matrix<double, 3, 1> modelPointMatrix;
        modelPointMatrix << p.second.x, p.second.y, p.second.z;
        Eigen::Matrix<double, 3, 1> scanPointMatrix;
        scanPointMatrix << p.first.x, p.first.y, p.first.z;
        
        error += (modelPointMatrix - R * scanPointMatrix).array().abs().sum();
    }

    ROS_INFO_STREAM("Error: " << error);

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "icp_matcher");

    ros::NodeHandle nh;

    // publisher
    marker_pub = nh.advertise<visualization_msgs::Marker>("ref_lines", 1000);


    // subscriber
    message_filters::Subscriber<sensor_msgs::LaserScan> model_sub(nh, "/model", 1000);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 1000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), model_sub, scan_sub);
    sync.registerCallback(boost::bind(&icpCallback, _1, _2));

    ros::spin();

    return 0;
}
