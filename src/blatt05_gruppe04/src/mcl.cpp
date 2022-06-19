#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>

#include <mcl_helper/scan_simulator.h>

#include <numeric>

#include <Eigen/Dense>

#include <random>


using namespace mcl_helper;


ScanSimulator scan_sim;
bool mapSet = false;

ros::Publisher pose_pub;
ros::Publisher pose_array_pub;

ros::Subscriber map_sub;

geometry_msgs::PoseWithCovarianceStamped last_pose;
geometry_msgs::PoseArray pose_array;

double sigma_weight;
double std_deviation_dist;
double std_deviation_rot;
int num_particles;

// Parameter for getPose
enum Mode {
    WEIGHTED_AVERAGE,
    HIGHEST_WEIGHT,
};

struct Pose2D {
    float x;
    float y;
    float alpha;
};

float get_yaw(const geometry_msgs::Quaternion& q)
{
    tf2::Quaternion rot;
    tf2::convert(q, rot);
    tf2::Matrix3x3 m(rot);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void convert(const geometry_msgs::Pose& from, Pose2D& to)
{
    to.x = from.position.x;
    to.y = from.position.y;
    to.alpha = get_yaw(from.orientation);
}

void convert(const geometry_msgs::Pose& from, Eigen::Affine3d& to)
{
    Eigen::Vector3d translation;
    translation.x() = from.position.x;
    translation.y() = from.position.y;
    translation.z() = from.position.z;
    Eigen::Quaterniond rotation;
    rotation.x() = from.orientation.x;
    rotation.y() = from.orientation.y;
    rotation.z() = from.orientation.z;
    rotation.w() = from.orientation.w;
    to.setIdentity();
    to.linear() = rotation.matrix();
    to.translation() = translation;
}

geometry_msgs::Pose delta_pose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
    Eigen::Affine3d T_p1_map;
    convert(p1, T_p1_map);

    Eigen::Affine3d T_p2_map;
    convert(p2, T_p2_map);

    // we need T_p2_p1
    
    // T_p2_p1 = T_p2->p1 
    Eigen::Affine3d T_p2_p1 = T_p1_map.inverse() * T_p2_map;

    Eigen::Vector3d t = T_p2_p1.translation();
    Eigen::Quaterniond rot;
    rot = T_p2_p1.linear();

    geometry_msgs::Pose delta_p;
    delta_p.position.x = t.x();
    delta_p.position.y = t.y();
    delta_p.position.z = t.z();

    delta_p.orientation.x = rot.x();
    delta_p.orientation.y = rot.y();
    delta_p.orientation.z = rot.z();
    delta_p.orientation.w = rot.w();

    return delta_p;
}

Pose2D apply_delta(const Pose2D& pose, const Pose2D& delta)
{
    Pose2D pose_new;

    pose_new.x = delta.x * cos(pose.alpha) + delta.y * -sin(pose.alpha) + pose.x;
    pose_new.y = delta.x * sin(pose.alpha) + delta.y * cos(pose.alpha) + pose.y;
    pose_new.alpha = pose.alpha + delta.alpha;

    return pose_new;
}

/**
 * Return weighted random Index of a NORMALIZED Vector of Weights
 * 
 * ---
 * 
 * Random in Range
 * 
 * Get Index from the subrange the random falls into
 */
int randomIndex(const std::vector<double>& weights)
{
    double random = (double)rand() / (double)RAND_MAX;
    double sum = 0.0;

    for (int i = 0; i < weights.size(); i++)
    {
        sum += weights[i];

        if (sum > random)
        {
            return i;
        }
    }

    ROS_ERROR("KEIN RANDOM");
    return 0;
    
}

void normalizeVector(std::vector<double>& weights)
{
    double sum = std::accumulate(weights.begin(), weights.end(), 0.0);

    for (double& d : weights)
    {
        d = d / sum;
    }
}

void motionUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {

    if(!last_pose.header.frame_id.empty() && !pose_array.header.frame_id.empty()) {

        Pose2D current_ekf_2d;
        convert(pose->pose.pose, current_ekf_2d);
        
        geometry_msgs::Pose current_ekf = pose->pose.pose;
        geometry_msgs::Pose delta_ekf = delta_pose(last_pose.pose.pose, current_ekf);

        Pose2D delta_ekf_2d;
        convert(delta_ekf, delta_ekf_2d);
        
        for(size_t i=0; i < pose_array.poses.size(); i++)
        {
            Pose2D old_pose;
            old_pose.x = pose_array.poses[i].position.x;
            old_pose.y = pose_array.poses[i].position.y;
            old_pose.alpha = get_yaw(pose_array.poses[i].orientation);
            Pose2D new_pose = apply_delta(old_pose, delta_ekf_2d);
            pose_array.poses[i].position.x = new_pose.x;
            pose_array.poses[i].position.y = new_pose.y;
            pose_array.poses[i].orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0,0,1), new_pose.alpha));
        }

        pose_array.header.stamp = pose->header.stamp;

        pose_array_pub.publish(pose_array);
    }

}

/**
 * Transform base->map
 *  That's the Pose, as the transform describes the positioning of base_link(the robot) inside the map
 * Transform odom->base
 *  Transform is given by ekf
 * Transform odom->map = base->map * odom->base;
 */
void publishOdomMapTransform(const geometry_msgs::Pose& pose)
{
    static tf2_ros::TransformBroadcaster tf_br;


    // Get odom -> base_link Transform //

    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListen(tfBuffer); // Was macht der eigentlich?

    geometry_msgs::TransformStamped tf_odom_base;

    while (ros::ok())
    {
        try
        {
            // target = base_link, source = odom_combined
            tf_odom_base = tfBuffer.lookupTransform("base_link", "odom_combined", ros::Time(0));
            break;
        }
        catch(const std::exception& e)
        {
            ROS_WARN("%s", e.what());
            ros::Duration(1.0).sleep();
        }
    }

    // Set base_link -> map Transform using Pose estimate // 

    geometry_msgs::TransformStamped tf_base_map;
    tf_base_map.child_frame_id = "base_link"; // Source Frame: "base_link"
    tf_base_map.header.frame_id = "map"; // Target Frame: "map"
    tf_base_map.header.stamp = ros::Time::now();

    tf_base_map.transform.rotation = pose.orientation;
    tf_base_map.transform.translation.x = pose.position.x;
    tf_base_map.transform.translation.y = pose.position.y;
    tf_base_map.transform.translation.z = pose.position.z;

    // Convert to tf2::Transforms //

    tf2::Stamped<tf2::Transform> tf2_odom_base;
    tf2::fromMsg(tf_odom_base, tf2_odom_base);
    tf2::Stamped<tf2::Transform> tf2_base_map;
    tf2::fromMsg(tf_base_map, tf2_base_map);

    // Combine Transforms using Multiplication (order important) //

    tf2::Transform tf2_odom_map = tf2_base_map * tf2_odom_base;

    // Set Message and Broadcast //

    geometry_msgs::Transform tf_odom_map = tf2::toMsg(tf2_odom_map);

    geometry_msgs::TransformStamped tfs_msg;
    tfs_msg.child_frame_id = "odom_combined"; // Source Frame: "odom"
    tfs_msg.header.frame_id = "map"; // Target Frame: "map"
    tfs_msg.header.stamp = ros::Time::now();
    tfs_msg.transform = tf_odom_map;
    
    tf_br.sendTransform(tfs_msg);
}

/**
 * @brief Get the Pose object
 * 
 * @uses GLOBAL pose_array 
 * @param particle_weights - NORMALIZED
 * @param mode 
 * @return geometry_msgs::PoseWithCovarianceStamped 
 */
geometry_msgs::PoseWithCovarianceStamped getPose(std::vector<double> particle_weights, Mode mode) {

    if(mode == Mode::WEIGHTED_AVERAGE) 
    {
        // get weighted average
        double x=0, y=0, yaw=0;
        for(int i = 0; i < pose_array.poses.size(); i++) {
            x += pose_array.poses[i].position.x * particle_weights[i];
            y += pose_array.poses[i].position.y * particle_weights[i];
            double tmp_yaw = get_yaw(pose_array.poses[i].orientation);
            yaw += tmp_yaw * particle_weights[i];
        }

        geometry_msgs::PoseWithCovarianceStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.pose.position.x = x;
        pose_stamped.pose.pose.position.y = y;
        pose_stamped.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, yaw));
        return pose_stamped;

    } else //if(mode == Mode::HIGHEST_WEIGHT) 
    {
        // get highest weight index
        int highest_weight_index = 0;
        for(int i = 0; i < pose_array.poses.size(); i++) {
            if(particle_weights[i] > particle_weights[highest_weight_index]) {
                highest_weight_index = i;
            }
        }
        geometry_msgs::PoseWithCovarianceStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.pose = pose_array.poses[highest_weight_index];
        return pose_stamped;
    }
}

/**
 * @brief 
 * 
 * @param particle_weights - NORMALIZED
 */
void resampleParticles(std::vector<double>& particle_weights) {
    // Resampling //
    geometry_msgs::PoseArray new_pose_array;
    std::vector<double> new_weights;
    for(int i = 0; i < pose_array.poses.size(); i++) {
        int index = randomIndex(particle_weights);
        new_pose_array.poses.push_back(pose_array.poses[index]);
        new_weights.push_back(particle_weights[index]);
    }
    pose_array.poses = new_pose_array.poses;
    particle_weights = new_weights;
    normalizeVector(particle_weights);
}

/**
 * @brief Weight Particles With Simulated Scans
 * 
 * @uses GLOBAL pose_array
 * 
 * @param scan 
 * @return std::vector<double> 
 */
std::vector<double> weightParticlesWithSimulatedScans(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // Simulate Scan for each Particle in Pose-Array and calculate weight given the formula on the sheet //
    std::vector<double> particle_weights;
    particle_weights.resize(pose_array.poses.size());
    for(int i = 0; i < pose_array.poses.size(); i++) {
        sensor_msgs::LaserScan simulated_scan;
        simulated_scan.header.stamp = scan->header.stamp;
        simulated_scan.header.frame_id = scan->header.frame_id;
        simulated_scan.angle_min = scan->angle_min;
        simulated_scan.angle_max = scan->angle_max;
        simulated_scan.angle_increment = scan->angle_increment;
        simulated_scan.range_min = scan->range_min;
        simulated_scan.range_max = scan->range_max;
        scan_sim.simulateScan(pose_array.poses[i], simulated_scan);

        double sum = 0;
        int invalids = 0;
        for(int j = 0; j < simulated_scan.ranges.size(); j++) {
            // ignore invalid scan data
            if(simulated_scan.ranges[j] > simulated_scan.range_max || simulated_scan.ranges[j] < simulated_scan.range_min || scan->ranges[j] > scan->range_max || scan->ranges[j] < scan->range_min) {
                invalids++;
                continue;
            }

            // calculate weight for each scan point
            double dw = simulated_scan.ranges[j] - scan->ranges[j];
            dw /= sigma_weight;
            dw = dw * dw;
            dw = std::exp(dw);
            sum += dw;
        }

        // ignore scan completely if too many invalid points
        if(invalids >= simulated_scan.ranges.size() * 0.4) {
            particle_weights[i] = 0;
        } else {
            particle_weights[i] = sum / (simulated_scan.ranges.size() - invalids);
            particle_weights[i] = 1 / particle_weights[i];  // Damit gegen 0 bei starkem Unterschied und gegen 1 bei starker Ähnlichkeit
        }
    }

    return particle_weights;
}

// ### CALLBACKS ### //

void realScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &ekf_pose) {
    if(!mapSet || pose_array.header.frame_id.empty()) {
        return;
    }
    
    // Weighting

    std::vector<double> particle_weights = weightParticlesWithSimulatedScans(scan);
    
    normalizeVector(particle_weights);

    // Resampling

    if(!last_pose.header.frame_id.empty() && ( std::pow(last_pose.pose.pose.position.x - ekf_pose->pose.pose.position.x, 2) > 0.001 || std::pow(last_pose.pose.pose.position.y - ekf_pose->pose.pose.position.y, 2) > 0.001 || std::pow(last_pose.pose.pose.orientation.z - ekf_pose->pose.pose.orientation.z, 2) > 0.00001 )) {
        resampleParticles(particle_weights);
    }


    // Get Pose from the weighted Particles using HIGHEST_WEIGHT or WEIGHTED_AVERAGE mode
    geometry_msgs::PoseWithCovarianceStamped pose = getPose(particle_weights, Mode::WEIGHTED_AVERAGE);
    
    // Publish
    pose_pub.publish(pose);
    publishOdomMapTransform(pose.pose.pose);

    // Motion Updates
    motionUpdate(ekf_pose);

    last_pose = *ekf_pose;

}

/**
 * @brief Initial Pose Callback
 * 
 * @param pose 
 */
void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose) {
    ROS_INFO_STREAM("Pose: " << pose->pose.pose.position.x << ", " << pose->pose.pose.position.y << ", " << pose->pose.pose.position.z);

    pose_array = geometry_msgs::PoseArray();

    pose_array.header.stamp = pose->header.stamp;
    pose_array.header.frame_id = pose->header.frame_id;

    pose_array.poses.push_back(pose->pose.pose);

    // get random poses around the current pose with a sigma of 0.5
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0, std_deviation_dist);
    std::normal_distribution<double> rot(0, std_deviation_rot);

    for (int i = 0; i < num_particles; i++) {
        geometry_msgs::Pose pose_random;
        pose_random.position.x = pose->pose.pose.position.x + dist(gen);
        pose_random.position.y = pose->pose.pose.position.y + dist(gen);

        tf::Quaternion q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        yaw += rot(gen);

        q.setRPY(roll, pitch, yaw);
        pose_random.orientation.x = q.x();
        pose_random.orientation.y = q.y();
        pose_random.orientation.z = q.z();
        pose_random.orientation.w = q.w();

        pose_array.poses.push_back(pose_random);
    }
}

/**
 * @brief Only called once
 * 
 * @param map 
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    scan_sim.setMap(*map);
    mapSet = true;
    map_sub.shutdown();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mcl");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // Parameters
    sigma_weight = nh_p.param<double>("sigma_weight", 270.0);
    std_deviation_dist = nh_p.param<double>("std_deviation_dist", 1.3);
    std_deviation_rot = nh_p.param<double>("std_deviation_rot", 0.2);
    num_particles = nh_p.param<int>("num_particles", 800);
    
    // subscriber
    // The map topic does not really publish data like most other topics. When first accessed it publishes the map one time
    map_sub = nh.subscribe("/map", 1000, &mapCallback);

    ros::Subscriber sub = nh.subscribe("/initialpose", 1000, &poseCallback);

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 5000);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> ekf_sub(nh, "/robot_pose_ekf/odom_combined", 5000);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), scan_sub, ekf_sub);
    sync.registerCallback(boost::bind(&realScanCallback, _1, _2));

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mcl/assumed_pose", 1000);

    pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/mcl/particles", 1000);

    ros::spin();

    return 0;

}