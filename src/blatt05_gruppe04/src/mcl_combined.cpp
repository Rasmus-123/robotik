#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/LaserScan.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mcl_helper/scan_simulator.h>

using namespace mcl_helper;
ScanSimulator scan_sim;
bool mapSet = false;

ros::Publisher pose_pub;
ros::Publisher sim_scan_pub;
ros::Subscriber map_sub;

int highest_weight_index = 0;
double sum_of_weights = 0;

// MISSING: pose_array being generated and motion update

struct euler {
    double roll, pitch, yaw;
};

// convert geometry_msgs::Quaternion to roll pitch yaw
euler convert(const geometry_msgs::Quaternion& q) {
    tf2::Quaternion rot;
    tf2::convert(q, rot);
    tf2::Matrix3x3 m(rot);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    euler e;
    e.roll = roll;
    e.pitch = pitch;
    e.yaw = yaw;
    return e;
}

enum Mode {
    WEIGHTED_AVERAGE,
    HIGHEST_WEIGHT,
};

void getPose(const geometry_msgs::PoseArray::ConstPtr &pose_array, std::vector<double> particle_weights, Mode mode) {

    if(mode == Mode::WEIGHTED_AVERAGE) {
        // ROS_INFO("WEIGHTED_AVERAGE");
        
        // get weighted average
        double x=0,y=0,z=0,roll=0,pitch=0,yaw=0;
        for(int i = 0; i < pose_array->poses.size(); i++) {
            // normalize weights
            particle_weights[i] /= sum_of_weights;
/*             ROS_INFO("particle_weights[%d] = %f", i, particle_weights[i]);
            ROS_INFO("x: %f", pose_array->poses[i].position.x);
            ROS_INFO("y: %f", pose_array->poses[i].position.y);
 */
            x += pose_array->poses[i].position.x * particle_weights[i];
            y += pose_array->poses[i].position.y * particle_weights[i];
            // z += pose_array->poses[i].position.z * particle_weights[i];
            euler e = convert(pose_array->poses[i].orientation);
            yaw += e.yaw * particle_weights[i];

/*             if(x > 100 || y > 100) {
                ROS_INFO("x: %f, y: %f", x, y);
                ros::shutdown();
                std::exit(0);
            } */
        }

        geometry_msgs::PoseWithCovarianceStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.pose.position.x = x;
        pose_stamped.pose.pose.position.y = y;
        pose_stamped.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, yaw));
        // ROS_INFO("pose_stamped: x: %f y: %f z: %f x: %f y: %f z: %f w: %f", pose_stamped.pose.pose.position.x, pose_stamped.pose.pose.position.y, pose_stamped.pose.pose.position.z, pose_stamped.pose.pose.orientation.x, pose_stamped.pose.pose.orientation.y, pose_stamped.pose.pose.orientation.z, pose_stamped.pose.pose.orientation.w);
        pose_pub.publish(pose_stamped);

    } else if(mode == Mode::HIGHEST_WEIGHT) {
        // ROS_INFO("HIGHEST_WEIGHT");
        geometry_msgs::PoseWithCovarianceStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.pose = pose_array->poses[highest_weight_index];
        pose_pub.publish(pose_stamped);


        // tmp
        sensor_msgs::LaserScan simulated_scan;
        simulated_scan.header.stamp = ros::Time::now();
        simulated_scan.header.frame_id = "laser";
        simulated_scan.angle_min = -2.3570001125335693;
        simulated_scan.angle_max = 2.3570001125335693;
        simulated_scan.angle_increment = 0.017459258437156677;
        simulated_scan.range_min = 0.05000000074505806;
        simulated_scan.range_max = 10.0;
        scan_sim.simulateScan(pose_array->poses[highest_weight_index], simulated_scan);
        sim_scan_pub.publish(simulated_scan);
        // tmp
    }
}

void realScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan, const geometry_msgs::PoseArray::ConstPtr &pose_array) {
    if(!mapSet) {
        return;
    }
    
    highest_weight_index = 0;
    sum_of_weights = 0;

    std::vector<double> particle_weights;
    particle_weights.resize(pose_array->poses.size());
    for(int i = 0; i < pose_array->poses.size(); i++) {
        sensor_msgs::LaserScan simulated_scan;
        simulated_scan.header.stamp = scan->header.stamp;
        simulated_scan.header.frame_id = scan->header.frame_id;
        simulated_scan.angle_min = scan->angle_min;
        simulated_scan.angle_max = scan->angle_max;
        simulated_scan.angle_increment = scan->angle_increment;
        simulated_scan.range_min = scan->range_min;
        simulated_scan.range_max = scan->range_max;
        scan_sim.simulateScan(pose_array->poses[i], simulated_scan);

        double sum = 0;
        int invalids = 0;
        for(int j = 0; j < simulated_scan.ranges.size(); j++) {
            // ROS_INFO("simulated_scan.ranges[%d]: %f; scan_ranges[%d]: %f", j, simulated_scan.ranges[j], j, scan->ranges[j]);
            if(simulated_scan.ranges[j] > simulated_scan.range_max || simulated_scan.ranges[j] < simulated_scan.range_min || scan->ranges[j] > scan->range_max || scan->ranges[j] < scan->range_min) {
                invalids++;
                continue;
            }

            double dw = simulated_scan.ranges[j] - scan->ranges[j];
            dw /= simulated_scan.ranges.size() * 1;
            dw = dw * dw;
            dw = std::exp(dw);
            sum += dw;

        }

        // ROS_INFO("sum: %f", sum);

        if(invalids >= simulated_scan.ranges.size() * 0.4) {
            particle_weights[i] = 0;
        } else {
            particle_weights[i] = sum / (simulated_scan.ranges.size() - invalids);
            particle_weights[i] = 1 / particle_weights[i];  // Damit gegen 0 bei starkem Unterschied und gegen 1 bei starker Ähnlichkeit
        }
        

        if(particle_weights[i] > particle_weights[highest_weight_index]) {
            highest_weight_index = i;
        }
        
        sum_of_weights += particle_weights[i];
        
/*         if(particle_weights[i] < 0.00000001) {
            ros::shutdown();
            std::exit(0);
        } */

        
    }
    getPose(pose_array, particle_weights, Mode::WEIGHTED_AVERAGE);
}




void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    scan_sim.setMap(*map);
    mapSet = true;
    map_sub.shutdown();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mcl_combined");

    ros::NodeHandle nh;

/*     if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn) ) {
        ros::console::notifyLoggerLevelsChanged();
    } */
    
    // subscriber
    map_sub = nh.subscribe("/map", 1000, &mapCallback);

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 50);
    message_filters::Subscriber<geometry_msgs::PoseArray> pose_array_sub(nh, "/mcl/poses", 50);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, geometry_msgs::PoseArray> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), scan_sub, pose_array_sub);
    sync.registerCallback(boost::bind(&realScanCallback, _1, _2));

    pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mcl/assumed_pose", 1000);

    sim_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/best_match_scan_simulated", 1000);

    ros::spin();

    return 0;

}