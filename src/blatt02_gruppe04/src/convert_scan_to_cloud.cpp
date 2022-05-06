#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointField.h>
#include <ros/ros.h>

#include <cmath>

ros::Publisher cloud_pub;


void cloudCallback(const sensor_msgs::LaserScan::ConstPtr &data) {
    sensor_msgs::PointCloud2 payload;
    payload.header.frame_id = data->header.frame_id;
    payload.header.stamp = data->header.stamp;


    // describe the bytes
    sensor_msgs::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0 * sizeof(float);
    field_x.datatype = sensor_msgs::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::PointField field_y;
    field_y.name = "y";
    field_y.offset = 1 * sizeof(float);
    field_y.datatype = sensor_msgs::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::PointField field_z;
    field_z.name = "z";
    field_z.offset = 2 * sizeof(float);
    field_z.datatype = sensor_msgs::PointField::FLOAT32;
    field_z.count = 1;

    payload.fields.push_back(field_x);
    payload.fields.push_back(field_y);
    payload.fields.push_back(field_z);

    payload.point_step = 3 * sizeof(float);

    payload.width = data->ranges.size();
    payload.height = 1;
    payload.row_step = payload.width * payload.point_step;
    payload.data.resize(payload.point_step * payload.width);

    // reinterpret byte memory as float memory
    float* data_raw = reinterpret_cast<float*>(&payload.data[0]);

    for(int i = 0; i < data->ranges.size(); i++) {
        float x, y;
        if(data->range_min >= data->ranges[i] || data->ranges[i] >= data->range_max) {
            continue;
        } else {
            x = data->ranges[i] * cos(data->angle_min + i * data->angle_increment);
            y = data->ranges[i] * sin(data->angle_min + i * data->angle_increment);
        }
    
        data_raw[i*3] = x;
        data_raw[i*3 + 1] = y;
        data_raw[i*3 + 2] = 0;
        
    }

    cloud_pub.publish(payload);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "convert_scan_to_cloud");

  ros::NodeHandle n;
  ros::NodeHandle np("~");

  // publisher
  cloud_pub = np.advertise<sensor_msgs::PointCloud2>("cloud", 1000);

  // subscriber
  ros::Subscriber sub = n.subscribe("scan", 1000, cloudCallback);
  ros::spin();

  return 0;
}
