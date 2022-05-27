#include <ros/ros.h>

// Image subscriber
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV stuff
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

image_transport::Publisher pub;

int rows;
int columns;
std::vector<double> flatKernel;
cv::Mat matKernel;


void imageCallback(const sensor_msgs::ImageConstPtr &img) {
    
    // konvertiere empfangenes Bild in ein cv2-Bild
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // https://docs.opencv.org/3.4/d4/dbd/tutorial_filter_2d.html
    // Faltung des Bildes mit CV und dem übergebenen Kernel 
    cv::Mat filtered;
    cv::filter2D(cv_ptr->image, filtered, -1, matKernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    cv_bridge::CvImage cv_out(img->header, cv_ptr->encoding, filtered);
    
    pub.publish(cv_out.toImageMsg());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "convolve_image");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    image_transport::ImageTransport it(nh);

    // flatKernel ist der Kernel als 1D-Matrix um die Übergabe es Parameters zu vereinfachen
    nh_p.param<std::vector<double>>("kernel", flatKernel, {0,0,0,0,1,0,0,0,0});
    nh_p.param<int>("kernel_rows", rows, 3);
    nh_p.param<int>("kernel_columns", columns, 3);

    nh_p.getParam("kernel", flatKernel);
    nh_p.getParam("kernel_rows", rows);
    nh_p.getParam("kernel_columns", columns);

    // Prüfen, ob der übergebene Kernel die richtige Größe besitzt
    if(flatKernel.size() != rows * columns) {
        ROS_ERROR("invalid kernel size");
        return 0;
    }

    // Umwandlung des 1D-Arrays zu einem 2D-Kernel
    matKernel = cv::Mat(rows, columns, CV_64FC1);
    for(int i = 0; i < rows; ++i)
        for(int j = 0; j < columns; ++j)
            matKernel.at<double>(i, j) = flatKernel.at(i*columns + j);


    pub = it.advertise("image_output", 1);
    image_transport::Subscriber sub = it.subscribe("image_input", 1, imageCallback);

    ros::spin();

    return 0;
}


