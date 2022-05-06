#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


ros::Publisher marker_pub;
float epsilon, epsilon_k;

struct Point {
    float x,y,z;
};

double euklidDist(Point a, Point b) {
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// Methode um Point (float) in geometry_msgs::Point (double) umzuwandeln
geometry_msgs::Point toGP(Point p) {
    geometry_msgs::Point tp;
    tp.x = p.x;
    tp.y = p.y;
    tp.z = p.z;
    return tp;
}


void markerCallback(const sensor_msgs::PointCloud2::ConstPtr &data) {
    // Marker vorbereiten
    visualization_msgs::Marker marker;
    marker.header.frame_id = data->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lines";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1;
    
    // Werte als Point interpretieren
    // geometry_msgs::Point funktioniert hier leider nicht, da dieser mit double definiert ist. Daher die toGP Methode später
    const Point* data_raw = reinterpret_cast<const Point*>(&data->data[0]);    

    int k = 0;
    int j = 0;

    while(j < data->width - 2) {
        k = j + 1;

        // Startknoten der Linie festlegen
        geometry_msgs::Point start_point = toGP(data_raw[j]);
        geometry_msgs::Point end_point;
        int points_on_line = 2;


        while(k < data->width - 1) {
            
            // Zähler von dem ersten Term berechnen
            double zaehler = euklidDist(data_raw[j], data_raw[k+1]);

            // Wenn die Punkte zu weit auseinander liegen, ziehe keine Linie mehr
            if(zaehler >= 1.5) {
                k++;
                break;
            }

            // Nenner von dem ersten Term berechnen
            double nenner = 0.0;
            for(int i = j; i <= k; i++) {
                nenner += euklidDist(data_raw[i], data_raw[i+1]);
            }


            // exp(-0.1 * (k-j) ist eine Funktion, welche für größere Werte kleiner wird
            // Überprüfe die beiden Bedingungen und ziehe nur eine Linie, wenn beide erfüllt sind
            if(zaehler / nenner >= 1 - exp(-1 * epsilon_k * (k-j))) {
                double a = euklidDist(data_raw[k-1], data_raw[k+1]);
                double b = euklidDist(data_raw[k-1], data_raw[k]) + euklidDist(data_raw[k], data_raw[k+1]);
                if(a / b >= 1 - epsilon) {
                    // end_point als potentieller Kandidaten für den Endknoten einer Linie
                    // wird ersetzt, wenn ein Punkt existiert, der auch auf die Linie passt, aber weiter weg liegt
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

        // Ziehe Linie, wenn auch min. 3 Punkte in dieser liegen
        if(points_on_line >= 3) {
            marker.points.push_back(start_point);
            marker.points.push_back(end_point);
            j--;
        }

    }
    marker_pub.publish(marker);  
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "online_line_finder");

    ros::NodeHandle n;
    ros::NodeHandle np("~");

    np.param<float>("epsilon", epsilon, 0.005);
    np.getParam("epsilon", epsilon);

    np.param<float>("epsilon_k", epsilon_k, 0.01);
    np.getParam("epsilon_k", epsilon_k);



    // publisher
    marker_pub = n.advertise<visualization_msgs::Marker>("marker_top", 1000);


    // subscriber
    ros::Subscriber sub = n.subscribe("/convert_scan_to_cloud/cloud", 1000, markerCallback);
    ros::spin();

    return 0;
}
