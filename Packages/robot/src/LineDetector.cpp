#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "math.h"

#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

#define number_of_values 719

using namespace std;
using namespace cv;

void callback(const sensor_msgs::LaserScan& msg) {

    //create image
    int screenwidth = 1000;
    int screenheight = 1000;
    cv::Mat image;
    image.create(screenwidth, screenheight, CV_8UC1);
    for(int i=0; i<image.rows; i++){
        for(int j=0; j<image.cols; j++){
        image.at<uchar>(i,j) = (uchar)255;
        }
    }

    //convert laser_scan data to image
    float angle = -2.094395;
    for (int i = 0; i < number_of_values; ++i) {
        //calculate cartesian coordinates
        float cartesianx = (msg.ranges[i] * sin(angle)) * 100;
        float cartesiany = (msg.ranges[i] * cos(angle)) * 100;
        angle += 0.005826;

        //convert to screen coordinates
        int screenx = (int)((cartesianx + screenwidth/2));
        int screeny = (int)((-cartesiany + screenheight/2));

        if (screenx > 0 && screeny > 0) {
            image.at<uchar>(screeny, screenx) = (uchar)0;
        } else {
            cout << screenx << endl;
            cout << screeny << endl;
        }

        //ROS_INFO("colour val = %u", image.at<uchar>(screeny,screenx));
        //ROS_INFO("colour val = %u", image.at<uchar>(screeny,screenx));

    }

    //compute Hough Transform
    cv::Mat dst, cdst;
    cv::Canny(image, dst, 50, 200, 3);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );



    cout << lines.size() << endl;
    for (int i = 0; i < lines.size(); ++i){
        cout << (lines.at(i)[0])/100 << endl;
        float scaled_distance = (lines.at(i)[0])/100;
        float originalx = scaled_distance * cos(lines.at(i)[1]) - 5;
        float originaly = scaled_distance * sin(lines.at(i)[1]) - 5;

        float distance = sqrt(originalx*originalx + originaly*originaly);
        cout << "distance = " << distance << endl;

        cout << lines.at(i) << endl;
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    return 0;
}
