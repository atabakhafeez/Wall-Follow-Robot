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

        /*cout << "srceenx" << screenx << endl;
        cout << "screeny" << screeny << endl;*/

        if (screenx > 0 && screeny > 0) {
            image.at<uchar>(screeny, screenx) = (uchar)0;
        } else {
            cout << screenx << endl;
            cout << screeny << endl;
        }

        //ROS_INFO("colour val = %u", image.at<uchar>(screeny,screenx));
        //ROS_INFO("colour val = %u", image.at<uchar>(screeny,screenx));

    }

  /*  namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);*/ 

    //cout << "image = "<< endl << " "  << image << endl << endl;

    //compute Hough Transform
    cv::Mat dst, cdst;
    cv::Canny(image, dst, 50, 200, 3);

    vector<Vec4i> lines;
    HoughLinesP( dst, lines, 1, CV_PI/180, 100, 30, 10 );

    cv::Mat gray;
    //cvtColor(image, gray, CV_BGR2GRAY);
    // smooth it, otherwise a lot of false circles may be detected
    //GaussianBlur( image, gray, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
    HoughCircles(dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 200, 100 );


    /*std::vector<cv::Vec2f> lines;
    cv::HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );*/



    cout << "number of lines = " << lines.size() << endl;
    cout << "number of circles = " << circles.size() << endl;
    /*for (int i = 0; i < lines.size(); ++i){
        //cout << (lines.at(i)[0])/100 << endl;
        float scaled_distance = (lines.at(i)[0])/100;
        float originalx = scaled_distance * cos(lines.at(i)[1]) - 5;
        float originaly = scaled_distance * sin(lines.at(i)[1]) - 5;

        float distance = sqrt(originalx*originalx + originaly*originaly);
        //cout << "distance = " << distance << endl;

        cout << lines.at(i) << endl;
    }

    for (int i = 0; i < circles.size(); ++i)
    {
        cout << circles.at(i) << endl;
    }*/



}

int main(int argc, char **argv) {
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n;

    ros::Rate loop_rate(50);
    ros::Subscriber sub = n.subscribe("base_scan", 1000, callback);
    ros::spin();
    loop_rate.sleep();
    return 0;
}
