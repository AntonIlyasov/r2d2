// #pragma once

#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "sensor_msgs/Image.h"

using namespace cv;
using namespace std;
ros::Publisher img_pub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cam_cap");
    ros::NodeHandle node;
    
    img_pub = node.advertise<sensor_msgs::Image>("color_topic", 1);

    Mat frame;

    VideoCapture cap;
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API

    cap.open(deviceID, apiID);

    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    while (ros::ok())
    {

        cap.read(frame);
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
    return 0;

}