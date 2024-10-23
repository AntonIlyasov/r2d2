#include "led_control.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "led_control_node");
    Led_control led_control;    
    return 0;
}