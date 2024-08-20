// #pragma once

#include <iostream>
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "cam_control/cam_control.h"


int main(int argc, char** argv) {
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║         CAM_CONTROL is running!       ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "cam_control");
    int tcp_port_save_frame = 1111;
    std::string tcp_ip_save_frame = "127.0.0.1";

    ros::param::get("/_tcp_port_save_frame", tcp_port_save_frame);
    ros::param::get("/_tcp_ip_save_frame", tcp_ip_save_frame);

    CamControl myCamControl(tcp_ip_save_frame, tcp_port_save_frame);
    while(ros::ok()){
      myCamControl.nodeProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
}