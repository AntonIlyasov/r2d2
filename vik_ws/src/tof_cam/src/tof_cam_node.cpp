// #pragma once

#include <ros/ros.h>
#include <iostream>
#include "tof_cam.h"


int main(int argc, char **argv) {
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║           TOF_CAM is running!         ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "tof_cam");
    Tof_cam my_cam;
    while(ros::ok()){
      my_cam.nodeProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
}