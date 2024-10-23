// #pragma once

#include <iostream>
#include <ros/ros.h>
#include "led_control.h"



int main(int argc, char** argv) {
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════════════╗\033[0m"
              << "\n\033[1;32m║         LED_CONTROL is running!       ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "led_control");






    Led_control led_control;
    while(ros::ok()){
      led_control.nodeProcess();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "\nExeption: " << e.what() << std::endl;
  }
  return 0;
}