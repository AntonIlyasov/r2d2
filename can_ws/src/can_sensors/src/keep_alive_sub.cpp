#include <ros/ros.h>
#include <rosbag/bag.h>
#include "std_msgs/UInt32.h"
#include "can_sensors/KeepAlive.h"
#include "can_sensors/KeepAliveBroadcast.h"

rosbag::Bag keepAlive32;
rosbag::Bag keepAlive32Broadcast;

void keepAlive32Handler(const can_sensors::KeepAlive& keepAlive32Msg){
  keepAlive32.write("keepAlive32", ros::Time::now(), keepAlive32Msg);
  std::cout << "keepAlive32request:   " << keepAlive32Msg.keepAlive32request << "\n";
  std::cout << "keepAlive32answer:    " << keepAlive32Msg.keepAlive32answer << "\n";
}

void keepAlive32BroadcastHandler(const can_sensors::KeepAliveBroadcast& keepAlive32BroadcastMsg){
  keepAlive32Broadcast.write("keepAlive32Broadcast", ros::Time::now(), keepAlive32BroadcastMsg);
  std::cout << "keepAlive32Broadcast: " << keepAlive32BroadcastMsg.keepAlive32Broadcast << "\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keep_alive_sub");
  ROS_INFO_STREAM("keep_alive_sub is ready");
  ros::NodeHandle n;

  keepAlive32.open("keepAlive32.bag",                   rosbag::bagmode::Write);
  keepAlive32Broadcast.open("keepAlive32Broadcast.bag", rosbag::bagmode::Write);

  ros::Subscriber keepAlive32Sub = n.subscribe("keepAlive32", 1000, keepAlive32Handler);
  ros::Subscriber keepAlive32BroadcastSub = n.subscribe("keepAlive32Broadcast", 1000, keepAlive32BroadcastHandler);

  ros::spin();
  keepAlive32.close();
  keepAlive32Broadcast.close();
  return 0;
}
