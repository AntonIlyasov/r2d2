/***********************************************************************************************************************************
Описание

код ноды модуля газовоздушного анализатора
 
Разработчик: -----------
Заметки
-------------
***********************************************************************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "can_sensors/KeepAlive.h"
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/topic_to_socketcan.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    // from mother board
    sub_command_air_analyzer = air_analyzer.subscribe("answer_command", 1000, &SubscribeAndPublish::callback_from_mother, this);
    // to can bus
    pub_command_can_node_air_analyzer = air_analyzer.advertise<can_msgs::Frame>("command_can_node_air_analyzer_topic", 1000);
    // from can bus
    sub_raw_data_air_analyzer = air_analyzer.subscribe("raw_data_air_analyzer_topic", 1000, &SubscribeAndPublish::callback_from_can_bus, this);
    // to mother board
    pub_processed_data_air_analyzer = air_analyzer.advertise<can_msgs::Frame>("answer_request", 1000);

    pubKeepAlive32 = air_analyzer.advertise<can_sensors::KeepAlive>("keepAlive32", 1000);

    keepAlive32request  = 0;
    keepAlive32answer   = 0;

    can_msgs::Frame output;

    uint32_t cob_id   = 0x243;
    boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    make_frame(output, cob_id, arr, arr.size(), false, false);
    pub_command_can_node_air_analyzer.publish(output);

    cob_id   = 0x243;
    arr = {0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00};       // 100 Hz = 0a 00 00 ...
    make_frame(output, cob_id, arr, arr.size(), false, false);
    pub_command_can_node_air_analyzer.publish(output);
  }

  void callback_from_mother(const std_msgs::String::ConstPtr& input)
  {
    //std::cout << "GET MSG FROM MOTHERBOARD\n";
    uint8_t keepalive[4] = {0};
    
    can_msgs::Frame output;

    if (input->data == "2" && !change){
      change = true;
      uint32_t cob_id = 0x243;
      boost::array<uint8_t, 8> arr = {0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_air_analyzer.publish(output);
      // std::cout << "send 0x03c0 0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00\n";
    }
    else if (input->data == "0" && change){
      change = false;
      uint32_t cob_id = 0x243;
      boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_air_analyzer.publish(output);
      // std::cout << "send 0x03c0 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00\n";
    }
  }

  void callback_from_can_bus(const can_msgs::Frame::ConstPtr& input)
  {
    keepAlive32answer = (input->data[3] << 24) + (input->data[2] << 16) + 
                        (input->data[1] << 8)  +  input->data[0];

    pub_processed_data_air_analyzer.publish(input);

    // std::cout << "keepAlive32request: " << keepAlive32request << "\n";
    // std::cout << "keepAlive32answer: " << keepAlive32answer << "\n";

    keepAlive.header.stamp       = ros::Time::now();
    keepAlive.keepAlive32request = keepAlive32request;
    keepAlive.keepAlive32answer  = keepAlive32answer;
    // pubKeepAlive32.publish(keepAlive);
  }

private:
  ros::NodeHandle air_analyzer; 
  ros::Publisher pub_processed_data_air_analyzer;
  ros::Publisher pub_command_can_node_air_analyzer;
  ros::Subscriber sub_command_air_analyzer;
  ros::Subscriber sub_raw_data_air_analyzer;
  uint32_t keepAlive32request;
  uint32_t keepAlive32answer;
  bool change = false;
  ros::Publisher pubKeepAlive32;
  can_sensors::KeepAlive keepAlive;

  void make_frame(can_msgs::Frame& frame, uint32_t cob_id, boost::array<uint8_t, 8>& data, uint8_t dlc, bool is_extended, bool is_rtr){
    frame.data          = data;   // 64 bit
    frame.dlc           = dlc;    // 4 bit
    frame.id            = cob_id; // 11 bit
    frame.header.stamp  = ros::Time::now();
    frame.is_error      = false;  // 
    frame.is_extended   = is_extended;
    frame.is_rtr        = is_rtr;
  }
};

void shutdownHandler(int sig) {
    // Корректное завершение работы
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "air_analyzer_node", ros::init_options::NoSigintHandler);
  SubscribeAndPublish air_analyzer_obj;
  std::cout << "air_analyzer_node is running\n";
  SubscribeAndPublish profilimetr_obj;
  ros::spin();
  return 0;
}