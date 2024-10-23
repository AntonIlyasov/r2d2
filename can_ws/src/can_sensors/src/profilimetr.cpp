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
#include "can_sensors/KeepAliveBroadcast.h"
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/topic_to_socketcan.h>

void make_frame(can_msgs::Frame& frame, uint32_t cob_id, boost::array<uint8_t, 8>& data, uint8_t dlc, bool is_extended, bool is_rtr);
ros::Publisher pub_command_can_node_profilimetr;

class SubscribeAndPublish
{
public:

  ~SubscribeAndPublish(){}

  SubscribeAndPublish()
  {
    // from mother board
    sub_command_profilimetr = profilimetr.subscribe("broadcast_command", 1000, &SubscribeAndPublish::callback_from_mother, this);
    // to can bus
    pub_command_can_node_profilimetr = profilimetr.advertise<can_msgs::Frame>("command_can_node_profilimetr_topic", 1000);
    // from can bus
    sub_raw_data_profilimetr = profilimetr.subscribe("raw_data_profilimetr_topic", 1000, &SubscribeAndPublish::callback_from_can_bus, this);
    // to mother board
    pub_processed_data_profilimetr = profilimetr.advertise<can_msgs::Frame>("broadcast_request", 1000);

    pubKeepAlive32Broadcast  = profilimetr.advertise<can_sensors::KeepAliveBroadcast>("keepAlive32Broadcast", 1000);

    can_msgs::Frame output;

    uint32_t cob_id   = 0x24a;
    boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    make_frame(output, cob_id, arr, arr.size(), false, false);
    pub_command_can_node_profilimetr.publish(output);

    cob_id   = 0x24a;
    arr = {0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00};       // 100 Hz = 0a 00 00 ...
    make_frame(output, cob_id, arr, arr.size(), false, false);
    pub_command_can_node_profilimetr.publish(output);
    keepAlive32Broadcast = 0;

  }

  void callback_from_mother(const std_msgs::String::ConstPtr& input)
  {
    //std::cout << "GET MSG FROM MOTHERBOARD\n";
    uint8_t keepalive[4] = {0};
    
    can_msgs::Frame output;

    if (input->data == "2" && !change){
      change = true;
      uint32_t cob_id = 0x24a;
      boost::array<uint8_t, 8> arr = {0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_profilimetr.publish(output);
      // std::cout << "send 0x03c0 0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00\n";
    }
    else if (input->data == "0" && change){
      change = false;
      uint32_t cob_id = 0x24a;
      boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_profilimetr.publish(output);
      // std::cout << "send 0x03c0 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00\n";
    }
  }

  void callback_from_can_bus(const can_msgs::Frame::ConstPtr& input)
  {
    keepAlive32Broadcast =  (input->data[3] << 24) + (input->data[2] << 16) + 
                            (input->data[1] << 8)  +  input->data[0];
    pub_processed_data_profilimetr.publish(input);
    std::cout << "keepAlive32Broadcast: " << keepAlive32Broadcast << "\n";

    keepAlive.header.stamp       = ros::Time::now();
    keepAlive.keepAlive32Broadcast = keepAlive32Broadcast;
    // pubKeepAlive32Broadcast.publish(keepAlive);
  }

private:
  ros::NodeHandle profilimetr; 
  ros::Publisher pub_processed_data_profilimetr;
  
  ros::Subscriber sub_command_profilimetr;
  ros::Subscriber sub_raw_data_profilimetr;
  uint32_t keepAlive32Broadcast;
  ros::Publisher pubKeepAlive32Broadcast;
  can_sensors::KeepAliveBroadcast keepAlive;
  bool change = false;
};

void make_frame(can_msgs::Frame& frame, uint32_t cob_id, boost::array<uint8_t, 8>& data, uint8_t dlc, bool is_extended, bool is_rtr){
  frame.data          = data;
  frame.dlc           = dlc;
  frame.id            = cob_id;
  frame.header.stamp  = ros::Time::now();
  frame.is_error      = false;
  frame.is_extended   = is_extended;
  frame.is_rtr        = is_rtr;
}

void shutdownHandler(int sig) {
    // Отправка финального сообщения
    for (size_t i = 0; i < 5; i++)
    {
      // init can communication
      can_msgs::Frame output;

      uint32_t cob_id = 0x24a;
      boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_profilimetr.publish(output);

    }

    // Ожидание для гарантированной отправки
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    system("rosnode kill /air_analyzer_node");
    system("rosnode kill /main_mother_board_one");

    // Корректное завершение работы
    ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "profilimetr_node", ros::init_options::NoSigintHandler);
  std::cout << "profilimetr_node is running\n";
  SubscribeAndPublish profilimetr_obj;
  signal(SIGINT, shutdownHandler);
  ros::spin();
  return 0;
}
