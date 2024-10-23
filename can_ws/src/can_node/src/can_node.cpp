#include "ros/ros.h"
#include "std_msgs/String.h"
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <map>
#define DEBUG_OFF 0
#define DEBUG_ON  1
#define DEBUG_MODE DEBUG_ON

int recvd_from_can_count  = 0;
int send_to_can_count     = 0;

#if DEBUG_MODE == DEBUG_ON
#endif

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Топики для публикации
    /**************************************CAN**************************************/  
    pub_to_can_bus = can_node.advertise<can_msgs::Frame>("sent_messages", 1000);

    pub_raw_data_profilimetr                  = can_node.advertise<can_msgs::Frame>("raw_data_profilimetr_topic", 1000);
    pub_raw_data_driver_profilimetr           = can_node.advertise<can_msgs::Frame>("raw_data_driver_profilimetr_topic", 1000);
    pub_raw_data_air_analyzer                 = can_node.advertise<can_msgs::Frame>("raw_data_air_analyzer_topic", 1000);
    pub_raw_data_cctv                         = can_node.advertise<can_msgs::Frame>("raw_data_cctv_topic", 1000);
    pub_raw_data_driver_opu                   = can_node.advertise<can_msgs::Frame>("raw_data_driver_opu_topic", 1000);
    pub_raw_data_power_opu_board              = can_node.advertise<can_msgs::Frame>("raw_data_power_opu_board_topic", 1000);
    pub_raw_data_control_manip_board          = can_node.advertise<can_msgs::Frame>("raw_data_control_manip_board_topic", 1000);
    pub_raw_data_driver_elbow_joint           = can_node.advertise<can_msgs::Frame>("raw_data_driver_elbow_joint_topic", 1000);
    pub_raw_data_drivers_unit                 = can_node.advertise<can_msgs::Frame>("raw_data_drivers_unit_topic", 1000);
    pub_raw_data_power_distribution_unit      = can_node.advertise<can_msgs::Frame>("raw_data_power_distribution_unit_topic", 1000);
    pub_raw_data_driver_beam_extension        = can_node.advertise<can_msgs::Frame>("raw_data_driver_beam_extension_topic", 1000);
    pub_raw_data_driver_wheel                 = can_node.advertise<can_msgs::Frame>("raw_data_driver_wheel_topic", 1000);
    pub_raw_data_gyroscope_module             = can_node.advertise<can_msgs::Frame>("raw_data_gyroscope_module_topic", 1000);
    pub_raw_data_driver_path_sensor_extension = can_node.advertise<can_msgs::Frame>("raw_data_driver_path_sensor_extension_topic", 1000);
    pub_raw_data_akb                          = can_node.advertise<can_msgs::Frame>("raw_data_akb_topic", 1000);
    pub_raw_data_charger                      = can_node.advertise<can_msgs::Frame>("raw_data_charger_topic", 1000);
    /*******************************************************************************/

    //Топики для подписки
    /**************************************CAN**************************************/
    sub_from_can_bus = can_node.subscribe("received_messages", 1000, &SubscribeAndPublish::callback_from_can_bus, this);

    sub_command_can_node_profilimetr                  = can_node.subscribe("command_can_node_profilimetr_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_profilimetr           = can_node.subscribe("command_can_node_driver_profilimetr_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_air_analyzer                 = can_node.subscribe("command_can_node_air_analyzer_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_cctv                         = can_node.subscribe("command_can_node_cctv_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_opu                   = can_node.subscribe("command_can_node_driver_opu_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_power_opu_board              = can_node.subscribe("command_can_node_power_opu_board_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_control_manip_board          = can_node.subscribe("command_can_node_control_manip_board_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_elbow_joint           = can_node.subscribe("command_can_node_driver_elbow_joint_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_drivers_unit                 = can_node.subscribe("command_can_node_drivers_unit_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_power_distribution_unit      = can_node.subscribe("command_can_node_power_distribution_unit_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_beam_extension        = can_node.subscribe("command_can_node_driver_beam_extension_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_wheel                 = can_node.subscribe("command_can_node_driver_wheel_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_gyroscope_module             = can_node.subscribe("command_can_node_gyroscope_module_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_driver_path_sensor_extension = can_node.subscribe("command_can_node_driver_path_sensor_extension_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_akb                          = can_node.subscribe("command_can_node_akb_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
    sub_command_can_node_charger                      = can_node.subscribe("command_can_node_charger_topic", 1000, &SubscribeAndPublish::send_frame_to_can_bus, this);
   /*******************************************************************************/

  }

private:
  ros::NodeHandle can_node;
  
  ros::Publisher pub_to_can_bus;
  ros::Publisher pub_raw_data_profilimetr;
  ros::Publisher pub_raw_data_driver_profilimetr;
  ros::Publisher pub_raw_data_air_analyzer;
  ros::Publisher pub_raw_data_cctv;
  ros::Publisher pub_raw_data_driver_opu;
  ros::Publisher pub_raw_data_power_opu_board;
  ros::Publisher pub_raw_data_control_manip_board;
  ros::Publisher pub_raw_data_driver_elbow_joint;
  ros::Publisher pub_raw_data_drivers_unit;
  ros::Publisher pub_raw_data_power_distribution_unit;
  ros::Publisher pub_raw_data_driver_beam_extension;
  ros::Publisher pub_raw_data_driver_wheel;
  ros::Publisher pub_raw_data_gyroscope_module;
  ros::Publisher pub_raw_data_driver_path_sensor_extension;
  ros::Publisher pub_raw_data_akb;
  ros::Publisher pub_raw_data_charger;

  ros::Subscriber sub_from_can_bus;
  ros::Subscriber sub_command_can_node_profilimetr;
  ros::Subscriber sub_command_can_node_driver_profilimetr;
  ros::Subscriber sub_command_can_node_air_analyzer;
  ros::Subscriber sub_command_can_node_cctv;
  ros::Subscriber sub_command_can_node_driver_opu;
  ros::Subscriber sub_command_can_node_power_opu_board;
  ros::Subscriber sub_command_can_node_control_manip_board;
  ros::Subscriber sub_command_can_node_driver_elbow_joint;
  ros::Subscriber sub_command_can_node_drivers_unit;
  ros::Subscriber sub_command_can_node_power_distribution_unit;
  ros::Subscriber sub_command_can_node_driver_beam_extension;
  ros::Subscriber sub_command_can_node_driver_wheel;
  ros::Subscriber sub_command_can_node_gyroscope_module;
  ros::Subscriber sub_command_can_node_driver_path_sensor_extension;
  ros::Subscriber sub_command_can_node_akb;
  ros::Subscriber sub_command_can_node_charger;

  uint8_t get_node_id(const can_msgs::Frame::ConstPtr& input){
    // id 0...0x7f
    return input->id & 0x7f;
  }

  void send_frame_to_topic(uint8_t node_id, const can_msgs::Frame::ConstPtr& input){
    // key = node id
    std::map<uint8_t, ros::Publisher>::iterator it;
    std::map<uint8_t, ros::Publisher> pub_dictionary{
                                                      {0x4a, pub_raw_data_profilimetr},
                                                      {0x2, pub_raw_data_driver_profilimetr}, 
                                                      {0x43, pub_raw_data_air_analyzer}, 
                                                      {0x4, pub_raw_data_cctv}, 
                                                      {0x5, pub_raw_data_driver_opu}, 
                                                      {0x6, pub_raw_data_power_opu_board}, 
                                                      {0x7, pub_raw_data_control_manip_board}, 
                                                      {0x8, pub_raw_data_driver_elbow_joint}, 
                                                      {0x9, pub_raw_data_drivers_unit}, 
                                                      {0xa, pub_raw_data_power_distribution_unit},   
                                                      {0xb, pub_raw_data_driver_beam_extension}, 
                                                      {0xc, pub_raw_data_driver_wheel}, 
                                                      {0xd, pub_raw_data_gyroscope_module}, 
                                                      {0xe, pub_raw_data_driver_path_sensor_extension}, 
                                                      {0xf, pub_raw_data_akb}, 
                                                      {0x10, pub_raw_data_charger}
                                                    };
    // widescreen broadcasting   
    if (node_id == 0){
      for (const auto& [key, value] : pub_dictionary){
        value.publish(input);
      }
      return;
    }

    // find node id
    it = pub_dictionary.find(node_id);
    // check it exists
    if (it == pub_dictionary.end()){
      #if DEBUG_MODE == DEBUG_ON
      std::cout << "ERROR PUBLISH TO TOPIC!!! NOT FIND NODE ID [" << (int)node_id << "]\n";
      #endif
      return;
    }
    // publish
    it->second.publish(input);
    #if DEBUG_MODE == DEBUG_ON
    std::cout << "SUCCESS PUBLISH TO TOPIC FROM [" << (int)node_id << "]\n";
    #endif
  }

  void send_frame_to_topic(const can_msgs::Frame::ConstPtr& input){
    if (input->id == 0x01c0 || input->id == 0x02c0){
      pub_raw_data_air_analyzer.publish(input);
    }
    if (input->id == 0x03c0 || input->id == 0x04c0){
      pub_raw_data_profilimetr.publish(input);
    }
  }
  
  void send_frame_to_can_bus(const can_msgs::Frame::ConstPtr& input)
  {
    send_to_can_count++;
    #if DEBUG_MODE == DEBUG_ON
    // std::cout << "GET MSG FROM TOPIC\n";
    #endif
    uint8_t node_id = get_node_id(input);
    pub_to_can_bus.publish(input);
    #if DEBUG_MODE == DEBUG_ON
    // std::cout << "SUCCESS PUBLISH TO [" << (int)node_id << "]\n";
    #endif
  }

  void callback_from_can_bus(const can_msgs::Frame::ConstPtr& input)
  {
    recvd_from_can_count++;
    uint8_t node_id = get_node_id(input);
    send_frame_to_topic(node_id, input);
   // send_frame_to_topic(input);
    #if DEBUG_MODE == DEBUG_ON
    std::cout << "send_to_can_count:    " << send_to_can_count << "\n";
    std::cout << "recvd_from_can_count: " << recvd_from_can_count << "\n";
    #endif
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_node");
  SubscribeAndPublish can_node_obj;
  ros::spin();
  return 0;
}
