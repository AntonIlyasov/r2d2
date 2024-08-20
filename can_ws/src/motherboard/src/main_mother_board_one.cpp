/***********************************************************************************************************************************
Описание

код главной ноды первой платы
 
Разработчик: -----------
Заметки
-------------
***********************************************************************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include "can_sensors/KeepAliveBroadcast.h"
#include "can_sensors/KeepAlive.h"

ros::Publisher pub_command_can_node_profilimetr;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {    
    main_mother_board_one.param("publish_rate_answer",publish_rate_answer,1.0);         // шлет единицу с частотой крайним параметром в Гц
    main_mother_board_one.param("publish_rate_broadcast",publish_rate_broadcast,1.0);   // шлет 2 или 0 с частотой крайним параметром в Гц
    main_mother_board_one.param("publish_value",publish_value,std::string("2"));

    timer_answer = main_mother_board_one.createTimer(ros::Duration(1.0/publish_rate_answer), &SubscribeAndPublish::callback_timer_answer,this);
    timer_broadcast = main_mother_board_one.createTimer(ros::Duration(1.0/publish_rate_broadcast), &SubscribeAndPublish::callback_timer_broadcast,this);

    pub_command_can_node_profilimetr = main_mother_board_one.advertise<can_msgs::Frame>("command_can_node_profilimetr_topic", 1000);

    //Топики для публикации
    /**************************************CAN**************************************/  

    pub_command_answer = main_mother_board_one.advertise<std_msgs::String>("answer_command", 1000);         // сюда шлет 1
    pub_command_broadcast = main_mother_board_one.advertise<std_msgs::String>("broadcast_command", 1000);   // сюда шлет 2 или 0

    pub_command_profilimetr = main_mother_board_one.advertise<std_msgs::String>("command_profilimetr_topic", 1000);
    pub_command_driver_profilimetr = main_mother_board_one.advertise<std_msgs::String>("command_driver_profilimetr_topic", 1000);
    pub_command_air_analyzer = main_mother_board_one.advertise<std_msgs::String>("command_air_analyzer_topic", 1000);
    pub_command_cctv = main_mother_board_one.advertise<std_msgs::String>("command_cctv_topic", 1000);
    pub_command_driver_opu = main_mother_board_one.advertise<std_msgs::String>("command_driver_opu_topic", 1000);
    pub_command_power_opu_board = main_mother_board_one.advertise<std_msgs::String>("command_power_opu_board_topic", 1000);
    pub_command_control_manip_board = main_mother_board_one.advertise<std_msgs::String>("command_control_manip_board_topic", 1000);
    pub_command_driver_elbow_joint = main_mother_board_one.advertise<std_msgs::String>("command_driver_elbow_joint_topic", 1000);
    pub_command_drivers_unit = main_mother_board_one.advertise<std_msgs::String>("command_drivers_unit_topic", 1000);
    pub_command_power_distribution_unit = main_mother_board_one.advertise<std_msgs::String>("command_power_distribution_unit_topic", 1000);
    pub_command_driver_beam_extension = main_mother_board_one.advertise<std_msgs::String>("command_driver_beam_extension_topic", 1000);
    pub_command_driver_wheel = main_mother_board_one.advertise<std_msgs::String>("command_driver_wheel_topic", 1000);
    pub_command_gyroscope_module = main_mother_board_one.advertise<std_msgs::String>("command_gyroscope_module_topic", 1000);
    pub_command_driver_path_sensor_extension = main_mother_board_one.advertise<std_msgs::String>("command_driver_path_sensor_extension_topic", 1000);
    pub_command_akb = main_mother_board_one.advertise<std_msgs::String>("command_akb_topic", 1000);
    pub_command_charger = main_mother_board_one.advertise<std_msgs::String>("command_charger_topic", 1000);
    pub_tx_can_boards = main_mother_board_one.advertise<std_msgs::String>("tx_can_boards_topic", 1000);
    /*******************************************************************************/


    /**************************************ETHERNET*********************************/ 
    pub_path_sensor = main_mother_board_one.advertise<std_msgs::String>("tx_path_sensor_topic", 1000);
    pub_profilimetr = main_mother_board_one.advertise<std_msgs::String>("tx_profilimetr_topic", 1000);
    pub_thickness_sensor = main_mother_board_one.advertise<std_msgs::String>("tx_thickness_sensor_topic", 1000);
    pub_tx_ethernet_boards = main_mother_board_one.advertise<std_msgs::String>("tx_ethernet_boards_topic", 1000);
    /*******************************************************************************/


    /**************************************RS-432***********************************/
    pub_command_optical_gyroscope = main_mother_board_one.advertise<std_msgs::String>("command_optical_gyroscope_topic", 1000);
    /*******************************************************************************/
    ////////////////////////////////////////////////////////////  

    //Топики для подписки
    /**************************************CAN**************************************/  

    sub_request_answer = main_mother_board_one.subscribe("answer_request", 1000, &SubscribeAndPublish::callback_answer, this);  // это от 1
    sub_request_broadcast = main_mother_board_one.subscribe("broadcast_request", 1000, &SubscribeAndPublish::callback_broadcast, this); // это от 2 или 0
    sub_param_value = main_mother_board_one.subscribe("param_value_setting", 1000, &SubscribeAndPublish::callback_value, this);  // сюда отправить 2 или 0 самому

    sub_processed_data_profilimetr = main_mother_board_one.subscribe("processed_data_profilimetr_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_profilimetr = main_mother_board_one.subscribe("processed_data_driver_profilimetr_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_air_analyzer = main_mother_board_one.subscribe("processed_data_air_analyzer_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_cctv = main_mother_board_one.subscribe("processed_data_cctv_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_opu = main_mother_board_one.subscribe("processed_data_driver_opu_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_power_opu_board = main_mother_board_one.subscribe("processed_data_power_opu_board_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_control_manip_board = main_mother_board_one.subscribe("processed_data_control_manip_board_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_elbow_joint = main_mother_board_one.subscribe("processed_data_driver_elbow_joint_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_drivers_unit = main_mother_board_one.subscribe("processed_data_drivers_unit_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_power_distribution_unit = main_mother_board_one.subscribe("processed_data_power_distribution_unit_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_beam_extension = main_mother_board_one.subscribe("processed_data_driver_beam_extension_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_wheel = main_mother_board_one.subscribe("processed_data_driver_wheel_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_gyroscope_module = main_mother_board_one.subscribe("processed_data_gyroscope_module_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_driver_path_sensor_extension = main_mother_board_one.subscribe("processed_data_driver_path_sensor_extension_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_akb = main_mother_board_one.subscribe("processed_data_akb_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_processed_data_charger = main_mother_board_one.subscribe("processed_data_charger_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_rx_can_boards = main_mother_board_one.subscribe("rx_can_boards_topic", 1000, &SubscribeAndPublish::callback, this);
    /*******************************************************************************/
    
    /**************************************ETHERNET*********************************/ 
    sub_path_sensor = main_mother_board_one.subscribe("rx_path_sensor_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_profilimetr = main_mother_board_one.subscribe("rx_profilimetr_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_thickness_sensor = main_mother_board_one.subscribe("rx_thickness_sensor_topic", 1000, &SubscribeAndPublish::callback, this);
    sub_rx_ethernet_boards = main_mother_board_one.subscribe("rx_ethernet_boards_topic", 1000, &SubscribeAndPublish::callback, this);
    /*******************************************************************************/


    /**************************************RS-432***********************************/ 
    sub_processed_data_optical_gyroscope = main_mother_board_one.subscribe("processed_data_optical_gyroscope_topic", 1000, &SubscribeAndPublish::callback, this);
    /*******************************************************************************/
    ////////////////////////////////////////////////////////////  

    pubKeepAlive32          = main_mother_board_one.advertise<can_sensors::KeepAlive>("keepAlive32", 1000);
    pubKeepAlive32Broadcast = main_mother_board_one.advertise<can_sensors::KeepAliveBroadcast>("keepAlive32Broadcast", 1000);

    keepAlive32request  = 0;
    keepAlive32answer   = 0;
    keepAlive32Broadcast = 0;

  }

//Функции обработки пришедших сообщений
  void callback(const std_msgs::String::ConstPtr& input)
  {
    /**************************************CAN**************************************/ 
    pub_command_driver_profilimetr.publish(input);
    pub_command_profilimetr.publish(input);
    pub_command_air_analyzer.publish(input);
    pub_command_cctv.publish(input);
    pub_command_driver_opu.publish(input);
    pub_command_power_opu_board.publish(input);
    pub_command_control_manip_board.publish(input);
    pub_command_driver_elbow_joint.publish(input);
    pub_command_drivers_unit.publish(input);
    pub_command_power_distribution_unit.publish(input);
    pub_command_driver_beam_extension.publish(input);
    pub_command_driver_wheel.publish(input);
    pub_command_gyroscope_module.publish(input);
    pub_command_driver_path_sensor_extension.publish(input);
    pub_command_akb.publish(input);
    pub_command_charger.publish(input);
    pub_tx_can_boards.publish(input);
    /*******************************************************************************/


    /**************************************ETHERNET*********************************/
    pub_path_sensor.publish(input);
    pub_profilimetr.publish(input);
    pub_thickness_sensor.publish(input);
    pub_tx_ethernet_boards.publish(input);
    /*******************************************************************************/


    /**************************************RS-432***********************************/ 
    pub_command_optical_gyroscope.publish(input);
    /*******************************************************************************/


  }




   void callback_answer(const can_msgs::Frame::ConstPtr& msg)
    {
      keepAlive32answer = (msg->data[3] << 24) + (msg->data[2] << 16) + 
                          (msg->data[1] << 8)  +  msg->data[0];
      // ROS_INFO("Received message from answer_request");
      std::cout << "keepAlive32request:   " << keepAlive32request         << "\n";
      std::cout << "keepAlive32answer:    " << keepAlive32answer          << "\n";
      std::cout << "keepAlive32Broadcast: " << keepAlive32Broadcast       << "\n\n";

      keepAlive.header.stamp       = ros::Time::now();
      keepAlive.keepAlive32request = keepAlive32request;
      keepAlive.keepAlive32answer  = keepAlive32answer;
      pubKeepAlive32.publish(keepAlive);
    }

    void callback_broadcast(const can_msgs::Frame::ConstPtr& msg)
    {
      keepAlive32Broadcast =  (msg->data[3] << 24) + (msg->data[2] << 16) + 
                              (msg->data[1] << 8)  +  msg->data[0];
      // ROS_INFO("Received message from broadcast_request");
      std::cout << "keepAlive32request:   " << keepAlive32request         << "\n";
      std::cout << "keepAlive32answer:    " << keepAlive32answer          << "\n";
      std::cout << "keepAlive32Broadcast: " << keepAlive32Broadcast       << "\n\n";

      keepAliveBroadcast.header.stamp         = ros::Time::now();
      keepAliveBroadcast.keepAlive32Broadcast = keepAlive32Broadcast;
      pubKeepAlive32Broadcast.publish(keepAliveBroadcast);
    }

    void callback_value(const std_msgs::String::ConstPtr& value)
    {
        
        if (value->data == "0" || value->data == "2")
        {
            publish_value = value->data;
            // ROS_INFO("Changed publish value on: %s",value->data.c_str());
        }
        else
        {
            // ROS_INFO("Invalid value, please enter 0 or 2.");
        }
    }



    void callback_timer_answer(const ros::TimerEvent&)
    {
      std_msgs::String msg;
      msg.data = "2";
      pub_command_answer.publish(msg);
      keepAlive32request++;
      std::cout << "keepAlive32request:   " << keepAlive32request         << "\n";
      std::cout << "keepAlive32answer:    " << keepAlive32answer          << "\n";
      std::cout << "keepAlive32Broadcast: " << keepAlive32Broadcast       << "\n\n";
    }

    void callback_timer_broadcast(const ros::TimerEvent&)
    {
      std_msgs::String msg;
      msg.data = publish_value;
      pub_command_broadcast.publish(msg);
      std::cout << "keepAlive32request:   " << keepAlive32request         << "\n";
      std::cout << "keepAlive32answer:    " << keepAlive32answer          << "\n";
      std::cout << "keepAlive32Broadcast: " << keepAlive32Broadcast       << "\n\n";
    }


////////////////////////////////////////////////////////////  
private:
  ros::NodeHandle main_mother_board_one; 

  /**************************************CAN**************************************/ 
  ros::Publisher pub_command_answer;
  ros::Publisher pub_command_broadcast;


  ros::Publisher pub_command_profilimetr;
  ros::Publisher pub_command_driver_profilimetr;
  ros::Publisher pub_command_air_analyzer;
  ros::Publisher pub_command_cctv;
  ros::Publisher pub_command_driver_opu;
  ros::Publisher pub_command_power_opu_board;
  ros::Publisher pub_command_control_manip_board;
  ros::Publisher pub_command_driver_elbow_joint;
  ros::Publisher pub_command_drivers_unit;
  ros::Publisher pub_command_power_distribution_unit;
  ros::Publisher pub_command_driver_beam_extension;
  ros::Publisher pub_command_driver_wheel;
  ros::Publisher pub_command_gyroscope_module;
  ros::Publisher pub_command_driver_path_sensor_extension;
  ros::Publisher pub_command_akb;
  ros::Publisher pub_command_charger;
  ros::Publisher pub_tx_can_boards;



  ros::Subscriber sub_request_answer;
  ros::Subscriber sub_request_broadcast;
  ros::Subscriber sub_param_value;


  ros::Subscriber sub_processed_data_profilimetr;
  ros::Subscriber sub_processed_data_driver_profilimetr;
  ros::Subscriber sub_processed_data_air_analyzer;
  ros::Subscriber sub_processed_data_cctv;
  ros::Subscriber sub_processed_data_driver_opu;
  ros::Subscriber sub_processed_data_power_opu_board;
  ros::Subscriber sub_processed_data_control_manip_board;
  ros::Subscriber sub_processed_data_driver_elbow_joint;
  ros::Subscriber sub_processed_data_drivers_unit;
  ros::Subscriber sub_processed_data_power_distribution_unit;
  ros::Subscriber sub_processed_data_driver_beam_extension;
  ros::Subscriber sub_processed_data_driver_wheel;
  ros::Subscriber sub_processed_data_gyroscope_module;
  ros::Subscriber sub_processed_data_driver_path_sensor_extension;
  ros::Subscriber sub_processed_data_akb;
  ros::Subscriber sub_processed_data_charger;
  ros::Subscriber sub_rx_can_boards;
  /*******************************************************************************/ 


  /**************************************ETHERNET*********************************/
  ros::Publisher pub_path_sensor;
  ros::Publisher pub_profilimetr;
  ros::Publisher pub_thickness_sensor;
  ros::Publisher pub_tx_ethernet_boards;

  ros::Subscriber sub_path_sensor;
  ros::Subscriber sub_profilimetr;
  ros::Subscriber sub_thickness_sensor;
  ros::Subscriber sub_rx_ethernet_boards;
  /*******************************************************************************/


  /**************************************RS-432***********************************/
  ros::Publisher pub_command_optical_gyroscope;

  ros::Subscriber sub_processed_data_optical_gyroscope;
  /*******************************************************************************/

  uint32_t keepAlive32request;
  uint32_t keepAlive32answer;

  ros::Publisher pubKeepAlive32;
  can_sensors::KeepAlive keepAlive;

  uint32_t keepAlive32Broadcast;
  ros::Publisher pubKeepAlive32Broadcast;
  can_sensors::KeepAliveBroadcast keepAliveBroadcast;



  std::string publish_value;
  double publish_rate_answer;
  double publish_rate_broadcast;
  ros::Timer timer_answer;
  ros::Timer timer_broadcast;

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

      uint32_t cob_id = 0x243;
      boost::array<uint8_t, 8> arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_profilimetr.publish(output);

      cob_id = 0x24a;
      arr = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
      make_frame(output, cob_id, arr, arr.size(), false, false);
      pub_command_can_node_profilimetr.publish(output);

    }

    // Ожидание для гарантированной отправки
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    system("rosnode kill /air_analyzer_node");
    system("rosnode kill /profilimetr_node");

    // Корректное завершение работы
    ros::shutdown();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "main_mother_board_one_node", ros::init_options::NoSigintHandler);
 
  
  SubscribeAndPublish mother_board_one_obj;
  signal(SIGINT, shutdownHandler);
  ros::spin();

  return 0;
}
