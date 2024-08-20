#pragma once
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_LED_SIZE                      2 // [1b DATA][OK]    // от led
#define DATA_TO_LED_TCP_RX_SIZE                 2 // [1b DATA][OK]    // к Tcp
#define DATA_FROM_LED_TCP_RX_SIZE               1 // [1b DATA]        // от Tcp
#define DATA_TO_LED_SIZE                        1 // [1b DATA]        // к led

#define TO_LED_TCP_RX_TOPIC_NAME        "fromLedControlTopic"      // к Tcp
#define FROM_LED_TCP_RX_TOPIC_NAME      "toLedControlTopic"        // от Tcp
#define TO_LED_TOPIC_NAME               "toLedTopic"               // к led
#define FROM_LED_TOPIC_NAME             "fromLedTopic"             // от led

class Led_control{
public:
  // constructors & destructors
  Led_control();
  ~Led_control();
  void nodeProcess();
  
private:
  ros::NodeHandle _node;

  // данные от led Tcp RX
  uint8_t dataFromLedTcpRx[DATA_FROM_LED_TCP_RX_SIZE];                // от Tcp

  // данные при нормальном режиме работы
  uint8_t dataToLedTcpRx[DATA_TO_LED_TCP_RX_SIZE];                    // к Tcp
  uint8_t dataToLed[DATA_TO_LED_SIZE];                                // к камере
  uint8_t dataFromLed[DATA_FROM_LED_SIZE];                            // от камеры


  ros::Subscriber led_eth_receiver_sub;
  void led_eth_receiverCallback(const std_msgs::ByteMultiArray::ConstPtr& msg);
};