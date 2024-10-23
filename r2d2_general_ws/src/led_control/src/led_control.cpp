#include "led_control.h"

Led_control::Led_control() {
  led_eth_receiver_sub = _node.subscribe<std_msgs::ByteMultiArray>("from_led_eth_receiver", 0,
      &Led_control::led_eth_receiverCallback, this);
}

Led_control::~Led_control(){}


void Led_control::led_eth_receiverCallback(const std_msgs::ByteMultiArray::ConstPtr& recvd_msg) {
  std::cout << "\nget bytes!\n";
  std::cout << "recvd_msg->data.size() = " << recvd_msg->data.size() << std::endl;
  for (int i = 0; i < recvd_msg->data.size(); i++){
    printf("[%u]", (uint8_t)recvd_msg->data[i]);
  }
}