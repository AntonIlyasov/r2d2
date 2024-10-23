#include <iostream>
#include <ros/ros.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "umba_crc_table.h"

using boost::asio::ip::udp;
using boost::asio::ip::address;

#define PORT 1234

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_UDP_SIZE                     9  // [AA][BB][LEN][1b DATA][4b KeepAL][CRC]
#define DATA_TO_UDP_SIZE                       10 // [BB][AA][LEN][1b DATA][OK][4b KeepAL][CRC]
#define DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE     1  // [1b DATA]
#define DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE   2  // [1b DATA][OK]

#define TO_TOF_CAM_CONTROL_TOPIC_NAME       "toTofCamControlTopic"
#define FROM_TOF_CAM_CONTROL_TOPIC_NAME     "fromTofCamControlTopic"

class UDPServer{
public:
  UDPServer(boost::asio::io_service& io_service)
  : socket_(io_service, udp::endpoint(udp::v4(), PORT)){
    toTofCamControlPub   = node.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_CONTROL_TOPIC_NAME, 0);
    fromTofCamControlSub = node.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_CONTROL_TOPIC_NAME, 0, 
        &UDPServer::from_tof_cam_control_callback, this);

    memset(dataFromUDP, 0, sizeof(dataFromUDP));
    memset(dataToUDP, 0, sizeof(dataToUDP));
    memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
    memset(dataFromTofCamControl, 0, sizeof(dataFromTofCamControl));

    boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred);
    read_msg_udp();
  }

  // основной цикл программы
  void nodeProcess(){
    if (getMsgFromUDP){
      getMsgFromUDP = false;
      sendMsgToTofCamControl();
      sendMsgToUDP();
    }
  }

private:
  ros::NodeHandle node;
  udp::socket socket_;
  udp::endpoint sender_endpoint_;

  // данные от сети по протоколу UDP
	uint8_t dataFromUDP[DATA_FROM_UDP_SIZE];

  // данные при нормальном режиме работы
  uint8_t dataToUDP[DATA_TO_UDP_SIZE];
  uint8_t dataToTofCamControl[DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE];
  uint8_t dataFromTofCamControl[DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE];
  
  // остальные переменные
  ros::Publisher toTofCamControlPub;
  ros::Subscriber fromTofCamControlSub;

  uint32_t send_count_udp                     = 0;
  uint32_t recvd_count_udp                    = 0;
  uint32_t send_count_tof_cam_control         = 0;
  uint32_t recvd_count_tof_cam_control        = 0;
  uint32_t resvdFailCount                     = 0;
  uint32_t sendFailCount                      = 0;
  uint32_t resvdBytesFromTofCamControl        = 0;

  bool getMsgFromUDP                          = false;

  struct currentState_{
    uint8_t keepalive[4]                      = {0};
    uint8_t from_tof_cam_control_command      = 0;
    uint8_t from_tof_cam_control_OK           = 0;
  };

  currentState_ currentState;

  boost::chrono::system_clock::time_point first_tp_recv = boost::chrono::system_clock::now();
  boost::chrono::system_clock::time_point first_tp_send = boost::chrono::system_clock::now();

  // вызов асинхронной функции "socket_.async_receive_from" для считывания данных с UDP
  void read_msg_udp(){
    socket_.async_receive_from(boost::asio::buffer(dataFromUDP, sizeof(dataFromUDP)), sender_endpoint_,
        boost::bind(&UDPServer::udp_handle_receive, this, boost::asio::placeholders::error, 
        boost::asio::placeholders::bytes_transferred));
  }

  // обработчик сообщений от ROS-топика с Tof Cam Control. 
  // полученные данные с ROS-топика "расфасовываются" в переменные для дальнейшей отправки по UDP
  void from_tof_cam_control_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg){
    
    recvd_count_tof_cam_control++;
    resvdBytesFromTofCamControl = recvdMsg->data.size();
    
    std::cout << "\n\033[1;34mRECVD FROM TOPIC fromTofCamControlTopic resvdBytesFromTofCamControl = \033[0m" 
        << resvdBytesFromTofCamControl << std::endl;
    std::cout << "recvd_count_tof_cam_control = " << recvd_count_tof_cam_control << std::endl;

    if (recvdMsg->data.size() == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
      for (int i = 0; i < recvdMsg->data.size(); i++){
        dataFromTofCamControl[i] = recvdMsg->data[i];
        printf("[%u]", dataFromTofCamControl[i]);
      }
      std::cout << std::endl;
      currentState.from_tof_cam_control_command  = dataFromTofCamControl[0];
      currentState.from_tof_cam_control_OK       = dataFromTofCamControl[1];
    }
  }

  /*
    Вывод времени получения данных в микросекундах
  */
  void checkGetFromUDPTime(){
    boost::chrono::system_clock::time_point cur_tp_recv = boost::chrono::system_clock::now();
    boost::chrono::duration<double> ex_time_recv = cur_tp_recv - first_tp_recv;
    std::cout << "\033\n[1;32mex_time_recv time: \033\n[0m" << ex_time_recv.count() * 1000000 << "\n";
    first_tp_recv = boost::chrono::system_clock::now();
    if ((ex_time_recv.count() * 1000) > 100) {
      resvdFailCount++;
      printf("\n\n\n\033[1;31mRECV ERR\nRECV ERR\nRECV ERR\nRECV ERR\n\n\n\n\033[0m");
    }
    printf("\n\033[1;31msendFailCount  = %u\033[0m\n", sendFailCount);
    printf("\n\033[1;31mresvdFailCount = %u\033[0m\n", resvdFailCount);
  }

  /*
    Вывод времени отправления данных в микросекундах
  */
  void checkSendToUDPTime(){
    boost::chrono::system_clock::time_point cur_tp_send = boost::chrono::system_clock::now();
    boost::chrono::duration<double> ex_time_send = cur_tp_send - first_tp_send;
    std::cout << "\033\n[1;32mex_time_send time: \033\n[0m" << ex_time_send.count() * 1000000 << "\n";
    first_tp_send = boost::chrono::system_clock::now();
    if ((ex_time_send.count() * 1000) > 100) {
      sendFailCount++;
      printf("\n\n\n\033[1;31mSEND ERR\nSEND ERR\nSEND ERR\nSEND ERR\n\n\n\n\033[0m");
    }
    printf("\n\033[1;31msendFailCount  = %u\033[0m\n", sendFailCount);
    printf("\n\033[1;31mresvdFailCount = %u\033[0m\n", resvdFailCount);
  }

  /*
    Вывод в консоль полученных по UDP данных
  */
  bool printGetFromUDPData(uint32_t bytes_transferred){
    std::cout << "\n\033[1;36mRECVD FROM UDP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
    std::cout << "recvd_count_udp = " << recvd_count_udp << std::endl;
    if (bytes_transferred == DATA_FROM_UDP_SIZE) {
      for (int i = 0; i < bytes_transferred; i++) { 
        printf("[%u]", dataFromUDP[i]);
      }
      printf("\n");
    } else {
      memset(dataFromUDP, 0, sizeof(dataFromUDP));
      read_msg_udp();
      return false;
    }
    return true;
  }

  /*
    Вывод в консоль отправляемых по UDP данных
  */
  void printSendToUDPData(uint32_t bytes_transferred){
    std::cout << "\nsend_count_udp = " << send_count_udp << std::endl;
    std::cout << "\033[1;36mSEND TO UDP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
    for (int i = 0; i < bytes_transferred; i++){
      printf("[%u]", dataToUDP[i]);
    }
    printf("\n");
  }

  /*
    обработчик сообщений от сети по протоколу UDP.
    полученные данные от сети по протоколу UDP "расфасовываются" в переменные 
    для дальнейшей отправки ROS-топик на Tof Cam Control.
  */
  void udp_handle_receive(const boost::system::error_code& error, size_t bytes_transferred) {
    if (error) {
      std::cout << "Receive failed: " << error.message() << "\n";
      return;
    }
    if(!parserUDP(dataFromUDP)){
      std::cout << "\033[1;31mUDP data not valid\033[0m\n";
      memset(dataFromUDP, 0, sizeof(dataFromUDP));
      read_msg_udp();
      return;
    }
    //checkGetFromUDPTime();
    recvd_count_udp++;
    if (!printGetFromUDPData(bytes_transferred)) return;
    getMsgFromUDP = true;
    memcpy(dataToTofCamControl,  &dataFromUDP[3], sizeof(dataToTofCamControl));
    memcpy(currentState.keepalive,    &dataFromUDP[4], sizeof(currentState.keepalive));

    read_msg_udp();
  }

  void sendMsgToTofCamControl(){
    //отправка пакета в топик "toTofCamControlTopic"
    std_msgs::ByteMultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 1;
    msg.layout.dim[0].stride = sizeof(dataToTofCamControl);
    msg.data.clear();

    send_count_tof_cam_control++;
    std::cout << "\nsend_count_tof_cam_control = " << send_count_tof_cam_control << std::endl;
    std::cout << "\033[1;34mSEND T0 toTofCamControlTopic: \033[0m";

    for (int i = 0; i < sizeof(dataToTofCamControl); i++) {
      printf("[%u]", dataToTofCamControl[i]);
      msg.data.push_back(dataToTofCamControl[i]);
    }
    std::cout << std::endl;
    
    toTofCamControlPub.publish(msg);
  }

  // отправка полученных данных с ROS-топикa от Tof Cam Control пользователю по протоколу UDP
  void sendMsgToUDP(){

    // формируем пакет данных для отправки по протоколу UDP
    if (resvdBytesFromTofCamControl == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
      dataToUDP[0] = 0xBB;  
      dataToUDP[1] = 0xAA;  
      dataToUDP[2] = sizeof(dataToUDP);
      dataToUDP[3] = currentState.from_tof_cam_control_command;
      dataToUDP[4] = currentState.from_tof_cam_control_OK;
      memcpy(&dataToUDP[5], currentState.keepalive, sizeof(currentState.keepalive));
      dataToUDP[sizeof(dataToUDP) - 1]  = umba_crc8_table(dataToUDP, sizeof(dataToUDP) - 1);
    }

    // отправляем пакет данных по протоколу UDP
    if (resvdBytesFromTofCamControl == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
      boost::system::error_code error;
      auto sent = socket_.send_to(boost::asio::buffer(dataToUDP), sender_endpoint_, 0, error);
      if (!error && sent > 0){
        // checkSendToUDPTime();
        currentState.from_tof_cam_control_OK = 0;
        send_count_udp++;
        printSendToUDPData(sent);
      }
    }
    resvdBytesFromTofCamControl = 0;

  }

  // возвращает длину пакета
  static inline uint8_t getLen(uint8_t* ptrBuff)
  {
    return ptrBuff[2];
  }

  // возвращает контрольную сумму пакета
  static inline uint8_t getCrc8(uint8_t* ptrBuff, uint32_t len)
  {
    return ptrBuff[len - sizeof(uint8_t)];
  }

  // проверка принятого пакета по протоколу UDP на валидность
  bool parserUDP(uint8_t* dataFromUDP){
    if (dataFromUDP[0] != 0xAA) return false;
    if (dataFromUDP[1] != 0xBB) return false;
    /* Если длина пакета не валидная, ошибка */
    if (getLen(dataFromUDP) != DATA_FROM_UDP_SIZE) {
      return false;
    }
    /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
    if ((umba_crc8_table(dataFromUDP, DATA_FROM_UDP_SIZE - sizeof(uint8_t)) != getCrc8(dataFromUDP, DATA_FROM_UDP_SIZE))) {
      return false;
    }

    return true;
  }

};

class Session
{
public:
  Session(boost::asio::io_context& io_context)
  : socket_(io_context) {
    recvd_count = 0;
    send_count  = 0;
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void start()
  {
    socket_.async_read_some(boost::asio::buffer(data_, max_length),
        boost::bind(&Session::handle_read, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
  uint32_t recvd_count;
  uint32_t send_count;

  void handle_read(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
      
    }
    else
    {
      delete this;
    }
  }

  void handle_write(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error && bytes_transferred > 0)
    {
      std::cout << "\nSEND TO TCP: ";
      for (int i = 0; i < bytes_transferred; i++){
          printf("[%u]", data_[i]);
      }
      std::cout << std::endl;
      send_count++;
      std::cout << "\033[1;32msend_count\033[0m = " << send_count << "\n";
      socket_.async_read_some(boost::asio::buffer(data_, max_length),
          boost::bind(&Session::handle_read, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred));
    }
    else
    {
      delete this;
    }
  }

  tcp::socket socket_;
  enum { max_length = 1024 };
  uint8_t data_[max_length];
};

class TCPServer{
public:
  TCPServer(boost::asio::io_context& io_context)
  : io_context(io_context),
    acceptor(io_context, tcp::endpoint(tcp::v4(), PORT))
  {
    std::cout << "TCP SERVER IS RUNNING\n";
    async_accept();
  }
private:
  boost::asio::io_context& io_context;
  tcp::acceptor acceptor;

  void async_accept()
  {
    Session* new_Session = new Session(io_context);
    acceptor.async_accept(new_Session->socket(),
        boost::bind(&TCPServer::handle_accept, this, new_Session,
        boost::asio::placeholders::error));
  }

  void handle_accept(Session* new_Session,
      const boost::system::error_code& error){
    if (!error){
      new_Session->start();
    }
    else {
      delete new_Session;
    }
    async_accept();
  }
};

int main(int argc, char* argv[])
{
  try{
    std::cout << "\n\033[1;32m╔═══════════════════════════════╗\033[0m"
              << "\n\033[1;32m║     vik_tcp_rx is running!    ║\033[0m" 
              << "\n\033[1;32m╚═══════════════════════════════╝\033[0m\n";
    ros::init(argc, argv, "vik_tcp_rx");

    boost::asio::io_context io_context;
    TCPServer tcpServer(io_context);

    while(ros::ok()){
      udpServer.nodeProcess();  // ??
      io_context.poll_one();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "Exeption: " << e.what() << std::endl;
  }
  return 0;
}
