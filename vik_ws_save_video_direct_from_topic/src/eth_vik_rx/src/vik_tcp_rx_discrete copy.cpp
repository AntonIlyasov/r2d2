#include <iostream>
#include <ros/ros.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <std_msgs/ByteMultiArray.h>
#include "umba_crc_table.h"

using boost::asio::ip::tcp;
using boost::asio::ip::address;

#define PORT 1234

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_TCP_SIZE                     9  // [AA][BB][LEN][1b DATA][4b KeepAL][CRC]
#define DATA_TO_TCP_SIZE                       10 // [BB][AA][LEN][1b DATA][OK][4b KeepAL][CRC]
#define DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE     1  // [1b DATA]
#define DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE   2  // [1b DATA][OK]

#define TO_TOF_CAM_CONTROL_TOPIC_NAME       "toTofCamControlTopic"
#define FROM_TOF_CAM_CONTROL_TOPIC_NAME     "fromTofCamControlTopic"

class Session
{
public:
  Session(boost::asio::io_context& io_context)
  : socket_(io_context) {
    toTofCamControlPub   = node.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_CONTROL_TOPIC_NAME, 0);
    fromTofCamControlSub = node.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_CONTROL_TOPIC_NAME, 0, 
        &Session::from_tof_cam_control_callback, this);

    memset(dataFromTCP, 0, sizeof(dataFromTCP));
    memset(dataToTCP, 0, sizeof(dataToTCP));
    memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
    memset(dataFromTofCamControl, 0, sizeof(dataFromTofCamControl));
  }

  tcp::socket& socket()
  {
    return socket_;
  }

  void async_read()
  {
    socket_.async_read_some(boost::asio::buffer(dataFromTCP, DATA_FROM_TCP_SIZE),
        boost::bind(&Session::tcp_handle_receive, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

  void async_write()
  {
    boost::asio::async_write(socket_,
        boost::asio::buffer(dataToTCP, DATA_TO_TCP_SIZE),
        boost::bind(&Session::handle_write, this,
          boost::asio::placeholders::error,
          boost::asio::placeholders::bytes_transferred));
  }

private:
  ros::NodeHandle node;
  tcp::socket socket_;

  // данные от сети по протоколу TCP
	uint8_t dataFromTCP[DATA_FROM_TCP_SIZE];

  // данные при нормальном режиме работы
  uint8_t dataToTCP[DATA_TO_TCP_SIZE];
  uint8_t dataToTofCamControl[DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE];
  uint8_t dataFromTofCamControl[DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE];

  // остальные переменные
  ros::Publisher toTofCamControlPub;
  ros::Subscriber fromTofCamControlSub;
  
  uint32_t send_count_TCP                     = 0;
  uint32_t recvd_count_TCP                    = 0;
  uint32_t send_count_tof_cam_control         = 0;
  uint32_t recvd_count_tof_cam_control        = 0;
  uint32_t resvdFailCount                     = 0;
  uint32_t sendFailCount                      = 0;
  uint32_t resvdBytesFromTofCamControl        = 0;

  bool getMsgFromTCP                          = false;

  struct currentState_{
    uint8_t keepalive[4]                      = {0};
    uint8_t from_tof_cam_control_command      = 0;
    uint8_t from_tof_cam_control_OK           = 0;
  };

  currentState_ currentState;

  boost::chrono::system_clock::time_point first_tp_recv = boost::chrono::system_clock::now();
  boost::chrono::system_clock::time_point first_tp_send = boost::chrono::system_clock::now();

  // обработчик сообщений от ROS-топика с Tof Cam Control. 
  // полученные данные с ROS-топика "расфасовываются" в переменные для дальнейшей отправки по TCP
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
    
      sendMsgToTCP();
    }
  }

  /*
    Вывод времени получения данных в микросекундах
  */
  void checkGetFromTCPTime(){
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
  void checkSendToTCPTime(){
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
    Вывод в консоль полученных по TCP данных
  */
  bool printGetFromTCPData(uint32_t bytes_transferred){
    std::cout << "\n\033[1;36mRECVD FROM TCP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
    std::cout << "recvd_count_TCP = " << recvd_count_TCP << std::endl;
    if (bytes_transferred == DATA_FROM_TCP_SIZE) {
      for (int i = 0; i < bytes_transferred; i++) { 
        printf("[%u]", dataFromTCP[i]);
      }
      printf("\n");
    } else {
      memset(dataFromTCP, 0, sizeof(dataFromTCP));
      async_read();
      return false;
    }
    return true;
  }

  /*
    Вывод в консоль отправляемых по TCP данных
  */
  void printSendToTCPData(uint32_t bytes_transferred){
    std::cout << "\nsend_count_TCP = " << send_count_TCP << std::endl;
    std::cout << "\033[1;36mSEND TO TCP bytes_transferred = \033[0m" << bytes_transferred << std::endl;
    for (int i = 0; i < bytes_transferred; i++){
      printf("[%u]", dataToTCP[i]);
    }
    printf("\n");
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

  // отправка полученных данных с ROS-топикa от Tof Cam Control пользователю по протоколу TCP
  void sendMsgToTCP(){

    // формируем пакет данных для отправки по протоколу TCP
    if (resvdBytesFromTofCamControl == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
      dataToTCP[0] = 0xBB;  
      dataToTCP[1] = 0xAA;  
      dataToTCP[2] = sizeof(dataToTCP);
      dataToTCP[3] = currentState.from_tof_cam_control_command;
      dataToTCP[4] = currentState.from_tof_cam_control_OK;
      memcpy(&dataToTCP[5], currentState.keepalive, sizeof(currentState.keepalive));
      dataToTCP[sizeof(dataToTCP) - 1]  = umba_crc8_table(dataToTCP, sizeof(dataToTCP) - 1);
    }

    // отправляем пакет данных по протоколу TCP
    if (resvdBytesFromTofCamControl == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
      async_write();
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

  // проверка принятого пакета по протоколу TCP на валидность
  bool parserTCP(uint8_t* dataFromTCP){
    if (dataFromTCP[0] != 0xAA) return false;
    if (dataFromTCP[1] != 0xBB) return false;
    /* Если длина пакета не валидная, ошибка */
    if (getLen(dataFromTCP) != DATA_FROM_TCP_SIZE) {
      return false;
    }
    /* Если контрольная сумма не совпадает, приняли муссор, ошибка */
    if ((umba_crc8_table(dataFromTCP, DATA_FROM_TCP_SIZE - sizeof(uint8_t)) != getCrc8(dataFromTCP, DATA_FROM_TCP_SIZE))) {
      return false;
    }

    return true;
  }

  /*
    обработчик сообщений от сети по протоколу TCP.
    полученные данные от сети по протоколу TCP "расфасовываются" в переменные 
    для дальнейшей отправки ROS-топик на Tof Cam Control.
  */
  void tcp_handle_receive(const boost::system::error_code& error,
      size_t bytes_transferred)
  {
    if (!error)
    {
      if(!parserTCP(dataFromTCP)){
        std::cout << "\033[1;31mTCP data not valid\033[0m\n";
        memset(dataFromTCP, 0, sizeof(dataFromTCP));
        async_read();
        return;
      }
      //checkGetFromTCPTime();
      recvd_count_TCP++;
      if (!printGetFromTCPData(bytes_transferred)) return;
      getMsgFromTCP = true;
      memcpy(dataToTofCamControl,  &dataFromTCP[3], sizeof(dataToTofCamControl));
      memcpy(currentState.keepalive,    &dataFromTCP[4], sizeof(currentState.keepalive));
      sendMsgToTofCamControl();
      async_read();
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
      currentState.from_tof_cam_control_OK = 0;
      send_count_TCP++;
      printSendToTCPData(bytes_transferred);
      async_read();
    }
    else
    {
      delete this;
    }
  }
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
      new_Session->async_read();
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
      io_context.poll_one();
      ros::spinOnce();
    }
  } catch (std::exception e){
    std::cerr << "Exeption: " << e.what() << std::endl;
  }
  return 0;
}
