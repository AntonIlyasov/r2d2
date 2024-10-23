#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int32.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/asio.hpp>
#include <fstream>

// размеры пакетов данных в нормальном режиме
#define DATA_FROM_TOF_CAM_SIZE                  2 // [1b DATA][OK]    // от камеры
#define DATA_TO_VIC_TCP_RX_SIZE                 2 // [1b DATA][OK]    // к Tcp
#define DATA_FROM_VIC_TCP_RX_SIZE               1 // [1b DATA]        // от Tcp
#define DATA_TO_TOF_CAM_SIZE                    1 // [1b DATA]        // к камере

#define TO_VIC_TCP_RX_TOPIC_NAME        "fromTofCamControlTopic"      // к Tcp
#define FROM_VIC_TCP_RX_TOPIC_NAME      "toTofCamControlTopic"        // от Tcp
#define TO_TOF_CAM_TOPIC_NAME           "toTofCamTopic"               // к камере
#define FROM_TOF_CAM_TOPIC_NAME         "fromTofCamTopic"             // от камеры

#define TO_COLOR_TOPIC_NAME "toColorTopic"
#define TO_DEPTH_TOPIC_NAME "toDepthTopic"
#define TO_IR_TOPIC_NAME    "toIrTopic"

enum class TofCamControlErrorStatus {frame_fail, allOk, img_server_is_not_available, tof_cam_is_not_available};

class CamControl
{
public:

  CamControl(std::string tcp_ip_save_frame, int tcp_port_save_frame);
  void nodeProcess();

private:
  ros::NodeHandle _node;
  int tcp_port_save_frame_;
  std::string tcp_ip_save_frame_;

  // данные от VIC Tcp RX
	uint8_t dataFromVicTcpRx[DATA_FROM_VIC_TCP_RX_SIZE];                // от Tcp

  // данные при нормальном режиме работы
  uint8_t dataToVicTcpRx[DATA_TO_VIC_TCP_RX_SIZE];                    // к Tcp
  uint8_t dataToTofCam[DATA_TO_TOF_CAM_SIZE];                         // к камере
  uint8_t dataFromTofCam[DATA_FROM_TOF_CAM_SIZE];                     // от камеры

  // остальные переменные
  ros::Publisher toVicTcpRxPub;
  ros::Subscriber fromVicTcpRxSub;
  ros::Publisher toTofCamPub;
  ros::Subscriber fromTofCamSub;
  ros::Subscriber depth_sub;
  ros::Subscriber color_sub;
  ros::Subscriber ir_sub;

  cv::Mat curr_color_img;
  cv::Mat curr_depth_img;
  cv::Mat curr_ir_img;

  uint32_t time_wait_msg_from_tof_cam       = 3;

  uint32_t send_count_vic_Tcp_rx            = 0;
  uint32_t recvd_count_vic_Tcp_rx           = 0;
  uint32_t send_count_tof_cam               = 0;
  uint32_t recvd_count_tof_cam              = 0;
  uint32_t time_wait_sec                    = 3;
  uint32_t countBmp                         = 0;
  uint8_t  resvdBytesFromVicTcpRx           = 0;
  uint8_t  resvdBytesFromTofCam             = 0;
  bool getMsgFromVicTcpRx                   = false;
  bool getMsgFromTofCam                     = false;
  bool sendMsgToTofCamFlag                  = false;
  bool m_get_color                          = false;
  bool m_get_depth                          = false; 
  bool m_get_ir                             = false;

  void colorCallback(const sensor_msgs::Image::ConstPtr& msg);
  void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
  void irCallback(const sensor_msgs::Image::ConstPtr& msg);
  void from_vic_tcp_rx_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg);
  void from_tof_cam_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg);
  
  void sendMsgToTofCam();
  void sendMsgToVicTcpRx();

  void checkSaveFrame();
  void saveColorFrameMaxQuality();
  void sendToTCPColorFrameMaxQuality();
};