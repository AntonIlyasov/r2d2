#pragma once
#include "OpenNI2OpenCV.h"
#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int32.h>
#include <boost/asio.hpp>

#define DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE   1  // [1b DATA]
#define DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE     2  // [1b DATA][OK]

#define TO_TOF_CAM_CONTROL_TOPIC_NAME       "fromTofCamTopic"   // от камеры
#define FROM_TOF_CAM_CONTROL_TOPIC_NAME     "toTofCamTopic"     // к камере

#define TO_COLOR_TOPIC_NAME "usb_cam/image_raw"
#define TO_DEPTH_TOPIC_NAME "toDepthTopic"
#define TO_IR_TOPIC_NAME    "toIrTopic"

enum class TofCamCmd {keepalive = 0, shutdown = 2, reset = 0xB, turnOn = 1, publishNormalQuality = 0xC,
                      publishMaxQuality = 0xD, saveMaxQuality = 4, saveDepth = 8, publishDepth = 9,
                      saveVideo = 0x0a};
enum class TofCamErrorStatus {error, allOk};

class Tof_cam{
public:
  Tof_cam();
  void nodeProcess();

private:
  OpenNIOpenCV::OpenNI2OpenCV oni;
  ros::NodeHandle nh;

  // данные при нормальном режиме рабsendToTCPColorFrameMaxQualityоты
  uint8_t dataToTofCamControl[DATA_TO_TOF_CAM_CONTROL_TOPIC_SIZE];
  uint8_t dataFromTofCamControl[DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE];

  // остальные переменные
  ros::Publisher pubColor;
  ros::Publisher pubDepth;
  ros::Publisher pubIr;
  ros::Publisher toTofCamControlPub;
  ros::Subscriber fromTofCamControlSub;

  uint32_t send_count_tof_cam_control         = 0;
  uint32_t recvd_count_tof_cam_control        = 0;
  uint32_t resvdBytesFromTofCamControl        = 0;
  bool getMsgFromTofCamControl                = false;
  bool cmd_from_tcp = false;

  struct currentState_{
    uint8_t tof_cam_cmd_current               = static_cast<uint8_t>(TofCamCmd::shutdown);
    uint8_t tof_cam_error_status              = static_cast<uint8_t>(TofCamErrorStatus::error);
  };

  currentState_ currentState;

  void process();
  
  void fromTofCamControlCallback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg);
  void sendMsgToTofCamControl();
  void getColorFrame();
  void getColorFrameMaxQuality();
  void getDepthFrame();
  void getDepthFrame16C1();
  void getIrFrame();

  void publishColorFrame();
  void publishColorFrameMaxQuality();
  void publishDepthFrame16C1();
  void publishIrFrame();

  void printColorSensorInfo();

  void shutdownTofCam();
  void resetTofCam();
  void turnOnTofCam();

  uint8_t getTofCamCmd();
  bool getTofCamErrorStatus();
};