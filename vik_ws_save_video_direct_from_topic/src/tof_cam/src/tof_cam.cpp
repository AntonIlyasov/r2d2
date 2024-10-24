#pragma once
#include "tof_cam.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "tcp_answers.h"


Tof_cam::Tof_cam(){
  while (oni.init() != openni::STATUS_OK){
    printf("Initializatuion failed\n");
  }
  printf("oni.init()\n");

  toTofCamControlPub   = nh.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_CONTROL_TOPIC_NAME, 100);
  fromTofCamControlSub = nh.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_CONTROL_TOPIC_NAME, 100, 
      &Tof_cam::fromTofCamControlCallback, this);

  pubColor  = nh.advertise<sensor_msgs::Image>(TO_COLOR_TOPIC_NAME, 100);
  pubDepth  = nh.advertise<sensor_msgs::Image>(TO_DEPTH_TOPIC_NAME, 100);
  pubIr     = nh.advertise<sensor_msgs::Image>(TO_IR_TOPIC_NAME,    100);

  memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
  memset(dataFromTofCamControl, 0, sizeof(dataFromTofCamControl));
}

void Tof_cam::process(){
  switch (getTofCamCmd())
  {
    case static_cast<uint8_t>(TofCamCmd::shutdown):             //0x02
      setAcceptedAnswer(dataToTofCamControl[0], static_cast<uint8_t>(TofCamCmd::shutdown), true);//!!!!
      shutdownTofCam();
      break;
    // case static_cast<uint8_t>(TofCamCmd::reset):                //0x01
    //   resetTofCam();
    //   publishColorFrame();
    //   // publishDepthFrame16C1();
    //   publishIrFrame();
    //   break;
    case static_cast<uint8_t>(TofCamCmd::turnOn):               //0x01
      setAcceptedAnswer(dataToTofCamControl[0], static_cast<uint8_t>(TofCamCmd::turnOn), true);//!!!!
      turnOnTofCam();
      publishColorFrame();
      publishDepthFrame16C1();
      publishIrFrame();
      break;
    // case static_cast<uint8_t>(TofCamCmd::publishNormalQuality): //0x03
    //   publishColorFrame();
    //   // publishDepthFrame16C1();
    //   publishIrFrame();
    //   break;
    case static_cast<uint8_t>(TofCamCmd::publishDepth): //0x08
      setAcceptedAnswer(dataToTofCamControl[1], static_cast<uint8_t>(TofCamCmd::publishDepth), true);//!!!!
      // publishColorFrame();
      publishDepthFrame16C1();
      // publishIrFrame();
      break;
    // case static_cast<uint8_t>(TofCamCmd::publishMaxQuality):    //0x04
    //   publishColorFrameMaxQuality();
    //   // publishDepthFrame16C1();
    //   publishIrFrame();
    //   break;
    case static_cast<uint8_t>(TofCamCmd::saveMaxQuality):       //0x04
      setAcceptedAnswer(dataToTofCamControl[1], static_cast<uint8_t>(TofCamCmd::saveMaxQuality), true);//!!!!!
      publishColorFrameMaxQuality();
      // publishDepthFrame16C1();
      // publishIrFrame();
      break;
    case static_cast<uint8_t>(TofCamCmd::saveDepth):            //0x08
      setAcceptedAnswer(dataToTofCamControl[1], static_cast<uint8_t>(TofCamCmd::saveDepth), true);//!!!!!
      publishDepthFrame16C1();
      break;
    case static_cast<uint8_t>(TofCamCmd::saveVideo):            //0x0a
      setAcceptedAnswer(dataToTofCamControl[1], static_cast<uint8_t>(TofCamCmd::saveVideo), true);//!!!!!
      publishColorFrame();
      break;
    case static_cast<uint8_t>(TofCamCmd::keepalive):             //0x00
      setAcceptedAnswer(dataToTofCamControl[0], static_cast<uint8_t>(TofCamCmd::shutdown), false);//!!!!
      // shutdownTofCam();
      dataToTofCamControl[0] = 0;
      dataToTofCamControl[1] = 0;
      publishColorFrame();
      break;
    default:
      std::cout << "LOH\n";
      break;
  }
}

// основной цикл программы
void Tof_cam::nodeProcess(){
  if (getMsgFromTofCamControl){
    process();
    // getTofCamErrorStatus();
    sendMsgToTofCamControl();
    getMsgFromTofCamControl     = false;
  }
  process();
}


uint8_t Tof_cam::getTofCamCmd(){
  return Tof_cam::currentState.tof_cam_cmd_current;
}

// uint8_t Tof_cam::getTofCamErrorStatus(){
//   if ((getTofCamCmd() == static_cast<uint8_t>(TofCamCmd::shutdown) && oni.getIsTurnOff()) ||
//       oni.getStatus() == openni::STATUS_OK){
//     currentState.tof_cam_error_status = static_cast<uint8_t>(TofCamErrorStatus::allOk);
//   } else {
//     currentState.tof_cam_error_status = static_cast<uint8_t>(TofCamErrorStatus::error);
//   }
//   return currentState.tof_cam_error_status;
// }

bool Tof_cam::getTofCamErrorStatus(){
  if ((getTofCamCmd() == static_cast<uint8_t>(TofCamCmd::shutdown) && oni.getIsTurnOff()) ||
      oni.getStatus() == openni::STATUS_OK){
    currentState.tof_cam_error_status = static_cast<uint8_t>(TofCamErrorStatus::allOk);
    return true;
  } else {
    currentState.tof_cam_error_status = static_cast<uint8_t>(TofCamErrorStatus::error);
    return false;
  }
}

void Tof_cam::fromTofCamControlCallback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
  std::cout << "\n\033[1;34mCALLBACK FROM TOFCamControl\n\033[0m";
  getMsgFromTofCamControl = true;
  recvd_count_tof_cam_control++;
  resvdBytesFromTofCamControl = recvdMsg->data.size();

  std::cout << "\n\033[1;34mRECVD FROM TOPIC toTofCamTopic resvdBytesFromTofCamControl = \033[0m" 
      << resvdBytesFromTofCamControl << std::endl;
  std::cout << "recvd_count_tof_cam_control = " << recvd_count_tof_cam_control << std::endl;

  if (recvdMsg->data.size() == DATA_FROM_TOF_CAM_CONTROL_TOPIC_SIZE){
    for (int i = 0; i < recvdMsg->data.size(); i++){
      dataFromTofCamControl[i] = recvdMsg->data[i];
      printf("[%u]", dataFromTofCamControl[i]);
    }
    std::cout << std::endl;
  }
  cmd_from_tcp = true;
  currentState.tof_cam_cmd_current = dataFromTofCamControl[0];
  memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
}

void Tof_cam::sendMsgToTofCamControl(){

  //формирование пакета в топик "fromTofCamTopic"
  // dataToTofCamControl[0] = getTofCamCmd();
  // dataToTofCamControl[1] = getTofCamErrorStatus();

  //отправка пакета в топик "fromTofCamTopic"
  std_msgs::ByteMultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = sizeof(dataToTofCamControl);
  msg.data.clear();

  send_count_tof_cam_control++;
  std::cout << "send_count_tof_cam_control = " << send_count_tof_cam_control << std::endl;
  std::cout << "\033[1;34mSEND T0 fromTofCamTopic: \033[0m";

  for (int i = 0; i < sizeof(dataToTofCamControl); i++) {
    printf("[%u]", dataToTofCamControl[i]);
    msg.data.push_back(dataToTofCamControl[i]);
  }
  std::cout << std::endl;
  
  toTofCamControlPub.publish(msg);
  memset(dataToTofCamControl, 0, sizeof(dataToTofCamControl));
}

void Tof_cam::getColorFrame(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrame;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1280_720_RGB888_30FPS);
  // std::cout << "getColorFrame:            " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrame);
  std::cout << colorFrame.size() << "\n";
  if(!colorFrame.empty()) cv::imshow("Color", colorFrame);
}

void Tof_cam::getColorFrameMaxQuality(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrameMaxQuality;  
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1920_1080_RGB888_15FPS);
  // std::cout << "getColorFrameMaxQuality:  " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);
  std::cout << colorFrameMaxQuality.size() << "\n";
  if(!colorFrameMaxQuality.empty()) cv::imshow("colorFrameMaxQuality", colorFrameMaxQuality);
}

void Tof_cam::getDepthFrame(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame;
  oni.getDepthFrame(depthFrame);
  if(!depthFrame.empty()) cv::imshow("depthFrame", depthFrame);
}

void Tof_cam::getDepthFrame16C1(){
  if (!oni.m_depthStreamIsValid()) return;
  cv::Mat depthFrame16C1;
  oni.getDepthFrame16C1(depthFrame16C1);
  if(!depthFrame16C1.empty()) cv::imshow("depthFrame16C1", depthFrame16C1);
}

void Tof_cam::getIrFrame(){
  if (!oni.m_irStreamIsValid()) return;
  cv::Mat irFrame;
  oni.getIrFrame(irFrame);
  if(!irFrame.empty())    cv::imshow("IR",    irFrame);
}

void Tof_cam::printColorSensorInfo(){
  oni.printColorSensorInfo();
}

void Tof_cam::publishColorFrame(){
  if (!oni.m_colorStreamIsValid()) return;
  cv::Mat colorFrame;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1280_720_RGB888_30FPS);
  // std::cout << "publishColorFrame: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrame);

  if(!colorFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = colorFrame;
    pubColor.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::publishColorFrameMaxQuality(){
  if (!oni.m_colorStreamIsValid()) {
    // std::cout << "\033[1;31m ERR ANSWER ColorFrameMaxQ\n\033[0m ";
    setErrAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishMaxQuality));
    return;
  }
  cv::Mat colorFrameMaxQuality;
  oni.setColorVideoMode(OpenNIOpenCV::COLOR_1920_1080_RGB888_15FPS);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // std::cout << "publishColorFrameMaxQuality: " << oni.getColorResolutionX() << "x" << oni.getColorResolutionY()<< ": " << oni.getColorPixelFormat() << ": " << oni.getColorFps() << std::endl;
  oni.getColorFrame(colorFrameMaxQuality);

  if(!colorFrameMaxQuality.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = colorFrameMaxQuality;
    pubColor.publish(out_msg.toImageMsg());
    // std::cout << "\033[1;32m DONE publishMaxQuality\n\033[0m ";
    // setDoneAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishMaxQuality), true);
  } else {
    setDoneAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishMaxQuality), false);
    std::cout << "\033[1;31m NOT DONE publishMaxQuality\n\033[0m ";
  }
}

void Tof_cam::publishDepthFrame16C1(){
  if (!oni.m_depthStreamIsValid()) {
    if (cmd_from_tcp) {
      std::cout << "\033[1;31m ERR publishDepth\n\033[0m ";
      cmd_from_tcp = false;
    }
    setErrAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishDepth));
    return;
  }
  cv::Mat depthFrame16C1;
  oni.getDepthFrame16C1(depthFrame16C1);

  if(!depthFrame16C1.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO16;
    out_msg.image = depthFrame16C1;
    pubDepth.publish(out_msg.toImageMsg());
    // std::cout << "\033[1;32m DONE publishDepth\n\033[0m ";
    setDoneAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishDepth), true);
  } else {
    std::cout << "\033[1;31m NOT DONE publishDepth\n\033[0m ";
    setDoneAnswer(dataToTofCamControl[1],static_cast<uint8_t>(TofCamCmd::publishDepth), false);
  }
}

void Tof_cam::publishIrFrame(){
  if (!oni.m_irStreamIsValid()) return;
  cv::Mat irFrame;
  oni.getIrFrame(irFrame);

  if(!irFrame.empty()){
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time();
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = irFrame;
    pubIr.publish(out_msg.toImageMsg());
  }
}

void Tof_cam::shutdownTofCam(){
  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
    std::cout << "\033[1;32m DONE shutdown\n\033[0m ";
    setDoneAnswer(dataToTofCamControl[0],static_cast<uint8_t>(TofCamCmd::shutdown), true);
  } else {
    if (cmd_from_tcp) {
      std::cout << "\033[1;31m NOT DONE shutdown\n\033[0m ";
      cmd_from_tcp = false;
    }
    setDoneAnswer(dataToTofCamControl[0],static_cast<uint8_t>(TofCamCmd::shutdown), true);
  }
}

void Tof_cam::resetTofCam(){

  if (oni.getIsReset() || !getMsgFromTofCamControl){
    return;
  }

  if (oni.getIsTurnOn()){
    std::cout << "\033[1;31m shutdown TofCam\n \033[0m\n";
    oni.~OpenNI2OpenCV();
  }

  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      printf("Initializatuion failed");
    }
    printf("oni.init()\n");
  }

  oni.setIsReset(true);
}

void Tof_cam::turnOnTofCam(){
  if (oni.getIsTurnOff()){
    std::cout << "\033[1;32m turn on TofCam\n \033[0m\n";
    if (oni.init() != openni::STATUS_OK){
      std::cout << "\033[1;31m ERR turn on TofCam\n\033[0m ";
      setErrAnswer(dataToTofCamControl[0],static_cast<uint8_t>(TofCamCmd::turnOn));
      printf("Initializatuion failed");
      return;
    }
    std::cout << "\033[1;32m DONE turn on TofCam\n\033[0m ";
    setDoneAnswer(dataToTofCamControl[0],static_cast<uint8_t>(TofCamCmd::turnOn), true);
    printf("oni.init()\n");
  } else {
    if (cmd_from_tcp) {
      std::cout << "\033[1;31m NOT DONE turn on TofCam\n\033[0m ";
      cmd_from_tcp = false;
    }
    setDoneAnswer(dataToTofCamControl[0],static_cast<uint8_t>(TofCamCmd::turnOn), true);
  }
}