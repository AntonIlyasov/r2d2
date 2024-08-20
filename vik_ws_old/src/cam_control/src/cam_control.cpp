#include "cam_control/cam_control.h"

CamControl::CamControl(std::string tcp_ip_save_frame, int tcp_port_save_frame)
    : tcp_port_save_frame_(tcp_port_save_frame),
      tcp_ip_save_frame_(tcp_ip_save_frame){

  toVicTcpRxPub   = _node.advertise<std_msgs::ByteMultiArray>(TO_VIC_TCP_RX_TOPIC_NAME, 0);
  fromVicTcpRxSub = _node.subscribe<std_msgs::ByteMultiArray>(FROM_VIC_TCP_RX_TOPIC_NAME, 0, 
      &CamControl::from_vic_tcp_rx_callback, this);

  toTofCamPub     = _node.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_TOPIC_NAME, 0);
  fromTofCamSub   = _node.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_TOPIC_NAME, 0, 
      &CamControl::from_tof_cam_callback, this);

  color_sub       = _node.subscribe(TO_COLOR_TOPIC_NAME,  0, &CamControl::colorCallback, this);
  depth_sub       = _node.subscribe(TO_DEPTH_TOPIC_NAME,  0, &CamControl::depthCallback, this);
  ir_sub          = _node.subscribe(TO_IR_TOPIC_NAME,     0, &CamControl::irCallback,    this);

  memset(dataFromVicTcpRx, 0, sizeof(dataFromVicTcpRx));
  memset(dataToVicTcpRx, 0, sizeof(dataToVicTcpRx));
  memset(dataToTofCam, 0, sizeof(dataToTofCam));
  memset(dataFromTofCam, 0, sizeof(dataFromTofCam));
}

void CamControl::nodeProcess(){

  static uint32_t fail_count = 0;
  
  if (getMsgFromVicTcpRx){
    getMsgFromVicTcpRx = false;
    sendMsgToTofCam();
    sendMsgToTofCamFlag = true;
    getMsgFromTofCam    = false;
    fail_count = 0;
  }
  if (getMsgFromTofCam){
    sendMsgToTofCamFlag = false;
    getMsgFromTofCam    = false;
    checkSaveFrame();
    sendMsgToVicTcpRx();
  }
  if (sendMsgToTofCamFlag && !getMsgFromTofCam){
    fail_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (fail_count >= time_wait_msg_from_tof_cam * 1000){
    fail_count = 0;
    std::cout << "[TOF CAM IS NOT AVAILABLE]\n";
    sendMsgToTofCamFlag = false;
    getMsgFromTofCam    = false;
    dataToVicTcpRx[0]   = dataFromVicTcpRx[0];
    dataToVicTcpRx[1]   = static_cast<uint8_t>(TofCamControlErrorStatus::tof_cam_is_not_available);
    sendMsgToVicTcpRx();
  }
}

void CamControl::checkSaveFrame() {
  if(dataFromVicTcpRx[0] == 5){
    if (curr_color_img.cols != 1920 || curr_color_img.rows != 1080){
      std::this_thread::sleep_for(std::chrono::milliseconds(time_wait_sec * 1000));
      ros::spinOnce();
    }
    saveColorFrameMaxQuality();
    sendToTCPColorFrameMaxQuality();
  }
};

void CamControl::colorCallback(const sensor_msgs::Image::ConstPtr& msg) {
  m_get_color = true;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  curr_color_img = cv_ptr->image;
  // cv::imshow("COLOR_CHL", curr_color_img);
  // cv::waitKey(3);
}

void CamControl::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
  m_get_depth = true;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  curr_depth_img = cv_ptr->image;
  m_get_depth = true;
  // cv::imshow("DEPTH_CHL", curr_depth_img);
  // cv::waitKey(3);
}

void CamControl::irCallback(const sensor_msgs::Image::ConstPtr& msg) {
  m_get_ir = true;
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  curr_ir_img = cv_ptr->image;
  m_get_ir = true;
  // cv::imshow("IR_CHL", curr_ir_img);
  // cv::waitKey(3);
}

void CamControl::from_vic_tcp_rx_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
  getMsgFromVicTcpRx = true;
  recvd_count_vic_Tcp_rx++;
  resvdBytesFromVicTcpRx = recvdMsg->data.size();

  std::cout << "\nrecvd_count_vic_Tcp_rx = " << recvd_count_vic_Tcp_rx << std::endl;
  std::cout << "\033[1;34mRECVD FROM TOPIC toTofCamControlTopic resvdBytesFromVicTcpRx: \033[0m";

  if (recvdMsg->data.size() == DATA_FROM_VIC_TCP_RX_SIZE){
    for (int i = 0; i < recvdMsg->data.size(); i++){
      dataFromVicTcpRx[i] = recvdMsg->data[i];
      printf("[%u]", dataFromVicTcpRx[i]);
    }
    std::cout << std::endl;
    memcpy(dataToTofCam, dataFromVicTcpRx, sizeof(dataToTofCam));
  }
}

void CamControl::from_tof_cam_callback(const std_msgs::ByteMultiArray::ConstPtr& recvdMsg) {
  getMsgFromTofCam = true;
  recvd_count_tof_cam++;
  resvdBytesFromTofCam = recvdMsg->data.size();

  std::cout << "\nrecvd_count_tof_cam = " << recvd_count_tof_cam << std::endl;
  std::cout << "\033[1;34mRECVD FROM TOPIC fromTofCamTopic resvdBytesFromTofCam: \033[0m";

  if (recvdMsg->data.size() == DATA_FROM_TOF_CAM_SIZE){
    for (int i = 0; i < recvdMsg->data.size(); i++){
      dataFromTofCam[i] = recvdMsg->data[i];
      printf("[%u]", dataFromTofCam[i]);
    }
    std::cout << std::endl;
    memcpy(dataToVicTcpRx, dataFromTofCam, sizeof(dataToVicTcpRx));
  }
}

void CamControl::sendMsgToTofCam(){
  //отправка пакета в топик "toTofCamTopic"
  std_msgs::ByteMultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = sizeof(dataToTofCam);
  msg.data.clear();

  send_count_tof_cam++;
  std::cout << "\nsend_count_tof_cam = " << send_count_tof_cam << std::endl;
  std::cout << "\033[1;34mSEND T0 toTofCamTopic: \033[0m";

  for (int i = 0; i < sizeof(dataToTofCam); i++) {
    printf("[%u]", dataToTofCam[i]);
    msg.data.push_back(dataToTofCam[i]);
  }
  std::cout << std::endl;
  
  toTofCamPub.publish(msg);
}

void CamControl::sendMsgToVicTcpRx(){
  //отправка пакета в топик "fromTofCamControlTopic"
  std_msgs::ByteMultiArray msg;
  msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  msg.layout.dim[0].size = 1;
  msg.layout.dim[0].stride = sizeof(dataToVicTcpRx);
  msg.data.clear();

  send_count_vic_Tcp_rx++;
  std::cout << "\nsend_count_vic_Tcp_rx = " << send_count_vic_Tcp_rx << std::endl;
  std::cout << "\033[1;34mSEND T0 fromTofCamControlTopic: \033[0m";

  for (int i = 0; i < sizeof(dataToVicTcpRx); i++) {
    printf("[%u]", dataToVicTcpRx[i]);
    msg.data.push_back(dataToVicTcpRx[i]);
  }
  std::cout << "\n#######################saefsef###########################\n";
  toVicTcpRxPub.publish(msg);
}

void CamControl::saveColorFrameMaxQuality(){
  if (dataFromTofCam[1] == static_cast<uint8_t>(TofCamControlErrorStatus::frame_fail)) return;
  if (curr_color_img.cols != 1920 || curr_color_img.rows != 1080) return;
  std::cout << "[SAVE IMG]"<< std::endl;
  countBmp++;
  cv::imwrite("colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp", curr_color_img);
}

void CamControl::sendToTCPColorFrameMaxQuality(){
  if (dataFromTofCam[1] == static_cast<uint8_t>(TofCamControlErrorStatus::frame_fail)) return;
  if (curr_color_img.cols != 1920 || curr_color_img.rows != 1080) return;
  std::cout << "[SEND IMG]"<< std::endl;
  boost::system::error_code ec;
  boost::asio::io_context context;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(tcp_ip_save_frame_), tcp_port_save_frame_);    
  boost::asio::ip::tcp::socket socket(context);
  socket.connect(endpoint, ec);
  
  if (ec || !socket.is_open()){
    std::cerr << ec.message() << "\n";
    dataToVicTcpRx[1] = static_cast<uint8_t>(TofCamControlErrorStatus::img_server_is_not_available);
    return;
  }

  std::string fileName = "colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp";
  socket.send(boost::asio::buffer(fileName.data(), fileName.size()));

  // Wait for response from server
  socket.wait(socket.wait_read);
  std::size_t bytes = socket.available();
  if (bytes > 0) {
    std::string response;
    response.resize(bytes);
    socket.read_some(boost::asio::buffer(response.data(), bytes), ec);
    if (response != "OK")
    {
      std::cerr << "Unexpected server Error!\n";
    }
  }

  std::ifstream input(fileName.data(), std::ios::binary);
  std::string buffer(std::istreambuf_iterator<char>(input), {});
  socket.send(boost::asio::buffer(buffer.data(), buffer.size()));
  input.close();
  std::cout << "File sent.\n";
}