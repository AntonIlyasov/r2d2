#include "cam_control/cam_control.h"
#include "tcp_answers.h"

CamControl::CamControl(std::string tcp_ip_save_frame, int tcp_port_save_frame)
    : tcp_port_save_fhd_frame_(tcp_port_save_frame),
      tcp_ip_save_fhd_frame_(tcp_ip_save_frame){

  tcp_port_save_depth_frame_  = 20003;
  tcp_ip_save_depth_frame_    = tcp_ip_save_fhd_frame_;

  toVicTcpRxPub   = _node.advertise<std_msgs::ByteMultiArray>(TO_VIC_TCP_RX_TOPIC_NAME, 0);
  fromVicTcpRxSub = _node.subscribe<std_msgs::ByteMultiArray>(FROM_VIC_TCP_RX_TOPIC_NAME, 0, 
      &CamControl::from_vic_tcp_rx_callback, this);

  toTofCamPub     = _node.advertise<std_msgs::ByteMultiArray>(TO_TOF_CAM_TOPIC_NAME, 0);
  fromTofCamSub   = _node.subscribe<std_msgs::ByteMultiArray>(FROM_TOF_CAM_TOPIC_NAME, 0, 
      &CamControl::from_tof_cam_callback, this);

  color_sub       = _node.subscribe(TO_COLOR_TOPIC_NAME,  0, &CamControl::colorCallback, this);
  depth_sub       = _node.subscribe(TO_DEPTH_TOPIC_NAME,  0, &CamControl::depthCallback, this);
  ir_sub          = _node.subscribe(TO_IR_TOPIC_NAME,     0, &CamControl::irCallback,    this);

  rtsp_url = "rtsp://192.168.100.2:8554/back";
  http_url = "http://127.0.0.1/stream?topic=/usb_cam/image_raw&type=ros_compressed&type=mjpeg&quality=100";

  memset(dataFromVicTcpRx, 0, sizeof(dataFromVicTcpRx));
  memset(dataToVicTcpRx, 0, sizeof(dataToVicTcpRx));
  memset(dataToTofCam, 0, sizeof(dataToTofCam));
  memset(dataFromTofCam, 0, sizeof(dataFromTofCam));
}
std::chrono::high_resolution_clock::time_point first = std::chrono::high_resolution_clock::now();

/*Время между вызовами данной функции*/
inline void time_check() {

  std::chrono::high_resolution_clock::time_point current = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = current - first;
  first = std::chrono::high_resolution_clock::now();

  std::cout << "TIME: " << dur.count() * 1000 << " ms" << std::endl;
}
void CamControl::nodeProcess(){

  static uint32_t fail_count = 0;
  
  if (getMsgFromVicTcpRx){
    getMsgFromVicTcpRx = false;
    sendMsgToTofCam();
    checkSaveFHDFrame();
    checkSaveDepthFrame();
    checkSaveVideo();
    sendMsgToVicTcpRx();
    sendMsgToTofCamFlag = true;
    getMsgFromTofCam    = false;
    fail_count = 0;
  }
  if (getMsgFromTofCam){
    sendMsgToTofCamFlag = false;
    getMsgFromTofCam    = false;
    sendMsgToVicTcpRx();
  }
  if (sendMsgToTofCamFlag && !getMsgFromTofCam){
    fail_count++;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (fail_count/2 >= time_wait_msg_from_tof_cam * 1000){
    fail_count = 0;
    std::cout << "[TOF CAM IS NOT AVAILABLE]\n";
    sendMsgToTofCamFlag = false;
    getMsgFromTofCam    = false;
    dataToVicTcpRx[0]   = dataFromVicTcpRx[0];
    dataToVicTcpRx[1]   = static_cast<uint8_t>(TofCamControlErrorStatus::tof_cam_is_not_available);
    sendMsgToVicTcpRx();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

/*Время между вызовами данной функции*/
inline double time_check(std::chrono::high_resolution_clock::time_point first) {
  std::chrono::duration<double> dur = std::chrono::high_resolution_clock::now() - first;
  return dur.count() * 1000;    // ms
}

void CamControl::checkSaveVideo(){

  if (dataFromVicTcpRx[0] != SAVE_VIDEO_CMD) return;

  cv::VideoWriter video;

  std::chrono::high_resolution_clock::time_point first = std::chrono::high_resolution_clock::now();
  while (curr_color_img.cols != 640 || curr_color_img.rows != 480){
    std::cout << "\033[1;31mIMAGE IS NOT 640х480!\033[0m\nSIZE: "
              << curr_color_img.cols << " x " << curr_color_img.rows << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ros::spinOnce();
    if (time_check(first) > 5000.){
      std::cout << "TIME OUT CHANGE CAM ON 640х480 QUALITY\n";
      return;
    }
  }

  // Определяем параметры видеопотока
  cv::Size frame_size(curr_color_img.cols, curr_color_img.rows);
  std::cout << "\033[1;31mframe_size.width: \033[0m" << frame_size.width << std::endl;
  std::cout << "\033[1;31mframe_size.height: \033[0m" << frame_size.height << std::endl;
  if (frame_size.width == 0 || frame_size.height == 0) return;

  int fps = 30; // Количество кадров в секунду (можно изменить)

  std::string outputFileName = "output_" + std::to_string(countVideo) + ".avi";

  // Определяем кодек и создаем объект VideoWriter для записи видео
  video.open(outputFileName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, frame_size);

  // Проверяем, успешно ли создан объект VideoWriter
  if (!video.isOpened()) {
    std::cout << "\033[1;31mОшибка: Не удалось создать видеофайл для записи\033[0m" << std::endl;
    return;
  } else {
    std::cout << "\033[1;32mСоздан видеофайл для записи\033[0m" << std::endl;
  }

  std::cout << "\033[1;32mНачинается запись видео\033[0m"<< std::endl;
  countVideo++;

  ros::Rate r(30);

  while(ros::ok()){
    video << curr_color_img;
    // std::cout << "\033[1;32mПродолжается запись видео\033[0m"<< std::endl;
    ros::spinOnce();
    if(prev_cmd == SAVE_VIDEO_CMD && prev_cmd != dataFromVicTcpRx[0]){                // когда запись нужно закончить
      std::cout << "\033[1;32mЗапись видео остановлена\033[0m"<< std::endl;
      video.release();
      break;
    }
    
    r.sleep();
  }

}

void CamControl::checkSaveDepthFrame(){
  if(dataFromVicTcpRx[0] == SAVE_DEPTH_FRAME_CMD && prev_cmd != dataFromVicTcpRx[0]){
    std::chrono::high_resolution_clock::time_point first = std::chrono::high_resolution_clock::now();
    while (curr_depth_img.cols != 640 || curr_depth_img.rows != 480){
      std::cout << "\033[1;31mIMAGE IS NOT DEPTH!\033[0m\nSIZE: "
                << curr_depth_img.cols << " x " << curr_depth_img.rows << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ros::spinOnce();
      if (time_check(first) > 5000.){
        std::cout << "TIME OUT CHANGE CAM ON DEPTH\n";
        return;
      }
    }

    saveDepthFrame();
    sendToTCPDepthFrame();
  }
}

void CamControl::checkSaveFHDFrame() {
  if(dataFromVicTcpRx[0] == SAVE_FHD_FRAME_CMD && prev_cmd != dataFromVicTcpRx[0]){
    std::chrono::high_resolution_clock::time_point first = std::chrono::high_resolution_clock::now();
    while (curr_color_img.cols != 1920 || curr_color_img.rows != 1080){
      std::cout << "\033[1;31mIMAGE IS NOT FULLHD!\033[0m\nSIZE: "
                << curr_color_img.cols << " x " << curr_color_img.rows << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      ros::spinOnce();
      if (time_check(first) > 5000.){
        std::cout << "TIME OUT CHANGE CAM ON MAX QUALITY\n";
        return;
      }
    }
    saveColorFrameMaxQuality();
    sendToTCPColorFrameMaxQuality();
  }
};

void CamControl::colorCallback(const sensor_msgs::Image::ConstPtr& msg) {
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
  m_get_color = true;
  // cv::imshow("COLOR_CHL", curr_color_img);
  // cv::waitKey(3);
}

void CamControl::depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
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

  if (prev_cmd != dataToTofCam[0]){
    sendDepthFrame = false;
    sendFHDFrame   = false;
  }
  prev_cmd = dataToTofCam[0];

  std::cout << "\nrecvd_count_vic_Tcp_rx = " << recvd_count_vic_Tcp_rx << std::endl;
  std::cout << "\033[1;33mRECVD FROM TOPIC toTofCamControlTopic resvdBytesFromVicTcpRx: \033[0m";

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
  std::cout << "\033[1;33mRECVD FROM TOPIC fromTofCamTopic resvdBytesFromTofCam: \033[0m";

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
  std::cout << "\033[1;33mSEND T0 toTofCamTopic: \033[0m";

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
  std::cout << "\033[1;33mSEND T0 fromTofCamControlTopic: \033[0m";

  for (int i = 0; i < sizeof(dataToVicTcpRx); i++) {
    printf("[%u]", dataToVicTcpRx[i]);
    msg.data.push_back(dataToVicTcpRx[i]);
  }
  std::cout << "\n#######################saefsef###########################\n";
  toVicTcpRxPub.publish(msg);
}

void CamControl::saveDepthFrame(){
  if (dataFromTofCam[1] == static_cast<uint8_t>(TofCamControlErrorStatus::frame_fail)) return;
  if (curr_depth_img.cols != 640 || curr_depth_img.rows != 480) return;
  std::cout << "\033[1;33m[SAVE DEPTH IMG]\033[0m"<< std::endl;
  countBmp++;
  cv::imwrite("depthFrame" + std::to_string(countBmp) + ".bmp", curr_depth_img);
}

void CamControl::saveColorFrameMaxQuality(){
  if (dataFromTofCam[1] == static_cast<uint8_t>(TofCamControlErrorStatus::frame_fail)) return;
  if (curr_color_img.cols != 1920 || curr_color_img.rows != 1080) return;
  std::cout << "\033[1;33m[SAVE FHD IMG]\033[0m"<< std::endl;
  countBmp++;
  cv::imwrite("colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp", curr_color_img);
}

// save depth img!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// send full image!!!!!!!!!!!!!!!!!!!!!!!!!!!!

void CamControl::sendToTCPDepthFrame(){
  boost::system::error_code ec;
  boost::asio::io_context context;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(tcp_ip_save_depth_frame_), tcp_port_save_depth_frame_);    
  boost::asio::ip::tcp::socket socket(context);
  socket.connect(endpoint, ec);
  std::cout << "\033[1;33m[CONNECT]\033[0m"<< std::endl;
  if (ec || !socket.is_open()){
    std::cerr << ec.message() << "\n";
    setErrAnswer(dataToVicTcpRx[1], TOF_CMD_SAVE_DEPTH);
    // dataToVicTcpRx[1] = static_cast<uint8_t>(TofCamControlErrorStatus::img_server_is_not_available);
    return;
  }

  std::cout << "\033[1;33m[SEND DEPTH IMG]\033[0m"<< std::endl;

  curr_depth_img = (curr_depth_img.reshape(0,1)); // to make it continuous
  int  imgSize = curr_depth_img.total()*curr_depth_img.elemSize();
  socket.send(boost::asio::buffer(curr_depth_img.data, imgSize));
  setDoneAnswer(dataToVicTcpRx[1], TOF_CMD_SAVE_DEPTH, true);
  std::cout << "File sent.\n";
  sendDepthFrame = true;
}

void CamControl::sendToTCPColorFrameMaxQuality(){
  // if (dataFromTofCam[1] == static_cast<uint8_t>(TofCamControlErrorStatus::frame_fail)) return;
  // if (curr_color_img.cols != 1920 || curr_color_img.rows != 1080) return;
  boost::system::error_code ec;
  boost::asio::io_context context;
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::make_address(tcp_ip_save_fhd_frame_), tcp_port_save_fhd_frame_);    
  boost::asio::ip::tcp::socket socket(context);
  socket.connect(endpoint, ec);
  std::cout << "\033[1;33m[CONNECT]\033[0m"<< std::endl;
  if (ec || !socket.is_open()){
    std::cerr << ec.message() << "\n";
    setErrAnswer(dataToVicTcpRx[1], TOF_CMD_SAVE_FHD);
    // dataToVicTcpRx[1] = static_cast<uint8_t>(TofCamControlErrorStatus::img_server_is_not_available);
    return;
  }

  std::cout << "\033[1;33m[SEND FHD IMG]\033[0m"<< std::endl;
  std::string fileName = "colorFrameMaxQuality" + std::to_string(countBmp) + ".bmp\0\n";


  std::cout << "Preparing image...\n";
  std::ifstream input(fileName.data(), std::ios::binary);
  // assert(input.open());
  std::string buffer(std::istreambuf_iterator<char>(input), {});

  std::cout << "\033[1;33mSIZE of buffer: \033[0m" << buffer.size() << std::endl;
  socket.send(boost::asio::buffer(buffer.data(), buffer.size()));
  input.close();
  setDoneAnswer(dataToVicTcpRx[1], TOF_CMD_SAVE_FHD, true);
  std::cout << "File sent.\n";
  sendFHDFrame = true;
}