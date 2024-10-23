// #include "ros/ros.h"
// #include <iostream>
// #include "OpenNI2OpenCV.h"

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "tof_cam_node");
//   ros::NodeHandle n;

//   OpenNIOpenCV::OpenNI2OpenCV oni;
//   if (oni.init() != openni::STATUS_OK){
//     printf("Initializatuion failed");
//     return 1;
//   }
//   printf("oni.init()\n");

//   cv::Mat colorFrame, depthFrame, irFrame;
//   oni.setColorVideoMode(6);


//   while (ros::ok()){
//     oni.getColorFrame(colorFrame);
//     oni.getDepthFrame16C1(depthFrame);
//     // oni.getIrFrame(irFrame);
//     if(!colorFrame.empty()) cv::imshow("Color", colorFrame);
//     if(!depthFrame.empty()) cv::imshow("Depth", depthFrame);
//     // if(!irFrame.empty())    cv::imshow("IR",    irFrame);
    
//     int framePerSecond = 30;
//     char character = cv::waitKey(1000 / framePerSecond);
//     if (character == 27) break;
//   }
  
//   openni::OpenNI::shutdown();
//   return 0;
// }

/////////////////////////////////////////////////////////////////////////////////////////
// #include "ros/ros.h"
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/ocl.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// using namespace cv;
// using namespace cv::ocl;
// using namespace std;


// int main(int argc, char **argv)
// {
  
//   if (!cv::ocl::haveOpenCL())
//   {
//     cout << "OpenCL is not available..." << endl;
//     return 0;
//   } else {
//     cout << "OpenCL is available..." << endl;
//   }

//   ros::init(argc, argv, "tof_cam_node");


//     TickMeter tm;
//     tm.start();
// if (cv::ocl::haveOpenCL())
//     {
//         cv::ocl::Context context;
//         if (!context.create(cv::ocl::Device::TYPE_GPU))
//         {
//             std::cerr << "Failed creating the OpenCL context" << std::endl;
//             return -1;
//         }
//        cv::ocl::setUseOpenCL(true);
//    }
//     cv::UMat output;
//     cv::UMat image1 = cv::imread("test1.png").getUMat(cv::ACCESS_RW);
//     cv::UMat image2 = cv::imread("test2.png").getUMat(cv::ACCESS_RW);

//     cv::multiply(image1, 0.333, output);

//     // cv::Mat img1 = cv::imread("test1.png", IMREAD_COLOR);
//     // cv::Mat img2 = cv::imread("test2.png", IMREAD_COLOR);

  
//     // if( img1.empty() ) { cout << "Error loading src1" << endl; return EXIT_FAILURE; }
//     // if( img2.empty() ) { cout << "Error loading src2" << endl; return EXIT_FAILURE; }

//     // cvtColor(img1, img1, COLOR_RGB2GRAY);
//     // cvtColor(img2, img2, COLOR_RGB2GRAY);

//     // image2 = image2.t();

//     // cv::multiply(image1, image2, output);


//     // cv::mergeImage(image1, image1, output);
//     // addWeighted( image1, 0.5, image1, 0.5, 0.0, output);
//     // addWeighted( image1, 0.5, image2, 0.5, 0.0, output);
//     // cv::addWeighted(img1, 0.5, img2, 0.5, 0.0, output);

//     imwrite("output.png", output);
//     tm.stop();
// 	  std::cout<<"count="<<tm.getCounter()<<" ,process time="<<tm.getTimeMilli()<<" ms."<<std::endl;
 
// 	return 0;
// }
/////////////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace cv::ocl;
using namespace std;

// Here are the functions used to measure time
double m_dTime = 0;

void perfMeasure_start()
{
    m_dTime = cv::getTickCount();
}

void perfMeasure_end(std::string strLabel)
{
    double currentTime = cv::getTickCount();
    double dTimeTaken = (currentTime - m_dTime) / cv::getTickFrequency();
    printf("[TIME]%25s : %.3lf ms\n", strLabel.c_str(), dTimeTaken * 1000);
    m_dTime = currentTime;
  }

void OpenCLProc(cv::InputArray matSrc, cv::OutputArray matDst)
{
    perfMeasure_start();
    for (int i = 0; i < 50; i++)
    {
        cv::cvtColor(matSrc, matDst, cv::COLOR_BGR2GRAY);
    }
    perfMeasure_end("cvtColor");

    for (int i = 0; i < 50; i++)
    {
        cv::GaussianBlur(matDst, matDst, cv::Size(9, 9), 1.5);
    }
    perfMeasure_end("GaussianBlur");

    for (int i = 0; i < 50; i++)
    {
        cv::Canny(matDst, matDst, 0, 50);
    }
    perfMeasure_end("Canny");

    for (int i = 0; i < 50; i++)
    {
        cv::dilate(matDst, matDst, cv::noArray());
    }
    perfMeasure_end("Dilate");

    for (int i = 0; i < 50; i++)
    {
        cv::add(matSrc, matSrc, matDst);
    }
    perfMeasure_end("Add");

    for (int i = 0; i < 50; i++)
    {
        cv::multiply(matSrc, matDst, matDst);
    }
    perfMeasure_end("multiply");

    for (int i = 0; i < 50; i++)
    {
        cv::multiply(matSrc, 2.5, matDst);
    }
    perfMeasure_end("multiply_scalar");

    for (int i = 0; i < 50; i++)
    {
        cv::divide(matSrc, matDst, matDst);
    }
    perfMeasure_end("divide");

    for (int i = 0; i < 50; i++)
    {
        cv::divide(matSrc, 2.5, matDst);
    }
    perfMeasure_end("divide_Scalar");

    for (int i = 0; i < 50; i++)
    {
        cv::addWeighted(matSrc,2.5 ,matDst,0.6 ,0,matDst);
    }
    perfMeasure_end("addWeighted");

}

void TestOpenCL(cv::InputArray matSrc)
{
    printf("[PERF] -= Performance Check =-\n");       
    printf("[PERF] ImageSize = %d x %d\n", matSrc.cols(), matSrc.rows());

    //////////// CPU MEASUREMENT CODE //////////////////////
    printf("[PERF] CPU Process\n");
    perfMeasure_start();
    cv::Mat img, gray;
    cv::Mat matImg, matDst;

    // Read
    matImg = matSrc.getMat();
    perfMeasure_end("Read");

    //Transfer
    matImg.copyTo(img);
    perfMeasure_end("Transfer CPU->CPU");

    // Process
    OpenCLProc(img, gray);

    // Transfer
    gray.copyTo(matDst);
    perfMeasure_end("Transfer CPU->CPU");

    std::cout << "\n";

    //////////////////// GPU MEASUREMENT CODE 1 //////////////////////
    printf("[PERF] GPU Process (copyTo)\n");
    cv::UMat img1, gray1;
    cv::Mat matImg1, matDst1;

    // Read
    matImg1 = matSrc.getMat();
    perfMeasure_end("Read");

    // Transfer
    matImg1.copyTo(img1);
    perfMeasure_end("Transfer CPU->GPU");

    //Process
    OpenCLProc(img1, gray1);

    // Transfer
    gray1.copyTo(matDst1);
    perfMeasure_end("Transfer GPU->CPU");
    std::cout << "\n";


    return;

}


int main(int argc, char** argv)
{

  if (cv::ocl::haveOpenCL())
  {
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
      std::cerr << "Failed creating the OpenCL context" << std::endl;
      return -1;
    }
    cv::ocl::setUseOpenCL(true);
  }

  ros::init(argc, argv, "tof_cam_node");

  cv::Mat matCPU  = cv::imread("test3.jpg");
  cv::UMat matGPU = cv::imread("test3.jpg").getUMat(cv::ACCESS_RW);
  cv::Mat outputCPU, outputGPU;
  TickMeter tmCPU, tmGPU;

  tmCPU.start();
  for (int i = 0; i < 50; i++)
  {
    cv::multiply(matCPU, 2.5, matCPU);
  }
  tmCPU.stop();
  std::cout<<"count="<<tmCPU.getCounter()<<" ,process time="<<tmCPU.getTimeMilli()<<" ms."<<std::endl;

  tmGPU.start();
  for (int i = 0; i < 50; i++)
  {
    cv::multiply(matGPU, 2.5, matGPU);
  }
  tmGPU.stop();
  std::cout<<"count="<<tmGPU.getCounter()<<" ,process time="<<tmGPU.getTimeMilli()<<" ms."<<std::endl;


  imwrite("matCPU.png", matCPU);
  imwrite("matGPU.png", matGPU);

  // cv::Mat matSrc = cv::imread("test3.jpg");
  // TestOpenCL(matSrc);
  return 0;
}
