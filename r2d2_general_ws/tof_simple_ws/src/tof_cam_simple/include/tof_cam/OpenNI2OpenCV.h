#ifndef _OpenNI2OpenCV_H_
#define _OpenNI2OpenCV_H_

#include <iostream>
#include <stdio.h>

#include <OpenNI.h>
#include <opencv2/opencv.hpp>

namespace OpenNIOpenCV {

/*

    Функция для получения поддерживаемых форматов пикселей с строковом
    виде для дальнейшего вывода
    Аргументы:
        - pixelformat - формат пикселя
*/
const char* PixelFormatToStr(openni::PixelFormat pixelformat) {
    switch (pixelformat) {
        case openni::PIXEL_FORMAT_DEPTH_100_UM:
            return "PIXEL_FORMAT_DEPTH_100_UM";
            break;
        case openni::PIXEL_FORMAT_DEPTH_1_MM:
            return "PIXEL_FORMAT_DEPTH_1_MM";
            break;
        case openni::PIXEL_FORMAT_DEPTH_1_3_MM:
            return "PIXEL_FORMAT_DEPTH_1_3_MM";
            break;
        case openni::PIXEL_FORMAT_DEPTH_1_2_MM:
            return "PIXEL_FORMAT_DEPTH_1_2_MM";
            break;
        case openni::PIXEL_FORMAT_GRAY16:
            return "PIXEL_FORMAT_GRAY16";
            break;
        case openni::PIXEL_FORMAT_GRAY8:
            return "PIXEL_FORMAT_GRAY8";
            break;
        case openni::PIXEL_FORMAT_JPEG:
            return "PIXEL_FORMAT_JPEG";
            break;
        case openni::PIXEL_FORMAT_RGB888:
            return "PIXEL_FORMAT_RGB888";
            break;
        case openni::PIXEL_FORMAT_SHIFT_9_2:
            return "PIXEL_FORMAT_SHIFT_9_2";
            break;
        case openni::PIXEL_FORMAT_SHIFT_9_3:
            return "PIXEL_FORMAT_SHIFT_9_3";
            break;
        case openni::PIXEL_FORMAT_YUV422:
            return "PIXEL_FORMAT_YUV422";
            break;
        default:
            return "unknown";
            break;
    }
}
    
class OpenNI2OpenCV
{
private:
    openni::VideoMode depthVideoMode, colorVideoMode, irVideoMode;
    openni::VideoStream m_depthStream, m_colorStream, m_irStream;
    openni::Device m_device;
    int m_height, m_width;

/*
    Функция для вычисления значения градиента писелей в зависимости от расстояния
    на карте глубины
    Агрументы:
        - color1, color2 - цвета наиболее удаленного и наиболее близкого объектов соответсвенно
        - progress - параметр определяющий долю каждого цвета (имеет значение от 0 до 1, зависит от раастояния)
*/
    cv::Vec3b interpolation(cv::Vec3b color1, cv::Vec3b color2, float progress)
    {
        cv::Vec3b newColor;
        uint8_t b1 = color1[0];
        uint8_t g1 = color1[1];
        uint8_t r1 = color1[2];

        uint8_t b2 = color2[0];
        uint8_t g2 = color2[1];
        uint8_t r2 = color2[2];

        float progress2 = 1 - progress;
        newColor[0] = (uchar)cv::saturate_cast<uint8_t>(b1 * progress + b2 * progress2);
        newColor[1] = (uchar)cv::saturate_cast<uint8_t>(g1 * progress + g2 * progress2);
        newColor[2] = (uchar)cv::saturate_cast<uint8_t>(r1 * progress + r2 * progress2);
        return newColor;
    }
public:
    OpenNI2OpenCV() {};
    ~OpenNI2OpenCV()
    {
        m_device.close();
        if (m_colorStream.isValid()){
            m_colorStream.stop();
            m_colorStream.destroy();
        }
        if (m_depthStream.isValid()){
            m_depthStream.stop();
            m_depthStream.destroy();
        }
        if (m_irStream.isValid()){
            m_irStream.stop();
            m_irStream.destroy();
        }
    }
/*
    Функция инициализации устройства с которого будут считываться информация,
    а также потоков для цветного изображения, карты глубины, и инфраксного канала.
*/
    openni::Status init()
    {
        openni::Status rc = openni::STATUS_OK;

        const char* deviceURI = openni::ANY_DEVICE;
        rc = openni::OpenNI::initialize();

        printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
        rc = m_device.open(deviceURI);
        if (rc != openni::STATUS_OK)
        {
            printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
            return openni::STATUS_NO_DEVICE;
        }
        /*
        Создание потоков для считывания информации с камер
        */
        rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
        if (rc != openni::STATUS_OK){
            std::cout << "Couldn't find depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
            return openni::STATUS_ERROR;
        }
        rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
        if (rc != openni::STATUS_OK){
            std::cout << "Couldn't find color stream: " << openni::OpenNI::getExtendedError() << std::endl;
            return openni::STATUS_ERROR;
        }
        rc = m_irStream.create(m_device, openni::SENSOR_IR);
        if (rc != openni::STATUS_OK){
            std:: cout << "Couldn't find ir stream: " <<  openni::OpenNI::getExtendedError() << std::endl;
            return openni::STATUS_ERROR;
        }

        /*
        Выбор режима синхронизации изображения
        В камере Scanmax M5 3D доступно 2 режима:
            1. Без синхронизации (IMAGE_REGISTRATION_OFF)
            2. С синхронизацией цветного канала и канала глубины (IMAGE_REGISTRATION_DEPTH_TO_COLOR)
        */
//        openni::ImageRegistrationMode regMode = openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
        openni::ImageRegistrationMode regMode = openni::IMAGE_REGISTRATION_OFF;
        if (m_device.isImageRegistrationModeSupported(regMode)){
            if (m_device.getImageRegistrationMode() != regMode){
                rc = m_device.setImageRegistrationMode(regMode);
                if (rc != openni::STATUS_OK){
                    std::cout << "Can't set registration mode" << openni::OpenNI::getExtendedError() <<std::endl;
                }
            }
            else{
                std::cout << "Image Registration Mode = ON" << std::endl;;
            }
        }
        else{
            std::cout << "Image Registration Mode dont supported " << std::endl;
        }

        m_colorStream.setMirroringEnabled(true);
        m_depthStream.setMirroringEnabled(true);
        m_irStream.setMirroringEnabled(true);

        rc = m_depthStream.start();
        if (rc != openni::STATUS_OK)
        {
            std::cout << "SimpleViewer: Couldn't start depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
            m_depthStream.destroy();
            return openni::STATUS_ERROR;
        }

        rc = m_colorStream.start();
        if (rc != openni::STATUS_OK)
        {
            std::cout << "Couldn't start color stream: " << openni::OpenNI::getExtendedError() << std::endl;
            m_colorStream.destroy();
            return openni::STATUS_ERROR;
        }

        rc = m_irStream.start();
        if (rc != openni::STATUS_OK)
        {
            std::cout << "Couldn't start IR stream: " << openni::OpenNI::getExtendedError() << std::endl;;
            m_irStream.destroy();
            return openni::STATUS_ERROR;
        }

        if ((!m_depthStream.isValid()) && (!m_colorStream.isValid()) && (!m_irStream.isValid()))
        {
            std::cout << "No valid streams. Exiting" << std::endl;
//            openni::OpenNI::shutdown();
            return openni::STATUS_ERROR;
        }
        if (m_depthStream.isValid() && m_colorStream.isValid() && m_irStream.isValid())
        {
            depthVideoMode = m_depthStream.getVideoMode();
            colorVideoMode = m_colorStream.getVideoMode();
            irVideoMode = m_irStream.getVideoMode();

            int depthWidth = depthVideoMode.getResolutionX();
            int depthHeight = depthVideoMode.getResolutionY();
            int colorWidth = colorVideoMode.getResolutionX();
            int colorHeight = colorVideoMode.getResolutionY();
            int irWidth = irVideoMode.getResolutionX();
            int irHeight = irVideoMode.getResolutionY();

            if (depthWidth == colorWidth &&
                depthHeight == colorHeight &&
                depthWidth == irWidth &&
                depthHeight == irHeight)
            {
                m_width = depthWidth;
                m_height = depthHeight;
            }
            else
            {
                printf("Error - expect color and depth to be in same resolution: D: %dx%d, C: %dx%d\n",
                       depthWidth, depthHeight,
                       colorWidth, colorHeight);
                return openni::STATUS_ERROR;
            }
        }
        else if (m_depthStream.isValid())
        {
            depthVideoMode = m_depthStream.getVideoMode();
            m_width = depthVideoMode.getResolutionX();
            m_height = depthVideoMode.getResolutionY();
        }
        else if (m_colorStream.isValid())
        {
            colorVideoMode = m_colorStream.getVideoMode();
            m_width = colorVideoMode.getResolutionX();
            m_height = colorVideoMode.getResolutionY();
        }
        else if (m_irStream.isValid())
        {
            irVideoMode = m_irStream.getVideoMode();
            m_width = irVideoMode.getResolutionX();
            m_height = irVideoMode.getResolutionY();
        }
        else
        {
            std::cout << "Error - expects at least one of the streams to be valid..." << std::endl;
            return openni::STATUS_ERROR;
        }
        return openni::STATUS_OK;
    }
    /*
        Функция для получения кадра цветоного канала
        Аргументы:
            - frame - Матрица для записи полученного с устройства кадра
    */
    void getColorFrame(cv::Mat& frame)
    {
        // if(frame.cols != m_width || frame.rows != m_height) {
        //     frame.create(m_height, m_width, CV_8UC3);
        // }
        frame.create(colorVideoMode.getResolutionY(), colorVideoMode.getResolutionX(), CV_8UC3);

        int cols, rows;

        openni::VideoFrameRef colorFrame;

        m_colorStream.setVideoMode(colorVideoMode);

        m_colorStream.readFrame(&colorFrame);
        openni::RGB888Pixel* dData = (openni::RGB888Pixel*)colorFrame.getData();
        memcpy(frame.data, dData, colorFrame.getStrideInBytes() * colorFrame.getHeight());
        cvtColor(frame, frame, cv::COLOR_RGB2BGR);
    }
    /*
        Функция для получения кадра канала глубины
        Аргументы:
            - frame - Матрица для записи полученного с устройства кадра
    */
    void getDepthFrame(cv::Mat& frame)
    {
        cv::Mat localFrame;
        if(localFrame.cols != m_width || localFrame.rows != m_height) {
            frame.create(m_height, m_width, CV_8UC3);
            localFrame.create(m_height, m_width, CV_16SC1);
        }
        openni::VideoMode vm = m_depthStream.getVideoMode();

        openni::VideoFrameRef depthFrame;

        m_depthStream.readFrame(&depthFrame);
        openni::DepthPixel* dData = (openni::DepthPixel*)depthFrame.getData();
        memcpy(localFrame.data, dData, depthFrame.getStrideInBytes() * depthFrame.getHeight());

        float maxVal = floor(4000.0);
        cv::Vec3b color1 = cv::Vec3b(255, 0, 0);    // Цвет для наиболее удаленных объектов
        cv::Vec3b color2 = cv::Vec3b(0, 0, 255);    // Цвет для наиболее близких объектов
        for( int y = 0; y < frame.rows; y++ ) {
            for( int x = 0; x < frame.cols; x++ ) {
                uint16_t dist = (uint16_t)localFrame.at<uint16_t>(y,x);
                if (dist == 0){
                    frame.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                }
                else{
                    float progress = dist / maxVal;
                    frame.at<cv::Vec3b>(y, x) = interpolation(color1, color2, progress);
                }
            }
        }
    }

    void getDepthFrame16C1(cv::Mat& frame){
        if(frame.cols != m_width || frame.rows != m_height) {
            frame.create(m_height, m_width, CV_16SC1);
        }

        openni::VideoFrameRef depthFrame;

        m_depthStream.readFrame(&depthFrame);
        openni::DepthPixel* dData = (openni::DepthPixel*)depthFrame.getData();
        memcpy(frame.data, dData, depthFrame.getStrideInBytes() * depthFrame.getHeight());

        frame = frame / 3;
    }

    /*
        Функция для получения кадра инфракрасного канала
        Аргументы:
            - frame - Матрица для записи полученного с устройства кадра
    */
    void getIrFrame(cv::Mat& frame){
        if(frame.cols != m_width || frame.rows != m_height) {
            frame.create(m_height, m_width, CV_8UC1);
        }
        openni::VideoMode vm = m_depthStream.getVideoMode();

        openni::VideoFrameRef irFrame;

        m_irStream.readFrame(&irFrame);
        openni::Grayscale16Pixel* dData = (openni::Grayscale16Pixel*)irFrame.getData();
        memcpy(frame.data, dData, irFrame.getStrideInBytes() * irFrame.getHeight());
    }
    /*
        Метод, для получения информации о цветном канале
    */
    void printColorSensorInfo(){
        std::cout << "Sensor Info fo color stream" << std::endl;
        if (m_colorStream.isValid()){
            const openni::SensorInfo& sensorinflo = m_colorStream.getSensorInfo();
            const openni::Array<openni::VideoMode>& videomodes = sensorinflo.getSupportedVideoModes();
            for (int i = 0; i < videomodes.getSize(); i++)
            {
                std::cout << i << ": " << videomodes[i].getResolutionX() << "x" << videomodes[i].getResolutionY()<< ": " << PixelFormatToStr(videomodes[i].getPixelFormat()) << ": " << videomodes[i].getFps() << std::endl;
            }
        }
        else{
            std::cout << "Color stream is not valid" << std::endl;
        }
    }
    /*
        Метод, для получения информации о канале глубины
    */
    void printDepthSensorInfo(){
        std::cout << "Sensor Info fo depth stream" << std::endl;
        if (m_depthStream.isValid()){
            const openni::SensorInfo& sensorinflo = m_depthStream.getSensorInfo();
            const openni::Array<openni::VideoMode>& videomodes = sensorinflo.getSupportedVideoModes();
            for (int i = 0; i < videomodes.getSize(); i++)
            {
                std::cout << i << ": " << videomodes[i].getResolutionX() << "x" << videomodes[i].getResolutionY()<< ": " << PixelFormatToStr(videomodes[i].getPixelFormat()) << ": " << videomodes[i].getFps() << std::endl;
            }
        }
        else{
            std::cout << "Depth stream is not valid" << std::endl;
        }
    }
    /*
        Метод, для получения информации о инфракрасном канале
    */
    void printIrSensorInfo(){
        std::cout << "Sensor Info fo IR stream" << std::endl;
        if (m_irStream.isValid()){
            const openni::SensorInfo& sensorinflo = m_irStream.getSensorInfo();
            const openni::Array<openni::VideoMode>& videomodes = sensorinflo.getSupportedVideoModes();
            for (int i = 0; i < videomodes.getSize(); i++)
            {
                std::cout << i << ": " << videomodes[i].getResolutionX() << "x" << videomodes[i].getResolutionY()<< ": " << PixelFormatToStr(videomodes[i].getPixelFormat()) << ": " << videomodes[i].getFps() << std::endl;
            }
        }
        else{
            std::cout << "Depth stream is not valid" << std::endl;
        }
    }

    void setColorVideoMode(int type){
      if (m_colorStream.isValid()){
        const openni::SensorInfo& sensorinflo = m_colorStream.getSensorInfo();
        const openni::Array<openni::VideoMode>& videomodes = sensorinflo.getSupportedVideoModes();
        for (int i = 0; i < videomodes.getSize(); i++)
        {
          std::cout << i << ": " << videomodes[i].getResolutionX() << "x" << videomodes[i].getResolutionY()<< ": " << PixelFormatToStr(videomodes[i].getPixelFormat()) << ": " << videomodes[i].getFps() << std::endl;
        }

        colorVideoMode = videomodes[type];
        if (colorVideoMode == videomodes[type]){
            std::cout << "SUCCESS VideoMode Set" << std::endl;
            std::cout << "colorVideoMode: " << colorVideoMode.getResolutionX() << "x" << colorVideoMode.getResolutionY()<< ": " << PixelFormatToStr(colorVideoMode.getPixelFormat()) << ": " << colorVideoMode.getFps() << std::endl;

        }
      } else{
        std::cout << "Color stream is not valid" << std::endl;
      }
    }

};

}

#endif // _OpenNI2OpenCV_H_)