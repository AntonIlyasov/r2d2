#pragma once

#include <iostream>
#include <stdio.h>

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

namespace OpenNIOpenCV {

    enum colorType{
        COLOR_1280_720_RGB888_30FPS,
        COLOR_1920_1080_RGB888_15FPS
    };

    /*

        Функция для получения поддерживаемых форматов пикселей с строковом
        виде для дальнейшего вывода
        Аргументы:
            - pixelformat - формат пикселя
    */
    const char* PixelFormatToStr(openni::PixelFormat pixelformat);
    
    class OpenNI2OpenCV
    {
    private:
        openni::VideoMode depthVideoMode, colorVideoMode, irVideoMode;
        openni::VideoStream m_depthStream, m_colorStream, m_irStream;
        openni::Device m_device;
        int m_height, m_width;
        bool is_turn_off, is_turn_on, is_reset;

    /*
        Функция для вычисления значения градиента писелей в зависимости от расстояния
        на карте глубины
        Агрументы:
            - color1, color2 - цвета наиболее удаленного и наиболее близкого объектов соответсвенно
            - progress - параметр определяющий долю каждого цвета (имеет значение от 0 до 1, зависит от раастояния)
    */
        cv::Vec3b interpolation(cv::Vec3b color1, cv::Vec3b color2, float progress);
    public:
        OpenNI2OpenCV();
        ~OpenNI2OpenCV();
        /*
            Функция инициализации устройства с которого будут считываться информация,
            а также потоков для цветного изображения, карты глубины, и инфраксного канала.
        */
        openni::Status init();
        /*
            TODO
        */
        openni::Status getStatus();
        /*
            Функция для получения кадра цветоного канала
            Аргументы:
                - frame - Матрица для записи полученного с устройства кадра
        */
        void getColorFrame(cv::Mat& frame);
        /*
            Функция для получения кадра канала глубины
            Аргументы:
                - frame - Матрица для записи полученного с устройства кадра
        */
        void getDepthFrame(cv::Mat& frame);
        /*
            Функция для получения кадра инфракрасного канала
            Аргументы:
                - frame - Матрица для записи полученного с устройства кадра
        */
        void getIrFrame(cv::Mat& frame);
        /*
            Метод, для получения информации о цветном канале
        */
        void printColorSensorInfo();
        /*
            Метод, для получения информации о канале глубины
        */
        void printDepthSensorInfo();
        /*
            Метод, для получения информации о инфракрасном канале
        */
        void printIrSensorInfo();
        // Пользовательские методы
        /*
            Метод, для установки разрешения цветного изображения
        */
        void setColorResolution(int, int);
        /*
            Метод, для установки частоты кадров цветного изображения
        */
        void setColorFps(int);
        /*
            Метод, для установки формата пикселей цветного изображения
        */
        void setColorPixelFormat(openni::PixelFormat);
        /*
            Метод, для установки видеорежима цветного изображения
        */
        void setColorVideoMode(size_t);
        /*
            Методы, для получения информации о разрешении цветного изображения
        */
        int getColorResolutionX();
        int getColorResolutionY();
        /*
            Метод, для получения информации о частоты кадров цветного изображения
        */
        int getColorFps();
        /*
            Метод, для получения информации формате пикселей цветного изображения
        */
        const char* getColorPixelFormat();
        /*
            Методы, для получения информации о разрешении изображения глубины
        */
        int getDepthResolutionX();
        int getDepthResolutionY();
        /*
            Методы, для получения информации о разрешении инфракрасного изображения
        */
        int getIrResolutionX();
        int getIrResolutionY();
        /*
            Функция для получения кадра канала глубины в формате для публикации в топик
            Аргументы:
                - frame - Матрица для записи полученного с устройства кадра 1 канальная 16 битная
        */
        void getDepthFrame16C1(cv::Mat& frame);

        bool m_colorStreamIsValid();
        bool m_depthStreamIsValid();
        bool m_irStreamIsValid();

        bool getIsTurnOff();
        bool getIsTurnOn();
        bool getIsReset();
        void setIsReset(bool);
    };

}