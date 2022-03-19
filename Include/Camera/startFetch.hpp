//
// Created by dasuda on 18-11-21.
//

#ifndef DELTAIPS_STARTFETCH_HPP
#define DELTAIPS_STARTFETCH_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
#include "StreamRetrieve.h"
#include "camParaConfig.h"
#include "shm.hpp"

class DahuaCamera
{
private:
    std::chrono::system_clock::time_point now;
    std::time_t now_c;
    std::ofstream fout;
    static void callback(cv::Mat& Src,cv::Mat &Pre);
    static void nothing(cv::Mat& in, cv::Mat& out);
    std::string node_class_name;
    daHuaPara_str daHuaPara; //相机参数
    ICameraPtr cameraSptr;   //相机指针
public:
    bool connectCamera(std::string &filename,
                       preProcessFunction _callback=boost::bind(DahuaCamera::nothing,_1,_2));
};

#endif //DELTAIPS_STARTFETCH_HPP
