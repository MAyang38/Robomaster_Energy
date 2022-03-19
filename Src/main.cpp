//
// Created by shuai on 2019/11/27.
//
#include "Src/ArmorFinder/ArmorLibs.hpp"
#include "shm.hpp"
#include "Src/windmill//Energy.h"

std::chrono::system_clock::time_point now;
std::time_t now_c;

int main()
{
#ifdef VIDEO_READ
    cv::Mat ss;
    publisher hhh("camera_Src_1",ss);
    publisher h("camera_Pre_1",ss);
#endif
    //int mode=0;

    int receive[3] = {1, 0, 1};

    static shm_subscriber<int> mode_sub("car_mode");
    subscriber Src_Sub_1("camera_Src_1");
    subscriber Pre_Sub_1("camera_Pre_1");
    AutoHit camnode1(1, Src_Sub_1, Pre_Sub_1);
    Energy energyDectction(Src_Sub_1);
    std::cout<<"TEST_FLAG_1"<<std::endl;
#ifdef SERIAL
    cubot::serial_port  sp (sp_callback, "/dev/ttyUSB0", 115200);
#endif
    while (1) {
        long whieStart = getCurrentTime__();
        mode_sub.get(receive);
#ifdef ENERGY_TEST
    int mode=2;
#elif defined(AUTOHIT_TEST)
    int mode=1;
#else
    int mode=receive[0];
#endif

#ifdef SERIAL
        if(mode==1)camnode1.start(sp);
        else if(mode==2)energyDectction.Task(sp);
#else
        if(mode==1)camnode1.start();
        else if(mode==2)energyDectction.Task();
#endif
        long t=getCurrentTime__()-whieStart;
        if((t)/1000.0<9.8)
            usleep((9.8-(t/1000.0))*1000.0);
        //std::cout<<mode<<std::endl;
        //std::cout<<"mode change"<<std::endl;
        std::vector<cv::Point2f>gtargetsAngle;
        gtargetsAngle.emplace_back(0,0);
        //gtargetsAngle.emplace_back(0,0);
#ifdef SERIAL
        int isSpSucss=sp.SerialPublish(gtargetsAngle);
        std::cout<<"FLAF_TEST_!!!!!!"<<std::endl;
#endif
    }
    return 0;
}

