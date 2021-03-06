#include <opencv2/opencv.hpp>
#include <iostream>
#include "Camera/startFetch.hpp"
#include "Camera/shm.hpp"
using namespace std;
void preProcess(cv::Mat &img, cv::Mat &out);//boost::bind(callback,_1,_2)
std::string filename = "../Config/Camera/daHuaCamConfig.yml";

int main() {
    DahuaCamera camera;

    camera.connectCamera(filename);

    return 1;
}

void preProcess(cv::Mat &img, cv::Mat &out) {
    // cv::cvtColor(img,out,cv::COLOR_RGB2GRAY);
    cv::resize(img, out, cv::Size(), 0.75, 0.75);
}
