#include <iostream>
#include "deltaCV/SIMD/core.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <time.h>

using namespace std;

int main() {

    cv::Mat img = cv::imread("***.bmp");
    cv::Mat out(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat ycrcbimg, opencvOUT;
    double deltaCVtime, openCVtime;

    // deltaCV
    deltaCV::ycrcbWithSeg(img.data, out.data, img.cols, img.rows, deltaCV::scalar(100, 0, 0),
                          deltaCV::scalar(255, 255, 255));

    // OpenCV
    cv::cvtColor(img, ycrcbimg, cv::COLOR_RGB2YCrCb);
    cv::inRange(ycrcbimg, cv::Scalar(100, 0, 0), cv::Scalar(255, 255, 255), opencvOUT);

    cout << "deltaCV: " << deltaCVtime / (5 * i) << endl;
    cout << "openCV: " << openCVtime / (5 * i) << endl;

    cv::imshow("deltaCV", out);
    cv::imshow("opencv", opencvOUT);
    cv::waitKey(0);

    return 0;
}
