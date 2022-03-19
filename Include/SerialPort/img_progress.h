//
// Created by zhanghang on 19-2-13.
//

#ifndef SENTRY10_21_IMG_PROGRESS_H
#define SENTRY10_21_IMG_PROGRESS_H

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>
//#include <opencv2/core/eigen.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "timer.h"//计时器
#include "Serial_port.h"//串口
#include "imgSubscrible.h"//图片下载器
#include "PublishAsync.h"//上传器



static void pub_callback(const std::vector<uint8_t> &data);
static void sp_callback(const uint8_t *recv_buf);
static void callback(const CubotImg &img);

#endif //SENTRY10_21_IMG_PROGRESS_H
