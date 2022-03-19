//
// Created by robostar on 2019/10/2.
//

#ifndef ARMOR_LEARNING_SVM_TRAIN_H
#define ARMOR_LEARNING_SVM_TRAIN_H

#include "Src/ArmorFinder/ArmorLibs.h"
#include <opencv2/opencv.hpp>
void get_data(cv::Mat &im_trains,std::vector<int>&im_labels,int label);
void SVM_train();


//训练及数据
static cv::Mat train_images,train_data;//训练集图片
static std::vector<int>train_labels;//训练集标签



#endif //ARMOR_LEARNING_SVM_TRAIN_H
