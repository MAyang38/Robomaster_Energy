//
// Created by robostar on 2019/10/19.
//

#include<string>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include "Src/ArmorFinder/ArmorLibs.h"

void get_data_hog(cv::Mat &im_trains, std::vector<int> &im_labels, int label);


//训练及数据
static cv::Mat train_images_hog, train_data_hog;//训练集图片
static std::vector<int> train_labels_hog;//训练集标签
cv::HOGDescriptor hog(cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
std::vector<float> descriptors;

/***********定义获取训练集函数********************************************************************************************/
void get_data_hog(cv::Mat &im_trains, std::vector<int> &im_labels, int label) {
    /*
      int nums;
      switch(label)//确定不同标签训练集数量
      {
          case 0:
              nums=10032;
              break;
          case 1:
              nums=667;//94;
              break;
          case 2:
              nums=806;//75;
              break;
          case 3:
              nums=1787;//81;
              break;
          case 4:
              nums=467;//150;
              break;
          case 5:
              nums=1616;//78;
              break;
          case 6:
              nums=86;
              break;
          case 7:
              nums=79;
              break;

          default:
              std::cout<<"程序出现错误"<<std::endl;
              cv::waitKey(0);
              break;
      }*/
    for (int index = 1;; index++)//样本数量
    {
        cv::Mat src_train = cv::imread(
                "/home/robostar/armor_02/Config/TrainDate2/" + std::to_string(label) + "/" + std::to_string(index) +
                ".bmp", 1);//注意保证每张图片均存在；
        if (src_train.size().width > 0 && src_train.size().height > 0) {
            cv::imshow("train_image", src_train);
            cvtColor(src_train, src_train, cv::COLOR_BGR2GRAY);

            //!Canny算子
            cv::Canny(src_train, src_train, 7, 21, 3, true);

            cv::erode(src_train, src_train, 5, cv::Point(-1, -1), 7, cv::BORDER_DEFAULT);
            cv::dilate(src_train, src_train, 5, cv::Point(-1, -1), 7, cv::BORDER_DEFAULT);

            /*中值滤波
            uchar* data= src_train.ptr<uchar>(0);
            for(int i=0;i<src_train.rows*src_train.cols;i++)
            {

                data[i]=(data[i])*10;
            }
            medianBlur(src_train, src_train, 3);

*/
            cv::imshow("train_image", src_train);

            descriptors.clear();
            hog.compute(src_train, descriptors);

            std::cout << label << "," << index << std::endl;
            src_train = cv::Mat(1, int(descriptors.size()), CV_32FC1, descriptors.data());
            im_trains.push_back(src_train);//添加hog特征训练集

            im_labels.push_back(label);//添加标签
            std::cout << "======" << index << "======" << std::endl;
        } else
            std::cout << "****************************************" << std::endl;
        if (src_train.empty()) {
            break;
        }
    }
}


int main() {
    /******************获取训练集***************************************************************************************/
    for (int id = 0; id <= 5; id++) {
        get_data_hog(train_images_hog, train_labels_hog, id);
    }
    train_images_hog.copyTo(train_data_hog);
    train_images_hog.convertTo(train_data_hog, CV_32FC1);



    /********************配置参数***************************************************************************************/
    cv::Ptr<cv::ml::SVM> SVM_params = cv::ml::SVM::create();
    SVM_params->setType(cv::ml::SVM::C_SVC);
    SVM_params->setKernel(cv::ml::SVM::KernelTypes::RBF);
    SVM_params->setGamma(0.01);
    SVM_params->setC(10);
    //SVM_params->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 200, FLT_EPSILON));
    SVM_params->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 3000, 1e-6));
    cv::Ptr<cv::ml::TrainData> tData = cv::ml::TrainData::create(train_data_hog, cv::ml::ROW_SAMPLE, train_labels_hog);
    SVM_params->trainAuto(tData);
    //SVM_params->train(train_data, cv::ml::ROW_SAMPLE, train_labels);//开始训练

    SVM_params->save("SVM_HOG_Canny_2.1_2019_11_04_MON.xml");
    std::cout << "训练成功" << std::endl;

    return 0;
}