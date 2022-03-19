//
// Created by robostar on 2019/10/2.
//训练模型数字识别

#include "Tools/SVM/SVM_train.h"
#include<string>


/***********定义获取训练集函数********************************************************************************************/
void get_data(cv::Mat &im_trains, std::vector<int> &im_labels, int label) {
    int nums;
    switch (label)//确定不同标签训练集数量
    {
        case 0:
            nums = 10032;
            break;
        case 1:
            nums = 667;//94;
            break;
        case 2:
            nums = 806;//75;
            break;
        case 3:
            nums = 1787;//81;
            break;
        case 4:
            nums = 467;//150;
            break;
        case 5:
            nums = 1616;//78;
            break;

            /*
            case 6:
            nums=86;
            break;
        case 7:
            nums=79;
            break;
             */
        default:
            std::cout << "程序出现错误" << std::endl;
            cv::waitKey(0);
            break;
    }
    for (int index = 1;; index++)//样本数量
    {
        cv::Mat src_train = cv::imread("Train_Data/" + std::to_string(label) + "/" + std::to_string(index) + ".bmp",
                                       0);//注意保证每张图片均存在；
        if (src_train.size().width > 0 && src_train.size().height > 0) {
            cv::imshow("t", src_train);
            std::cout << label << "," << index << std::endl;
            //cv::Mat temp1,temp2;
            //cv::threshold(temp1,temp2,0,255,cv::THRESH_OTSU+cv::THRESH_BINARY);//OTSU自适应阈值
            src_train = src_train.reshape(0, 1);//将图片转成一行
            im_trains.push_back(src_train);//添加训练集
            im_labels.push_back(label);//添加标签
        } else
            std::cout << "****************************************" << std::endl;
        if (src_train.empty()) {
            break;
        }
    }
}


void SVM_train() {
    /******************获取训练集***************************************************************************************/
    for (int id = 0; id <= 5; id++) {
        get_data(train_images, train_labels, id);
    }
    train_images.copyTo(train_data);
    train_images.convertTo(train_data, CV_32FC1);



    /********************配置参数***************************************************************************************/
    cv::Ptr<cv::ml::SVM> SVM_params = cv::ml::SVM::create();
    SVM_params->setType(cv::ml::SVM::C_SVC);
    SVM_params->setKernel(cv::ml::SVM::KernelTypes::LINEAR);
    //SVM_params->setGamma(0.01);
    //SVM_params->setC(10);
    //SVM_params->setTermCriteria(cv::TermCriteria(cv::TermCriteria::MAX_ITER, 200, FLT_EPSILON));
    SVM_params->setTermCriteria(cv::TermCriteria(cv::INTER_MAX, 10000, 1e-6));
    cv::Ptr<cv::ml::TrainData> tData = cv::ml::TrainData::create(train_data, cv::ml::ROW_SAMPLE, train_labels);
    SVM_params->trainAuto(tData);
    //SVM_params->train(train_data, cv::ml::ROW_SAMPLE, train_labels);//开始训练

    SVM_params->save("SVM_params.xml");
    std::cout << "训练成功" << std::endl;
}