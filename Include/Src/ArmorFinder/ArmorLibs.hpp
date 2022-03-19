#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <string>
#include <sys/time.h>

#include "ForDefine.h"
#ifdef LOG_DATA
#include <fstream>
#include <ctime>
#include <chrono>
#endif

#include "shm.hpp"
#include "Serial_port.h"
#include "imgSubscrible.h"
#include "SubscribeAsync.h"



void RoiProcesing(cv::Mat &roi)
{
    cv::cvtColor(roi,roi,cv::COLOR_BGR2GRAY);
    uchar* data= roi.ptr<uchar>(0);
    for(int i=0;i<roi.rows*roi.cols;i++)
    {
        data[i]=(data[i])*10;
    }
    cv::medianBlur(roi,roi,3);//中值滤波
}

inline long getCurrentTime__()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000000+tv.tv_usec;
}



inline void PointLimit(std::vector<cv::Point2f> &p, int rows, int cols) {
    for (int index = 0; index < p.size(); index++) {
        if (p[index].x >= cols)
            p[index].x = cols - 1;
        else if (p[index].x < 0)
            p[index].x = 0;

        if (p[index].y >= rows)
            p[index].y = rows - 1;
        else if (p[index].y < 0)
            p[index].y = 0;
    }
}
//重载函数，适用于单个点情况
inline void PointLimit(cv::Point2f &p, int rows, int cols)
{
    if (p.x >= cols)
        p.x = cols - 1;
    else if (p.x < 0)
        p.x = 0;

    if (p.y >= rows)
        p.y = rows - 1;
    else if (p.y < 0)
        p.y = 0;
}


inline void PointLimit(cv::Point2d &p, int rows, int cols)
{
    if (p.x >= cols)
        p.x = cols - 1;
    else if (p.x < 0)
        p.x = 0;

    if (p.y >= rows)
        p.y = rows - 1;
    else if (p.y < 0)
        p.y = 0;
}

class AutoHit;
class Armor;
class LightBlob
{
public:
    LightBlob(cv::RotatedRect &r,cv::Mat &Src);//构造函数
    LightBlob();
private:
    friend AutoHit;
    friend Armor;

    float angle;//外接矩形倾角,椭圆长轴与x轴夹角
    cv::RotatedRect rect;//最小外接矩形
    cv::Point2f start_point,end_point;
    float priority;//距屏幕中心距离
    bool color;
};
inline LightBlob::LightBlob(cv::RotatedRect &r,cv::Mat &Src): rect(r)
{
    angle=rect.angle*CV_PI/180;//计算灯条的倾角
    priority=sqrtf(pow(rect.center.x- Src.cols / 2.0, 2) + pow(rect.center.y - Src.rows / 2.0, 2));

    start_point.x=rect.center.x+rect.size.height/2.0*sinf(angle);
    start_point.y=rect.center.y-rect.size.height/2.0*cosf(angle);
    end_point.x=rect.center.x-rect.size.height/2.0*sinf(angle);
    end_point.y=rect.center.y+rect.size.height/2.0*cosf(angle);

    if(start_point.y>end_point.y)
    {
        cv::Point2f temp=start_point;
        start_point=end_point;
        end_point=temp;
    }

    PointLimit(start_point, Src.rows, Src.cols);
    PointLimit(end_point, Src.rows, Src.cols);
};

class Armor
{
public:
    Armor(LightBlob &gblob1, LightBlob &gblob2, cv::Mat &src, cv::Mat &SrcShow, cv::Ptr<cv::ml::SVM> SVM,bool &isTrack);
    //float Solve(std::vector<cv::Point2f>&pixelPoints);

private:
    friend AutoHit;
    LightBlob blobL;
    LightBlob blobR;
    cv::Rect2d rect;//包裹装甲区域的矩形
    cv::Point2f roi_tl,roi_tr,roi_bl,roi_br;
    cv::Mat roi;
    int id=0;
    float distancef;
    float priority;//计算装甲中心到屏幕中心的距离，为辅助操作手的瞄准，越靠近屏幕中心，越优先传出目标
    std::vector<cv::Point2f>blobPointsPairs;
    cv::Point2f center;//装甲板中心点坐标
};

class LinearPredict
{
public:
    LinearPredict(float an)
    {
        angularLast=angularCurrency;
        angularCurrency=an;

        palstance5=palstance4;
        palstance4=palstance3;
        palstance3=palstance2;
        palstance2=palstance1;
        palstance1=(angularCurrency-angularLast)/10;
    }
private:
    float angularCurrency;
    float angularLast;
    
    float palstance1=0;
    float palstance2=0;
    float palstance3=0;
    float palstance4=0;
    float palstance5=0;

    float angularAcceleration1=0;
    float angularAcceleration2=0;
    float angularAcceleration3=0;
    float angularAcceleration4=0;
    float angularAcceleration5=0;
};

class AutoHit
{
public:

#ifdef LOG_DATA
    std::ofstream fout;
#endif

    friend Armor;
    AutoHit(int id,subscriber src,subscriber pre);
    float Solve(std::vector<cv::Point2f>&pixel)
    {
        std::vector<cv::Point2f>pixelPoints_backup;

        for(int i=0;i<4;i++){
            pixelPoints_backup.emplace_back(pixel[i]*2);
        }

        cv::solvePnP(objectPoints,pixelPoints_backup, cameraMatrix, disCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        float depth = tvec.at<double>(2,0);
      //  std::cout<<tvec<<std::endl;
        //

        //cv::waitKey(0);
#ifdef TEST_PnP
        cv::Mat Temp;
        cv::resize(Src,Temp,cv::Size(),2,2);
        cv :: drawFrameAxes(Temp,cameraMatrix,disCoeffs,rvec,tvec,1000,3);

        cv::putText(Temp, std::to_string((int)depth/10.0) + "cm", cv::Point2f(520,640), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(50, 100, 200), 1);
        //cv::imshow("solvePnP",Temp);
        std::cout<<"tvec："<<tvec<<std::endl;
    //std::cout<<"距离："<<depth/10.0<<std::endl;
#endif
        return depth/10.0;
    }
    void TrackInit(Armor &armor) {
        trackBoxBackUp = armor.rect;
        trackBoxLimit = trackBoxBackUp;
        float w = 1, h = 1;
        if (grade == 1) {
            w = trackBoxLimit.width / 2.0 * 3.5;
            h = trackBoxLimit.height / 2.0 * 3.5;
        }
        else if (grade == 2) {
            w = trackBoxLimit.width / 2.0 * 4.5;
            h = trackBoxLimit.height / 2.0 * 4.5;
        }
        else if (grade == 3) {
            w = trackBoxLimit.width / 2.0 * 6.0;
            h = trackBoxLimit.height / 2.0 * 6.0;
        }
        cv::Point2d Tcenter = (trackBoxLimit.br() + trackBoxLimit.tl()) / 2.0;
        cv::Point2d t(w, h);
        cv::Point2d t1 = Tcenter - t;
        cv::Point2d t2 = Tcenter + t;
        PointLimit(t1, Pre.rows, Pre.cols);
        PointLimit(t2, Pre.rows, Pre.cols);
        trackBoxLimit = cv::Rect2d(t1, t2);
        ROIRect = Src(trackBoxLimit);


        trackBox=(cv::Rect2d(trackBoxBackUp.tl()-t1,trackBoxBackUp.br()-t1));
        tracker=cv::TrackerKCF::create();
        TrackingROI=ROIRect.clone();
        tracker->init(TrackingROI, trackBox);
        //cv::rectangle(ROIRect,trackBox,cv::Scalar(0,0,255));
        cv::rectangle(SrcShow,trackBoxBackUp,cv::Scalar(0,0,255));
        isTrack = 1;
    }
    void TrackGo()
    {
        trackBoxLimit = trackBoxBackUp;
        //trackBoxLimit.x=trackBox.x;
        //trackBoxLimit.y=trackBox.y;
        float w = 1, h = 1;
        if (grade == 1) {
            w = trackBoxBackUp.width / 2.0 * 3.5;
            h = trackBoxBackUp.height / 2.0 * 3.5;
        }
        else if (grade == 2) {
            w = trackBoxBackUp.width / 2.0 * 4.5;
            h = trackBoxBackUp.height / 2.0 * 4.5;
        }
        else if (grade == 3) {
            w = trackBoxBackUp.width / 2.0 * 6.0;
            h = trackBoxBackUp.height / 2.0 * 6.0;
        }
        cv::Point2d Tcenter = (trackBoxBackUp.br() + trackBoxBackUp.tl()) / 2.0;
        cv::Point2d t(w, h);
        cv::Point2d t1 =  Tcenter - t;
        cv::Point2d t2 =  Tcenter + t;
        PointLimit(t1, Pre.rows, Pre.cols);
        PointLimit(t2, Pre.rows, Pre.cols);
        trackBoxLimit = cv::Rect2d(t1, t2);
        TrackingROI=Src(trackBoxLimit).clone();
        cv::rectangle(SrcShow,trackBoxLimit,cv::Scalar(0,0,255));
    }


    bool blobJudge(float areaContours, float areaRect,cv::RotatedRect rect);
    void blobFinder();
    bool blobMatch(LightBlob &blob1, LightBlob &blob2);
    void armorFind();
    void SingBlob();
    cv::Point2f Target2Angle(cv::Point2f target);
    int sortTarget();

#ifdef SERIAL
    int start(cubot::serial_port &sp);
#else
    int start();
#endif
    void antiSpin();
    void Compensate(float& distance);
    void Compensate(double& distance);
private:
    int grade=0;
    int lastGrade=0;
    float lastdistance;
#ifdef LOG_DATA
    std::chrono::system_clock::time_point now;
    std::time_t now_c;
#endif

    int thread_id;
    subscriber Src_Sub;
    subscriber Pre_Sub;

    cv::Mat Src;
    cv::Mat Pre;
    cv::Mat SrcShow;
    cv::Mat ROIRect;
    cv::Mat TrackingROI;

    cv::Ptr<cv::Tracker> tracker;


    int ENEMY_COLOR;
    bool isTrack=false;
    bool isFindLastID=false;
    int LastID=0;
    int notLastIDNums=0;
    int noTargetNums=0;
    cv::Point2f TargetAngle;
    cv:: Point2f TargetPixel;
    bool isSingleBlobMode=false;
    std::vector<cv::Point2f> gtargetsAngle;


    std::vector<Armor>armors;
    std::vector<cv::Point2f>pixelPoints;
    cv::Point2f armorTransform[4];
    //!得到装甲板时解算 旋转向量/平移向量/深度 函数
    cv::Rect2d trackBox;
    cv::Rect2d trackBoxBackUp;
    cv::Rect2d trackBoxLimit;
    std::vector<LightBlob> lightBlobs;
    std::vector<LightBlob> blobsSingle;
    int index=1;
    int fps = 0;
    int fpsall = 0;

    float areaRect;
    float areaRatioMin;
    float areaRatioMax;
    float heightBlobRectMin;
    float heightBlobRectMax;
    //float angleBlobRectMin;
    float angleBlobRectMax;
    float hwRatioBlobRectMin;
    float hwRatioBlobRectMax;

    float angleDiffBlobPairs;
    float heightRatioBlobPairs;
    float blobSlope;
    float distanceBlobPairsMin;
    float distanceBlobPairsMax;


    float SingleBarHeighLimit;//单灯条检测长度限制

    //!重力补偿距离参数
    float distance1;
    float distance2;
    float distance3;
    //!重力补偿补偿参数
    float comgX1;
    float comgY1;
    float comgX2;
    float comgY2;
    float comgX3;
    float comgY3;

    cv::Ptr<cv::ml::SVM> SVM;


    //!分别定义目标二维/三维点

    std::vector<cv::Point3f>objectPoints;
    //!定义相机内外参
    cv::Mat cameraMatrix;
    cv::Mat disCoeffs;
    Eigen::Isometry3d calMatrix;
    //!定义旋转/平移向量
    cv::Mat rvec;
    cv::Mat tvec;

    Eigen::Matrix3d cameraMatrixe;
    Eigen::Matrix<double, 5, 1> disCoeffse;
    Eigen::Vector3d  targetDistotrted;//畸变校正后目标点

    //像素平面目标点，内参矩阵计算相机坐标系目标点，再得到偏转角

    //决策
};

AutoHit::AutoHit(int id,subscriber src,subscriber pre):Src_Sub(src),Pre_Sub(pre)
{

#ifdef LOG_DATA


    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now );
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    fout.open("../../Robomaster2020/LOG/AH"+std::to_string(id)+"_"+str_time+".log");
    fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
    fout<<"AutoHit initing..."<<std::endl<<std::endl;
#endif
#ifdef SERIAL
    //!串口通信区
//定义一个串口对象

#endif
    //Src_Sub=src;
    //Pre_Sub=pre;
    thread_id=id;
    cv::FileStorage ParamsAH("../Config/AutoHit/ParamsAutoHit.yml",cv::FileStorage::READ);
    if(ParamsAH.isOpened())
    {
        std::cout<<"参数表读取成功!"<<std::endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now );
        fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
        fout<<"load AutoHit params successfully"<<std::endl<<std::endl;
#endif
        //灯条判断区域
        areaRect            =   (float) ParamsAH["areaRect"];
        areaRatioMin        =   (float) ParamsAH["areaRatioMin"];
        areaRatioMax        =   (float) ParamsAH["areaRatioMax"];
        heightBlobRectMin   =   (float) ParamsAH["heightBlobRectMin"];
        heightBlobRectMax   =   (float) ParamsAH["heightBlobRectMax"];
//angleBlobRectMin=       (float) ParamsAH["angleBlobRectMin"];
        angleBlobRectMax    =   (float) ParamsAH["angleBlobRectMax"];
        hwRatioBlobRectMin  =   (float) ParamsAH["hwRatioBlobRectMin"];
        hwRatioBlobRectMax  =   (float) ParamsAH["hwRatioBlobRectMax"];
        SingleBarHeighLimit =   (float) ParamsAH["SingleBarHeighLimit"];
        //灯条匹配区域
        angleDiffBlobPairs  =   (float) ParamsAH["angleDiffBlobPairs"];
        heightRatioBlobPairs=   (float) ParamsAH["heightRatioBlobPairs"];
        blobSlope           =   (float) ParamsAH["blobSlope"];
        distanceBlobPairsMin=   (float) ParamsAH["distanceBlobPairsMin"];
        distanceBlobPairsMax=   (float) ParamsAH["distanceBlobPairsMax"];

        //!重力补偿距离参数
        distance1           =   (float) ParamsAH["distance1"];
        distance2           =   (float) ParamsAH["distance2"];
        distance3           =   (float) ParamsAH["distance3"];
        //!重力补偿补偿参数
        comgX1              =   (float) ParamsAH["comgX1"];
        comgY1              =   (float) ParamsAH["comgY1"];
        comgX2              =   (float) ParamsAH["comgX2"];
        comgY2              =   (float) ParamsAH["comgY2"];
        comgX3              =   (float) ParamsAH["comgX3"];
        comgY3              =   (float) ParamsAH["comgY3"];

        //灯条颜色
        ENEMY_COLOR         =   (int)   ParamsAH["ENEMY_COLOR"];

        ParamsAH.release();
    }
    else
    {
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now );
        fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
        fout<<"load AutoHit params failure"<<std::endl<<std::endl;
#endif
        std::cout<<"参数表读取失败"<<std::endl;
    }
    SVM = cv::Algorithm::load<cv::ml::SVM>("../Config/Models/cubot_svm_7_2_autohit.xml");
    //SVM = cv::Algorithm::load<cv::ml::SVM>("../Config/Models/SVM_HOG_JSTemp_4.3_2019_12_21_SAT.xml");
    //SVM = cv::Algorithm::load<cv::ml::SVM>("../Config/Models/SVM_HOG_2.1_2019_11_04_MON.xml");
    //SVM = cv::Algorithm::load<cv::ml::SVM>("../Config/Models/SVM_HOG_3.0_2019_12_05_MON.xml");
#ifdef LOG_DATA
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now );
    fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
    fout<<"load SVM model successfully"<<std::endl<<std::endl;
#endif

    armorTransform[0]=cv::Point2f(0,0);   armorTransform[1]=cv::Point2f(40,0);
    armorTransform[2]=cv::Point2f(40,40);armorTransform[3]=cv::Point2f(0,40);


    //!读取参数表
    cv::FileStorage ParamsTH("../Config/Camera/armor_cam_para_cfg.yml",cv::FileStorage::READ);
    //!装甲板在世界坐标系坐标
    float objectiveWidth=135;
    float objectiveHeight=125;
    objectPoints.emplace_back(-objectiveWidth / 2.0, -objectiveHeight / 2.0, 0);
    objectPoints.emplace_back(objectiveWidth / 2.0, -objectiveHeight / 2.0, 0);
    objectPoints.emplace_back(objectiveWidth / 2.0, objectiveHeight / 2.0, 0);
    objectPoints.emplace_back(-objectiveWidth / 2.0, objectiveHeight / 2.0, 0);

    if(ParamsTH.isOpened())
    {
        ParamsTH["cameraMatrix"]>>cameraMatrix;
        ParamsTH["distCoeffs"]  >>disCoeffs;
        cv::Mat cvCalMatrix;
        ParamsTH["calMatrix"]   >>cvCalMatrix;
        cv::cv2eigen(cvCalMatrix,calMatrix.matrix());
        std::cout<<"相机参数读取成功"<<std::endl;
    }
    else
    {
        std::cout<<"相机参数读取失败"<<std::endl;
    }
    cv::cv2eigen(disCoeffs, disCoeffse);
    cv::cv2eigen(cameraMatrix,cameraMatrixe);//类型转换

    ParamsTH.release();
    tracker=cv::TrackerKCF::create();

}


bool AutoHit::blobJudge(float areaContours, float areaRect,cv::RotatedRect rect)
{


    /*************尚不知对倒下车辆的影响***********************************************************************************/
    float angle=0;
    if(rect.angle>90)
    {
        angle = 180-rect.angle ;
    } else angle=rect.angle;
    //cv::putText(AL::SrcShow, std::to_string((int)angle), rect.center, cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar(255,255));
    if(angle>angleBlobRectMax)
    {//此处参数表有问题，Max与Min读取结果相反
#ifdef SHOW_BLOB_INCLUDING
        std::cout<<"灯条角度不符合要求:"<<angle<<std::endl;
        cv::putText(SrcShow, "angle"+std::to_string(angle), rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }
    if((rect.size.area() <areaRect && areaContours / areaRect <= areaRatioMin)
       &&          (rect.size.area() >= areaRect && areaContours / areaRect <= areaRatioMax))//面积比
    {
#ifdef SHOW_BLOB_INCLUDING
        std::cout<<"灯条面积比不符合要求:"<<areaContours / areaRect<<std::endl;
        cv::putText(SrcShow, "area", rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }


    if(rect.size.height>heightBlobRectMax||rect.size.height<heightBlobRectMin)
    {
#ifdef SHOW_BLOB_INCLUDING
        std::cout<<"灯条高度不符合要求:"<<rect.size.height<<std::endl;
        cv::putText(SrcShow, "height" + std::to_string(rect.size.height), rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }

    if((rect.size.height/rect.size.width)>hwRatioBlobRectMax||(rect.size.height/rect.size.width<hwRatioBlobRectMin))
    {
#ifdef SHOW_BLOB_INCLUDING
        std::cout<<"灯条长宽比不符合要求:"<<rect.size.height/rect.size.width<<std::endl;
        cv::putText(SrcShow, "HW_ratio" + std::to_string(rect.size.height / rect.size.width), rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }
    return true;
}

void AutoHit::blobFinder()
{
    cv::Mat SrcThresh;
    if(isTrack){
        SrcThresh=ROIRect;
    }else{
        SrcThresh=Pre;
    }

    cv::Mat imContours;//绘制轮廓图
    //轮廓查找
    std::vector<cv::Vec4i>hierarchy_R;//
    std::vector<std::vector<cv::Point>> contoursPoints;//轮廓点储存

    findContours(SrcThresh, contoursPoints, hierarchy_R, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    std::vector<LightBlob>blobs;
    if(contoursPoints.size() > 0)
    {
        //imContours=cv::Mat::zeros(SrcThresh.size(), CV_8UC3);
        for (int i = 0; i >= 0; i = hierarchy_R[i][0])
        {
            if (contoursPoints[i].size() < 5)//轮廓点需要大于4才可使用旋转椭圆

                continue;
            /*
            if(cv::contourArea(contoursPoints[i], false)<5)//临时计算轮廓面积
                {
                    cv::drawContours(temp1,contoursPoints,-1,cv::Scalar(255,255,255));
                    continue;
                }
            */
             cv::RotatedRect rect = cv::fitEllipse(contoursPoints[i]);//旋转椭圆
            std::cout<<"椭圆："<<i+1<<rect.size<<std::endl;
            //cv::putText(temp1,std::to_string(rect.size.height),rect.center,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,0.5,cv::Scalar(0,0,255));
            if(rect.size.width>rect.size.height)std::swap(rect.size.width,rect.size.height);
            float s1 = cv::contourArea(contoursPoints[i], false);//临时计算轮廓面积
            float s2 = rect.size.width * rect.size.height;//临时计算最小外接矩形面积



            //PointLimit(rect.center,AL::Src_1.rows,AL::Src_1.cols);
            if(rect.center.x>Src.cols || rect.center.y > Src.rows)
                continue;
            if (blobJudge(s1, s2, rect))
            {

                if(isTrack)
                    rect.center+=(cv::Point2f)trackBoxLimit.tl();

                blobs.emplace_back(rect,Src);//将灯条添加到容器灯条集合

                float blue=0;
                float red=0;
                for(int a=0;a<contoursPoints[i].size();a++)
                {
                    blue+=(Src.at<cv::Vec3b>(contoursPoints[i][a])[0]+Src.at<cv::Vec3b>(blobs.back().start_point)[0]+Src.at<cv::Vec3b>(blobs.back().end_point)[0]);

                    red+=(Src.at<cv::Vec3b>(contoursPoints[i][a])[2]+Src.at<cv::Vec3b>(blobs.back().start_point)[2]+Src.at<cv::Vec3b>(blobs.back().end_point)[2]);
                }
                if(blue>=red)blobs.back().color=0;
                else blobs.back().color=1;
                if(blobs.back().color!=ENEMY_COLOR)
                    blobs.pop_back();
            }
        }
    }
    else{
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now );
        fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
        fout<<"No Contours in this image"<<std::endl<<std::endl;
#endif
    }

    //cv::imshow("contours",temp1);
#ifdef LOG_DATA
if(blobs.size()==0){
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now );
    fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
    fout<<"Couldn't find lightblob in this image "<<std::endl<<std::endl;
    }

#endif
    std::sort(blobs.begin(),blobs.end(),[&blobs](LightBlob &b1,LightBlob &b2){return b1.priority<b2.priority;});
    lightBlobs.clear();
    lightBlobs=blobs;
}




bool AutoHit::blobMatch(LightBlob &blob1, LightBlob &blob2)
{
    cv::Point2f centers = blob1.rect.center - blob2.rect.center;
    double side_length = sqrt(centers.ddot(centers));
    double side = side_length / blob1.rect.size.height;
    float angle_diff;
    if ((blob1.angle < 90 && blob2.angle < 90)
        ||
        (blob1.angle > 90 && blob2.angle > 90))
        angle_diff = fabsf(blob1.angle - blob2.angle);
    else if (blob1.angle > 90 && blob2.angle < 90)
        angle_diff = 180 - blob1.angle + blob2.angle;
    else
        angle_diff = 180 - blob2.angle + blob1.angle;
    if (angle_diff >= angleDiffBlobPairs) {
#ifdef TEST
        std::cout<<"两灯条角度差不符合要求"<<std::endl;
        cv::putText(SrcShow, "angle_diff", blob1.rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }

    if ((fabsf(blob1.rect.size.height / blob2.rect.size.height) >= 1 / heightRatioBlobPairs) ||
        (fabsf(blob1.rect.size.height / blob2.rect.size.height) <= heightRatioBlobPairs)) {
#ifdef TEST
        std::cout<<"两灯条长度比不符合要求"<<std::endl;
        cv::putText(SrcShow, "length_div", blob1.rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        //return false;
    }


    if (side >distanceBlobPairsMin || side < distanceBlobPairsMax) {
#ifdef TEST
        std::cout << "两灯条距离不符合要求" << std::endl;
        cv::putText(SrcShow, "distance", blob1.rect.center, cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255));
#endif
        return false;
    }

    float t=atan2(fabs(blob1.rect.center.y-blob2.rect.center.y),fabs(blob1.rect.center.x-blob2.rect.center.x))*180/CV_PI;
    if(t>blobSlope)
    {
#ifdef TEST
        std::cout << "两灯条中心角度不符合要求" +std::to_string(t)<< std::endl;
        cv::putText(SrcShow, "Cangle"+std::to_string(t), blob1.rect.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
#endif
        return false;
    }

    /*
    if (blob1.rect.center.x > blob2.rect.center.x)
    {
        if(blob1.end_point.y-blob2.start_point.y<-30||blob1.start_point.y-blob2.end_point.y>30)
        {
#ifdef TEST
            std::cout<<"两灯条错位"<<std::endl;
            cv::putText(SrcShow, "cuowei01", blob1.rect.center, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0255), 1);
#endif
            return false;
        }
    }
    else{
        if(blob2.end_point.y-blob1.start_point.y<-30||blob2.start_point.y-blob1.end_point.y>30)
        {
#ifdef TEST
            std::cout<<"两灯条错位"<<std::endl;
#endif
            return false;
        }
    }
     */
    return true;
}




void AutoHit::armorFind() {
    if (isTrack) {
        cv::Mat T;
    }
    std::vector<LightBlob> blobsBackUp = lightBlobs;

    long t6 = getCurrentTime__();
    armors.clear();//清除上一帧图片内容
    blobsSingle.clear();
    isFindLastID = false;
/*************************选出装甲板待选区域******************************************************************************/
    for (int i = 0; blobsBackUp.size() > 1;)//因为每个前元素匹配均会被去除，因此永远是第一个元素开始匹配
    {
        int TempID = 0;
        //对第一个元素之后的元素按到此装甲板的距离进行升序排序
        std::sort(blobsBackUp.begin(), blobsBackUp.end(), [&blobsBackUp](LightBlob &b1, LightBlob &b2) {
            return sqrt((b1.rect.center - blobsBackUp[0].rect.center).ddot(
                    b1.rect.center - blobsBackUp[0].rect.center)) <
                   sqrt((b2.rect.center - blobsBackUp[0].rect.center).ddot(
                           b2.rect.center - blobsBackUp[0].rect.center));
        });

        if ((LastID == 0 && armors.size() > 2) || (LastID != 0 && armors.back().id == LastID))
            break;//发现上一帧目标或者装甲>=3且上一帧id==0停止匹配
        for (int j = 1; j < blobsBackUp.size(); j++) {

            if (j > 1) {
                if ((blobsBackUp[j - 1].rect.center.x - blobsBackUp[0].rect.center.x) /
                    (blobsBackUp[j].rect.center.x - blobsBackUp[j - 1].rect.center.x) >= 0 && fabsf(fabsf(
                        fabsf(blobsBackUp[j - 1].rect.center.y - blobsBackUp[0].rect.center.y) /
                        fabsf(blobsBackUp[j - 1].rect.center.x - blobsBackUp[0].rect.center.x) -
                        fabsf(blobsBackUp[j].rect.center.y - blobsBackUp[j - 1].rect.center.y) /
                        fabsf(blobsBackUp[j - 1].rect.center.x - blobsBackUp[0].rect.center.x))) <
                                                                                              4) {
                    continue;
                }

            }
            if (blobMatch(blobsBackUp[i], blobsBackUp[j])) {
                armors.emplace_back(blobsBackUp[i], blobsBackUp[j], Src, SrcShow, SVM, isTrack);
                if (armors.back().id != 0) {
                    if (LastID && armors.back().id == LastID)isFindLastID = true;
                    blobsBackUp.erase(blobsBackUp.begin() + j);//去除已匹配成功灯条对
                    TempID = armors.back().id;
                    break;//已匹配成成功防止继续匹配浪费时间
                } else {
                    armors.pop_back();//删除非装甲板灯条对
                    TempID = 0;
#ifdef LOG_DATA
                    now = std::chrono::system_clock::now();
                    now_c = std::chrono::system_clock::to_time_t(now);
                    fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                    fout << "ID=0" << std::endl << std::endl;
#endif
                }
            } else {
#ifdef LOG_DATA
                now = std::chrono::system_clock::now();
                now_c = std::chrono::system_clock::to_time_t(now);
                fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                fout << "Blobs can't be matched as armors" << std::endl << std::endl;
#endif
            }
        }
        if (TempID == 0) {
            blobsSingle.emplace_back(blobsBackUp[0]);//添加用于单灯条匹配
            blobsBackUp.erase(blobsBackUp.begin());//去除已与所有灯条匹配过或匹配成功灯条
            std::sort(blobsBackUp.begin(), blobsBackUp.end(), [&blobsBackUp](LightBlob &b1, LightBlob &b2) {
                return b1.priority < b2.priority;
            });//对剩余元素重新按灯条距屏幕中心距离降序排序
        }
    }
#ifdef LOG_DATA
    if (armors.size() == 0) {
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "Two blobs Match didn't find armors" << std::endl << std::endl;
    }
#endif
    if (blobsBackUp.size() == 1) { blobsSingle.emplace_back(blobsBackUp[0]); }//添加最后一根灯条
    //cv::circle(SrcShow,blobsSingle.back().rect.center,3,cv::Scalar(0,0,255),3);
    std::cout << std::endl << "《《《《灯条对匹配耗时:" << (getCurrentTime__() - t6) / 1000.0 << std::endl;
#ifdef SIGLE_BLOB
    //没找到装甲板或上一帧目标不为0且没发现上一帧目标时进入单灯条模式
    if ((LastID && !isFindLastID) || (LastID && armors.size() == 0 && !blobsSingle.empty())) {
        isSingleBlobMode = true;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "======Come Into SingleBlob Mode======" << std::endl << std::endl;
#endif
        SingBlob();
    } else {
        isSingleBlobMode = false;
    }
#endif
    if (armors.size() > 0) //否则出错
    {
        for (int a = 0; a < armors.size(); a++) {
            armors[a].distancef = Solve(armors[a].blobPointsPairs);
        }


        int t = sortTarget();
        if (armors[t].id != LastID && LastID != 0) {
            notLastIDNums++;
            if (notLastIDNums > 2)//有目标情况下连续3帧没有发现之前目标，切换锁定目标
            {
                LastID = armors[t].id;
                notLastIDNums = 0;
            }
        } else {

            notLastIDNums = 0;
        }
        LastID = armors[t].id;
        TargetPixel = armors[t].center;
        float t1 = (armors[t].blobL.rect.size.height + armors[t].blobR.rect.size.height) / 2.0;
#ifdef TEST
        cv::putText(SrcShow,std::to_string(t1),cv::Point2f(500,160),cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,cv::Scalar(255,255,255));
#endif
#ifdef ARMOR_TEST
        cv::circle(SrcShow, TargetPixel, 8, cv::Scalar(0, 0, 255), 2);
#endif
        Compensate(t1);
        lastdistance=t1;
        TargetAngle = Target2Angle(TargetPixel);
        //!视觉补偿添加处；

        //Compensate(t1);
        //Compensate(armors[t].distancef);
        //Compensate((armors[t].blobL.rect.size.height+armors[t].blobR.rect.size.height)/2.0);

        gtargetsAngle.emplace_back(TargetAngle);
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "Find Target:" + std::to_string(armors[t].id) + ";PixelPoint:" << TargetPixel << std::endl << std::endl;
#endif

#ifdef TRACKER
        TrackInit(armors[t]);
#endif
        noTargetNums = 0;
    } else {
        isTrack = false;
        noTargetNums++;
        if (noTargetNums > 5)//连续5帧没有发现目标，上帧目标记为0，避免一直查找之前目标浪费查找时间
            LastID = 0;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "No Target" << std::endl << std::endl;
#endif
    }
}

void AutoHit::SingBlob()
{
    std::vector<LightBlob>blobs=blobsSingle;
    std::sort(blobs.begin(), blobs.end(), [&blobs](LightBlob &b1, LightBlob &b2) {return b1.rect.size.height > b1.rect.size.height;});
    for(int i=0;i<blobs.size();i++)
    {
        if(blobs[i].rect.size.height>SingleBarHeighLimit)
        {
            float dx=0;
            float dy=0;
            //!沿倾斜角进行推测
            if(blobs[i].angle>CV_PI/2.0)
            {
                float angle = blobs[i].angle + CV_PI;

                dx = blobs[i].rect.size.height * 135.0 / 53.0 * cosf(angle)*0.8;
                dy = blobs[i].rect.size.height * 135.0 / 53.0 * sinf(angle);
            }
            else
            {
                dx = blobs[i].rect.size.height * 135.0 / 53.0 * cosf(blobs[i].angle)*0.8;
                dy = blobs[i].rect.size.height * 135.0 / 53.0 * sinf(blobs[i].angle);
            }
            //假设虚拟灯条在右侧
            cv::Point2f T;
            T.x=blobs[i].rect.center.x+dx;
            T.y=blobs[i].rect.center.y+dy;
            cv::RotatedRect Temp1(T,blobs[i].rect.size,blobs[i].rect.angle);
            LightBlob blobT1(Temp1,Src);
#ifdef TEST
            cv::line(SrcShow,blobT1.start_point,blobT1.end_point,cv::Scalar(0,255,255),2,8);
            cv::putText(SrcShow,"virsual",T,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255),1);
#endif
            armors.emplace_back(blobs[i], blobT1, Src, SrcShow, SVM,isTrack);
            if(armors.back().id==0){
                armors.pop_back();
                //假设虚拟灯条在左侧
                T.x=blobs[i].rect.center.x-dx;
                T.y=blobs[i].rect.center.y-dy;
                cv::RotatedRect Temp2(T,blobs[i].rect.size,blobs[i].rect.angle);
                LightBlob blobT2(Temp2,Src);
#ifdef TEST
                cv::line(SrcShow,blobT2.start_point,blobT2.end_point,cv::Scalar(0,255,255),2,8);
                cv::putText(SrcShow,"virsual",T,cv::FONT_HERSHEY_COMPLEX,0.5,cv::Scalar(0,0,255),1);
#endif
                armors.emplace_back(blobs[i], blobT2, Src, SrcShow, SVM,isTrack);
            }

            if(armors.back().id==0){
                armors.pop_back();
                dx = blobs[i].rect.size.height * 135.0 / 53.0 ;
                T.x=blobs[i].rect.center.x-dx;
                T.y=blobs[i].rect.center.y;
                cv::RotatedRect Temp3(T,blobs[i].rect.size,blobs[i].rect.angle);
                LightBlob blobT3(Temp3,Src);
                armors.emplace_back(blobs[i], blobT3, Src, SrcShow, SVM,isTrack);
                if(armors.back().id==0){
                    armors.pop_back();
                    T.x=blobs[i].rect.center.x+dx;
                    T.y=blobs[i].rect.center.y;
                    cv::RotatedRect Temp4(T,blobs[i].rect.size,blobs[i].rect.angle);
                    LightBlob blobT4(Temp4,Src);
                    armors.emplace_back(blobs[i], blobT4, Src, SrcShow, SVM,isTrack);
                    if(armors.back().id==0)armors.pop_back();
                }
            }
        }
        if((LastID!=0&&armors.back().id==LastID)||(LastID==0&&armors.size()>2))break;
    }
    blobsSingle.clear();
}



int AutoHit::sortTarget()
{
    std::vector<Armor>Armors=armors;
    //按照到屏幕中心点的距离进行排序
    std::sort(Armors.begin(), Armors.end(), [&Armors](Armor &a1, Armor &a2) { return a1.priority < a2.priority; });

    for(int i=0;i<Armors.size();i++)//优先返回上一个目标
    {
        if(Armors[i].id == LastID)
            return i;
    }
    for(int i=0;i<Armors.size();i++)
    {
        if(Armors[i].id==3||Armors[i].id==4||Armors[i].id==5)
            return i;
    }
    for(int i=0;i<Armors.size();i++)
    {
        if(Armors[i].id==2||Armors[i].id==1)
            return i;
    }
    return 0;
}
void AutoHit::antiSpin()
{
}

inline void AutoHit::Compensate(float &distance) {
    //!分段重力补偿
    //!distance值待定，使用PnP或者灯条长度待定
    if (distance > distance1) {
        grade = 1;
        //
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!comgX1:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX1;
        TargetPixel.y += comgY1;
    } else if (distance > distance2 && distance <= distance1) {
        grade = 2;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!comgX2:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX2;
        TargetPixel.y += comgY2;
    } else {
        grade = 3;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!comgX3:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX3;
        TargetPixel.y += comgY3;
    }
    //lastGrade=grade;
    //!待定
}
inline void AutoHit::Compensate(double &distance) {
    //!分段重力补偿
    //!distance值待定，使用PnP或者灯条长度待定
    if (distance < distance1) {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!comgX1:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX1;
        TargetPixel.y += comgY1;
    } else if (distance < distance2 && distance >= distance1) {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!comgX2:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX2;
        TargetPixel.y += comgY2;
    } else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!comgX3:" << distance << ";" << comgX1 << std::endl;
        TargetPixel.x += comgX3;
        TargetPixel.y += comgY3;
    }
    //lastGrade=grade;
    //!待定
}
#ifdef SERIAL
inline int AutoHit::start(cubot::serial_port &sp)
#else
inline int AutoHit::start()
#endif
{
    //TargetPixel=cv::Point2f (-1,-1);
    //cv::waitKey(1);
#ifdef LOG_DATA
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now );
    fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
    fout<<"======Come Into AutoHit Mode======"<<std::endl<<std::endl;
#endif
#ifdef SERIAL
    //cubot::serial_port  sp (sp_callback, "/dev/ttyUSB"+std::to_string(thread_id-1), 115200);
#endif
#ifdef VIDEO_READ
    cv::VideoCapture capture;
    capture.open("/home/robostar/Videos/Realtime_Armor.avi");
#endif
#ifdef ARMOR_TEST
    int receive[3] = {1, 0, 1};
    shm_subscriber<int> mode_sub("car_mode");
    while (char(cv::waitKey(1)) != 27)
#else
        while(1)
#endif

    {
        if(grade!=0)lastGrade=grade;
        grade = 0;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now );
        fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
        fout<<std::endl<<"///***********New Index*************///"<<std::endl<<std::endl;
#endif
        long whieStart = getCurrentTime__();


#ifdef VIDEO_READ
        capture >> Src;
        cv::resize(Src, Src, cv::Size(), 0.5, 0.5);
        cv::cvtColor(Src, Pre, cv::COLOR_BGR2HSV);
        cv::inRange(Pre, cv::Scalar(0, 0, 50), cv::Scalar(180, 255, 255), Pre);
#else
        Src_Sub.get(Src);
        Pre_Sub.get(Pre);
#endif
#ifdef SHOW_THRESH
        //cv::imshow("Pre",Pre);
#endif
        std::cout << "thread_"+std::to_string(thread_id)+"《《《《获取图片耗时:" << (getCurrentTime__() - whieStart) / 1000.0 << "ms" << endl;

        if (Src.empty()) {
            std::cout << "thread_"+std::to_string(thread_id)+"!!!!!!WARNING-NO-IMAGE-INPUT!!!!!!" << std::endl;
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now );
            fout<<std::put_time(std::localtime(&now_c), "%F %T")<<std::endl;
            fout<<"load image failure,Continue"<<std::endl<<std::endl;
#endif
            std::cout<<"load image failure,Continue"<<std::endl<<std::endl;
            continue;
        }
        armors.clear();
        TargetPixel = cv::Point2f(-1, -1);
        gtargetsAngle.clear();
#ifdef SRC_MODE
#ifdef ARMOR_TEST
        cv::resize(Src,SrcShow,cv::Size(),0.5,0.5);
#endif
#else
#ifdef ARMOR_TEST
        SrcShow=Src.clone();
        cv::circle(SrcShow,cv::Point2f(1280/4,1024/4),3,cv::Scalar(0,0,255),2);
#endif
#endif
#ifdef TRACKER
        //isTrack=0;
        //long T_track=getCurrentTime__();
        if (isTrack) {
            std::cout << "======THREAD_" + std::to_string(thread_id) + "_TRACKING__MODE======" << std::endl;
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "|||==Come Into AutoHit::TrackingMode==|||" << std::endl << std::endl;
#endif

#ifdef ARMOR_TEST
#ifdef TEST
            cv::rectangle(SrcShow, trackBoxLimit, cv::Scalar(0, 0, 255), 1);
#endif
#endif
            bool trackStatus=tracker->update(TrackingROI,trackBox);
            //trackBoxBackUp=trackBox;
            //trackBoxBackUp.tl()=trackBox.tl()+trackBoxLimit.tl();
            //trackBoxBackUp.br()=trackBox.br()+trackBoxLimit.tl();
             trackBoxBackUp=cv::Rect2d(trackBox.tl()+trackBoxLimit.tl(),trackBox.br()+trackBoxLimit.tl());
            TargetPixel=(trackBox.tl()+trackBox.br())/2.0+trackBoxLimit.tl();
            Compensate(lastdistance);

            gtargetsAngle.emplace_back(Target2Angle((cv::Point2f)TargetPixel));
            gtargetsAngle.emplace_back(0,lastGrade);
            //PointLimit(trackBoxBackUp.tl(),Pre.rows,Pre.cols);
            //PointLimit(trackBoxBackUp.br(),Pre.rows,Pre.cols);
            cv::rectangle(SrcShow,trackBoxBackUp,cv::Scalar(0,255,255),1);

            TrackGo();
            if(index%10==0||!trackStatus){
                isTrack=0;
                //tracker.reset();
                //tracker.release();
                std::cout<<index%100<<" "<<!trackStatus<<"EXIT TRACKING STATUS "<<std::endl;
            }
        }
#endif
        if (!isTrack) {
#ifdef TEST_OUTPUT
            std::cout << "======THREAD_" + std::to_string(thread_id) + "_SEARCHING__MODE======" << std::endl;
#endif
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "|||==Come Into AutoHit::SearchingMode==|||" << std::endl << std::endl;
#endif
            trackBox = cv::Rect2f(-1, -1, 0, 0);

            long t1 = getCurrentTime__();

            blobFinder();//预处理
#ifdef TEST_OUTPUT
            std::cout << "THREAD_" + std::to_string(thread_id) + "_《《《《发现灯条耗时:" << ((getCurrentTime__() - t1) / 1000.0)
                      << "ms" << endl;
#endif
            long t2 = getCurrentTime__();
            armorFind();//寻找装甲板区域
#ifdef TEST_OUTPUT
            std::cout << "THREAD_" + std::to_string(thread_id) + "_《《《《发现装甲耗时:" << ((getCurrentTime__() - t2) / 1000.0)
                      << "ms" << endl;
#endif
        }
        mode_sub.get(receive);
        if(receive[0]!=1)break;


///////////////帧率控制///////////////////////////////////////////
#ifdef FPS_CONTROL
        long t=getCurrentTime__()-whieStart;
        if((t)/1000.0<9.8)
            usleep((9.8-(t/1000.0))*1000.0);
#endif
#ifdef SERIAL
        //发送信息*******************************************//
        gtargetsAngle.emplace_back(0,grade);
        //gtargetsAngle.emplace_back(0,0);
        int isSpSucss=sp.SerialPublish(gtargetsAngle);
#ifdef LOG_DATA
        fout<<"Final target' pixelPoint:"<<TargetPixel<<std::endl;

#endif

#endif
        long now = getCurrentTime__();
        /******************测试信息编辑区******************************************/
        long temp=now-whieStart;
        fps = 1000000.0 / (temp);
        fpsall += fps;
#ifdef TEST_OUTPUT
        long t4 = getCurrentTime__();
        std::cout << "THREAD_"+std::to_string(thread_id)+"_FRAME:" <<index << std::endl;
        std::cout << "THREAD_"+std::to_string(thread_id)+"_AVERAGE_FPS:" << fpsall / index << std::endl;
        std::cout << "THREAD_"+std::to_string(thread_id)+"_CURRENT_FPS:" << fps << std::endl;
        std::cout << "THREAD_"+std::to_string(thread_id)+"_CURRENT_TIME:" << (temp) / 1000.0 << "ms" << std::endl;
        std::cout << "THREAD_"+std::to_string(thread_id)+"_====END====" << std::endl;
#endif
#ifdef TEST
        if (isSingleBlobMode) {
            cv::putText(SrcShow, "SIGLE_PAIRS :" + std::to_string(index), cv::Point2f(400, 20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 0, 255));;
        } else {
            cv::putText(SrcShow, "PAGES_INDEX:" + std::to_string(index), cv::Point2f(400, 20),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));;
        }
        cv::putText(SrcShow, "CURRENT_FPS:" + std::to_string((int) fps), cv::Point2f(400, 40),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
        cv::putText(SrcShow, "AVERAGE_FPS:" + std::to_string((int) (fpsall / index)), cv::Point2f(400, 60),
                    cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
        cv::putText(SrcShow, "RUNTIME_NOW:" + std::to_string((temp) / 1000.0) + "ms",
                    cv::Point2f(400, 80), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 255));
        if (isTrack) {
            cv::putText(SrcShow, "ANGLE_CAMERA:" + std::to_string((int) gtargetsAngle[0].x) + "," +
                                 std::to_string((int) gtargetsAngle[0].y), cv::Point2f(400, 100),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
            cv::putText(SrcShow, "PIXEL_POINT  :" + std::to_string((int) TargetPixel.x) + "," +
                                 std::to_string((int) TargetPixel.y), cv::Point2f(400, 120),
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255));
        } else {
            cv::putText(SrcShow, "ANGLE_CAMERA:NO_TARGET", cv::Point2f(400, 100), cv::FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0, 255, 255));
            cv::putText(SrcShow, "PIXEL_POINT  :NO_TARGET", cv::Point2f(400, 120), cv::FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0, 255, 255));
        }
        if (isTrack)
            cv::putText(SrcShow, "TRACKING_STATUS", cv::Point2f(400, 140), cv::FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0, 255, 255));
        else
            cv::putText(SrcShow, "SEARCHING_STATUS", cv::Point2f(400, 140), cv::FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0, 0, 255));

#endif
#ifdef SHOW_BLOB_INCLUDING
        cv::Point2f vertices[4];
        for(int i=0;i<lightBlobs.size();i++){
            lightBlobs[i].rect.points(vertices);
            for(int j=0;j<4;j++){
                if(!isTrack)
                    cv::line(SrcShow, vertices[j], vertices[(j + 1) % 4], cv::Scalar(255, 0, 0), 1);
                else
                    cv::line(SrcShow, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 0, 255), 1);
            }
        }
#endif
#ifdef  ARMOR_TEST
        cv::circle(SrcShow, TargetPixel, 10, cv::Scalar(255, 255, 255), 2);
        cv::imshow("srcShow"+std::to_string(thread_id), SrcShow);
        //SrcShow_backup=SrcShow.clone();
#endif
        index++;
#ifdef TEST_OUTPUT
        std::cout << "《《《《《测试输出耗时:" << ((getCurrentTime__() - t4) / 1000.0) << "ms" << endl;
#endif
        std::cout << std::endl << std::endl << std::endl;
#ifdef  LOOKONCE
        cv::waitKey(0);
#endif
    }
    return 0;
}


cv::Point2f AutoHit::Target2Angle(cv::Point2f target)
{
    if(target.x<0||target.y<0)
        return cv::Point2f(0,0);


    target*=2;//因为源图像被缩放了1/2，所以要乘二，否则调用的像素坐标系都小了1/2
    Eigen::Vector3d  targetCam;
    Eigen::Vector3d  pixel;
    pixel << target.x, target.y, 1.0;//像素平面坐标

    targetCam=cameraMatrixe.inverse() * pixel;

    //畸变校正；
    double r2 = pow(targetCam.x(), 2) + pow(targetCam.y(), 2);

    targetDistotrted.x() = targetCam.x() * ( 1 + disCoeffse[0] * pow( r2, 1 ) + disCoeffse[1] * pow( r2, 2 ) + disCoeffse[2] * pow( r2, 3 ) )
                           + 2 * disCoeffse[3] * targetCam.x() * targetCam.y() + disCoeffse[4] * (r2 + 2 * pow(targetCam.x(), 2 ) );

    targetDistotrted.y() = targetCam.y() * ( 1 + disCoeffse[0] * pow( r2, 1 ) + disCoeffse[1] * pow( r2, 2 ) + disCoeffse[2] * pow( r2, 3 ) )
                           + disCoeffse[3] * ( r2 + 2 * pow( targetCam.y(), 2 ) ) + 2 * disCoeffse[4] * targetCam.x() * targetCam.y();
    targetDistotrted.z() = 1;

    Eigen::Vector3d cannon = calMatrix.rotation().inverse() * (targetDistotrted - calMatrix.translation());

    cv::Point2f angle;
    angle.x=atan2(cannon[0],1)*180/CV_PI/1.0;
    angle.y=atan2(cannon[1],1)*180/CV_PI/1.0;
    return angle;

}



/************************灯条对类构造函数**********************************************************************************/
Armor::Armor(LightBlob &gblob1, LightBlob &gblob2, cv::Mat &src, cv::Mat &SrcShow, cv::Ptr<cv::ml::SVM> SVM,bool &isTrack): blobL(gblob1), blobR(gblob2)//不给构造函数引入源图像src可能会导致src为空
{
    center= (blobL.rect.center + blobR.rect.center) / 2.0;
    priority=sqrtf(pow((center.x- src.cols / 2.0), 2) + pow((center.y - src.rows), 2));//到屏幕中心距离，以像素为单位

#ifdef WITHOUT_BLOB
    float k=0.5;
    float dx,dy;
    if(blobL.start_point.x > blobR.start_point.x)
    {
        LightBlob temp=blobL;
        blobL=blobR;
        blobR=temp;
    }
    //!姿态解算所需像素平面坐标
    /*********************************************/
    blobPointsPairs.emplace_back(blobL.start_point);
    blobPointsPairs.emplace_back(blobR.start_point);
    blobPointsPairs.emplace_back(blobR.end_point);
    blobPointsPairs.emplace_back(blobL.end_point);
    /*********************************************/
    //distancef=AutoHit::Solve(blobPointsPairs);//距离估测
    //确定装甲板左侧坐标
    dx= blobL.end_point.x - blobL.start_point.x;
    dy= blobL.end_point.y - blobL.start_point.y;

    roi_tl.x= blobL.start_point.x - k * dx + blobL.rect.size.width;
    roi_tl.y= blobL.start_point.y - k * dy;

    roi_bl.x= blobL.end_point.x + k * dx + blobL.rect.size.width;
    roi_bl.y= blobL.end_point.y + k * dy;

    //右侧
    dx= blobR.end_point.x - blobR.start_point.x;
    dy= blobR.end_point.y - blobR.start_point.y;

    roi_tr.x= blobR.start_point.x - k * dx;// - blobR.rect.size.width;
    roi_tr.y= blobR.start_point.y - k * dy;

    roi_br.x= blobR.end_point.x + k * dx;// - blobR.rect.size.width;
    roi_br.y= blobR.end_point.y + k * dy;
#ifdef SRC_MODE
    PointLimit(roi_bl, src.rows/2.0, src.cols/2.0);
    PointLimit(roi_br, src.rows/2.0, src.cols/2.0);
    PointLimit(roi_tl, src.rows/2.0, src.cols/2.0);
    PointLimit(roi_tr, src.rows/2.0, src.cols/2.0);
    //开始进行透视变换
#else
    PointLimit(roi_bl, src.rows, src.cols);
    PointLimit(roi_br, src.rows, src.cols);
    PointLimit(roi_tl, src.rows, src.cols);
    PointLimit(roi_tr, src.rows, src.cols);
#endif

    cv::Point2f armor_src[4],armor_transform[4];
#ifdef SRC_MODE
    armor_src[0]=roi_tl*2;armor_src[1]=roi_tr*2;
    armor_src[3]=roi_bl*2;armor_src[2]=roi_br*2;
#else
    armor_src[0]=roi_tl;armor_src[1]=roi_tr;
    armor_src[3]=roi_bl;armor_src[2]=roi_br;
#endif
    armor_transform[0]=cv::Point2f(0,0);armor_transform[1]=cv::Point2f(40,0);
    armor_transform[2]=cv::Point2f(40,40);armor_transform[3]=cv::Point2f(0,40);


#ifdef TRACKER
    /*******************************************************/
//!选取追踪区域
/*
    roi_tl.x=std::min(roi_bl.x,roi_tl.x);
    roi_tl.y=std::min(roi_tl.y,roi_tr.y);
    roi_br.x=std::max(roi_br.x,roi_tr.x);
    roi_br.y=std::max(roi_br.y,roi_bl.y);
*/
    roi_tl.x=std::min(blobL.start_point.x,blobL.end_point.x);
    roi_tl.y=std::min(blobL.start_point.y,blobR.start_point.y);
    roi_br.x=std::max(blobR.start_point.x,blobR.end_point.x);
    roi_br.y=std::max(blobL.end_point.y,blobR.end_point.y);
    rect=cv::Rect2d(roi_tl,roi_br);
    /************************************************/
#endif
    //进行透视变换

    cv::Mat Transform=cv::getPerspectiveTransform(armor_src,armor_transform);
    cv::warpPerspective(src,roi,Transform,cv::Size(40,40));
#ifdef train_date_make
    for(int i=0;i<4;i++){
        cv::line(SrcShow,armor_src[i%4],armor_src[(i+1)%4],cv::Scalar(255,255,2));
    }
    cv::imshow("Srfc",SrcShow);
    cv::Mat roi_train_data=roi.clone();
#endif


    RoiProcesing(roi);//尝试对ROI区域重新进行形态学处理

    std::vector<float>descriptors;
    //hog=cv::HOGDescriptor (cv::Size(40,40),cv::Size(16,16),cv::Size(8,8),cv::Size(8,8),9);
    cv::HOGDescriptor hog=cv::HOGDescriptor (cv::Size(40,40),cv::Size(16,16),cv::Size(8,8),cv::Size(8,8),9);

    hog.compute(roi,descriptors);
    cv::Mat dst(1 ,int(descriptors.size()), CV_32FC1, descriptors.data());
    //roi=cv::Mat(1, int(descriptors.size()), CV_32FC1, descriptors.data());
    id=SVM->predict(dst);
    //id=3;
    std::cout<<"id:"<<id<<std::endl;
#ifdef train_date_make
    std::chrono::system_clock::time_point now;
    std::time_t now_c;
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now );
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    if(id!=0){
        cv::imwrite("/home/shuai/trainData4.0/0/true/"+str_time+".bmp",roi_train_data);
        RoiProcesing(roi_train_data);
        cv::imwrite("/home/shuai/trainData4.0/0/processing/true/"+str_time+".bmp",roi_train_data);

    }
    else {
        cv::imwrite("/home/shuai/trainData4.0/0/false/"+str_time+".bmp",roi_train_data);
        RoiProcesing(roi_train_data);
        cv::imwrite("/home/shuai/trainData4.0/0/processing/false/"+str_time+".bmp",roi_train_data);

    }

#endif

#ifdef ARMOR_TEST
    if(id!=0) //若id为0，则说明不是装甲;
    {
#ifdef TEST
        cv::putText(SrcShow, std::to_string((int)distancef) + "cm", center, cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(50, 100, 200), 1);
#endif
#ifdef ARMOR_RECT
        //标注灯条对四个点位置所用
        for(int i=0;i<4;i++)
        {


            cv::putText(SrcShow, std::to_string(i), armor_src[i], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
            cv::line(SrcShow, armor_src[i % 4], armor_src[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
        }
#endif
        if(isTrack)
            cv::putText(SrcShow, std::to_string(id), (blobL.rect.center + blobR.rect.center) / 2.0, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 255));
        else
            cv::putText(SrcShow, std::to_string(id), (blobL.rect.center + blobR.rect.center) / 2.0, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0));

    }
#ifdef TEST_ZERO
    else//标注分类为0灯条类
    {
        /*
        for(int i=0;i<4;i++)
        {

            cv::putText(SrcShow, std::to_string(i), armor_src[i], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
#ifdef ARMOR_RECT
            cv::line(SrcShow, armor_src[i % 4], armor_src[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
#endif
        }
         */
        cv::line(SrcShow,blobL.rect.center,blobR.rect.center,cv::Scalar(0,0,255),1);
        cv::putText(SrcShow, std::to_string(id), (blobL.rect.center + blobR.rect.center) / 2.0, cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 0));
    }
#endif
#endif
#endif
}
