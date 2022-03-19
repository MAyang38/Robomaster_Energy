//
// Created by mayang on 2020/8/2.
//
#include<opencv2/highgui.hpp>
#include<eigen3/Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include<iostream>
#include <opencv2/ml.hpp>
#include "shm.hpp"

using namespace std;
using namespace cv;
#define pi 3.1415926
//////////////////相机坐标系////////
struct s_pixel
{
    cv::Point2f center_point;
    cv::Point2f armor_point;
};
/////////////////极坐标系///////////
struct s_polar
{   cv::Point2f cc_armor_point;
    double polar_radius;
    double polar_angle;
};
static void pub_callback(const std::vector<uint8_t> &data)
{
    if( data.size() > 0 )
        std::cout << "publish EP has received data...\n";
}
static void sp_callback(const uint8_t *recv_buf){}//缓冲区
///////////获取时间//////////////////////
inline long getCurrentTime()

{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000000+tv.tv_usec;
}




///////////////能量机关类/////////////////
class Energy
{
public:
    Energy(subscriber src);                                        ////构造函数,初始化固定参数
#ifdef SERIAL
    void Task(cubot::serial_port &sp);
#else
    void Task();                                                    /////主任务
#endif
    void Energy_detection(cv::Mat &src,cv::Point2f &point);                       /////检测大符
    void windmill_find(cv::Mat &image,cv::Point2f &result_p);
    void image_Preprocesse(cv::Mat &image);                                       //////图片预处理
    void find_All_Counters(cv::Mat &image1,cv::Point2f &point);
    void find_All_Counters1(cv::Mat &image1,cv::Point2f &point) ;                 ////寻找轮廓
    int EA_SVM_detect(cv::Mat &img);                                              ////SVM判断中心点

    cv::Point2f Pixel2Angle(cv::Point2f &connor);                                 //////点变成圆上的角度
    void pixel_to_polar( struct s_pixel &pixel_list, struct s_polar &polar_list );////相机坐标转换为极坐标
    double polar_angle_transformation( double angle );                            ////将点的角度转换为360°内
    ////////////////////将装甲板上的点转换为以0为圆心的圆上点,便于后期角度补偿
    cv::Point2f cartesian_coordinates_transformation(cv::Point2f &center_point, cv::Point2f &armor_center_point );

    cv::Point2f armor_predict_angle(cv::Point2f &armor,cv::Point2f &center);      /////////角度补偿
    cv::Point2f armor_predict_gravity(cv::Point2f &sp,double angle);               ////////重力补偿
    double Big_predict_angle();
    static bool ContoursSortFun(vector<cv::Point> contour1, vector<cv::Point> contour2) ;//////面积排序


private :
    int color ;                       //击打的颜色
    cv::Mat imagesrc,binary,tempimage,image_armor,image_kai;
    cv::Point2f result_sp;

    ///////////////////////SVM参数//////////////////
    cv::HOGDescriptor *pHog;
    cv::Ptr<cv::ml::SVM> model_hog;

    ///////角度转换参数
    Eigen::Isometry3d calMatrix;
    Eigen::Matrix3d cameraMatrix;
    Eigen::Matrix<double, 5, 1> distCoeffs;

    int Rotation_order;                        ////////旋转方向    0顺1逆
    int flag;                                  ////////是否识别标志位
    cv::Mat cvCameraMatrix,  cvDistCoeffs, cvCalMatrix;

    subscriber Src_Sub;                        ///相机
    //////decetion
    vector<Mat> channels;

    ////////设置内核
    Mat element1,element2;                     ////开闭运算内核
    ////Find_all_contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    int center_rm_founded_flag;
    cv::RotatedRect min_center_rect;
    cv::Point2f  RM_center,armor_center;

    cv::RotatedRect min_RM_rect;
    cv::RotatedRect min_armor_rect;
    cv::Rect center_roi;
    cv::Mat center_bgr_roi;

    int center_svm_result;

    cv::Point2f predicttemp;
    s_pixel pixel_list;
    s_polar polar_list;
    double area_of_contour;
    vector<Point> points;
    double center_rect_h_w_div;
    cv::Point2f vertices[4];


    double curAngle;                                       /////旋转补偿角度
    vector<Point> speeds;                                 /////存放几个点 判定转向
    cv::Point2f last_point,now_point;                    /////第一个点,最后一个点
    double time_left;                                    /////比赛剩余时间
    int BIG_Flag;                                        /////大能量机关是否处于可激活状态
    int B_S;                                            //判断是大能量还是小能量   0小1大
    double Big_time_start,Big_time;
};
#ifdef SERIAL
void Energy::Task(cubot::serial_port &sp)
#else
void Energy::Task()
#endif
{
    int receive[3] = {2, 0, 1};
#ifdef video_test
    VideoCapture video("/home/mayang/xiangmu/Windmills-master/wind.mp4");//打开视频
#endif
//#ifdef camera_test
    shm_subscriber<int> mode_sub("car_mode");
//#endif
    while (1)
    {
        result_sp.x=0;
        result_sp.y=0;
        flag=0;
        /////
#ifdef video_test
           video >> imagesrc;
           if (imagesrc.empty())
               break;
#endif
        long whileStart=getCurrentTime();
       ////////读相机
#ifdef camera_test
       Src_Sub.get(imagesrc);
       if(!imagesrc.data)
           continue;
#endif
       // if(time_left<240)
            B_S=1;                              ////旋转模式初始化
        Rotation_order=0;                       /////旋转方向初始化
        image_armor = imagesrc.clone();         ////寻找装甲板的图

        ////////SRC为找中心点的图   armor为找装甲板的图  binary为show图
        imagesrc.copyTo(binary);
#ifdef show
        //imshow("原图",imagesrc);
        //cv::waitKey(1);
#endif
        Energy_detection(imagesrc,result_sp);
        imshow("寻找打击点", binary);
        waitKey(1);

#ifdef SERIAL

        std::vector<cv::Point2f> gtargetsAngle;//!
        gtargetsAngle.emplace_back(result_sp);
        gtargetsAngle.emplace_back(0,flag);
        //!
        long t=getCurrentTime()-whileStart;
        std::cout <<"TIME"<<t<< std::endl;
        ///////////////睡眠时间    保证帧率
        if((t)/1000.0<7.8)
            usleep((7.8-(t/1000.0))*1000.0);
        //gtargetsAngle.emplace_back(0,0);
        int isSpSucss=sp.SerialPublish(gtargetsAngle);
#endif
//        mode_sub.get(receive);
        //if(receive[0]!=2)break;

        long whileEnd = getCurrentTime();
        long Time = whileEnd - whileStart;
        float fps = 1000000.0 / Time;
        std::cout <<"TIME"<<Time<< "FPS:" << fps << std::endl;
     //   waitKey(1);
    }

}
Energy::Energy(subscriber src):Src_Sub(src)
{
    cv::FileStorage fs("../Config/Camera/armor_cam_para_cfg.yml", cv::FileStorage::READ);
    if(fs.isOpened())
    {
        fs["cameraMatrix"] >>  cvCameraMatrix;
        fs["distCoeffs"] >>  cvDistCoeffs;
        fs["calMatrix"] >>  cvCalMatrix;
        cv::cv2eigen(cvCalMatrix, calMatrix.matrix());
        cv::cv2eigen(cvCameraMatrix, cameraMatrix);
        cv::cv2eigen(cvDistCoeffs, distCoeffs);

        fs.release();
   }
    cv::FileStorage file_1( "../Config/Energy/WindMillCfg.yml", cv::FileStorage::READ );
    if(file_1.isOpened())
    {
        string svm_model_path = (std::string) file_1["svm_model_path"];
        color = (int) file_1["color2Strike"];
        model_hog = cv::ml::SVM::load(svm_model_path);
        pHog = new cv::HOGDescriptor(cv::Size(40, 40), cv::Size(16, 16), cv::Size(8, 8), cv::Size(8, 8), 9);
        file_1.release();
    }
    Rotation_order =-1;
    center_rm_founded_flag=0;
    B_S=1;                                                                  //////能力机关模式
    BIG_Flag=0;                                                             /////大符是否激活
    element1 = getStructuringElement(MORPH_RECT, Size(5, 5));//设置内核1
    element2 = getStructuringElement(MORPH_RECT, Size(25, 25));//设置内核2
    cout<<"能量机关初始化成功"<<endl;
}
void Energy::Energy_detection(cv::Mat &image,cv::Point2f &point) {
#ifdef video_test
    resize(image, image, Size(binary.cols * 0.5, binary.rows * 0.5));
#endif
  ///  double t1 = (double) getTickCount();

    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    dilate(image, image, element);
    split(image, channels);
    if (color == 0) {
        tempimage = channels.at(0) - channels.at(2);
    }
    else
    {       tempimage = channels.at(2) - channels.at(0);

    }

    //imshow("分离",tempimage);
#ifdef video_test
    resize(binary, binary, Size(binary.cols * 0.5, binary.rows * 0.5));
#endif
    windmill_find(tempimage,point);
    //return point;
//   cout<<"图像预处理time:"<<(t2-t1)*1000/(getTickFrequency())<<endl;
}
void Energy::windmill_find(cv::Mat &image,cv::Point2f &result_p)
{
    ///////预处理
    image_Preprocesse(image);
    ///////寻找轮廓
    find_All_Counters(image_kai,result_p);
  //  find_armor(image1,)
    //point_process()

}
void Energy::image_Preprocesse(cv::Mat &image)
{

    threshold(channels.at(0) - channels.at(2), image_armor, 100, 255, THRESH_BINARY_INV);//二值化
    morphologyEx(image_armor, image_armor, MORPH_OPEN, element1);//开运算
    image_armor.copyTo(image_kai);
    floodFill(image_armor, Point(0, 0), Scalar(0));//漫水
    morphologyEx(image_armor, image_armor, MORPH_CLOSE, element2);//闭运算
#ifdef show
    //imshow("闭运算",image_armor);
#endif    /////////////////////////////新///////////////////
}

void Energy::find_All_Counters(cv::Mat &image1,cv::Point2f &point)
{
    /////////////////读图起始时间////////

#ifdef show
    //imshow("image1",image1);
#endif
    center_rm_founded_flag = 0;
    /// 寻找轮廓
    findContours(image1, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    /// 绘出轮廓

#ifdef show
//    Mat drawing = Mat::zeros(image1.size(), CV_8UC3);
//    for (int i = 0; i < contours.size(); i++)
//    {
//        //cout << "第" << i << "个面积" << contourArea(contours[i]) << endl;
//        Scalar color = Scalar(0, 0, 255);
//        drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
//    }
//    imshow("轮廓",drawing);
#endif

    ///////////////////  面积大小排序轮廓 ///////////////////////
    std::sort(contours.begin(), contours.end(), ContoursSortFun);

    ////////////////第一张图找中心点
    for (int i = 0; i < contours.size(); ++i)
    {
        ////////////////////////写出每个轮廓的面积和长宽比
        area_of_contour = contourArea(contours[i]);
        if (contourArea(contours[i]) < 10)
            continue;
        if(contourArea(contours[i])>300)
            break;
        min_center_rect = cv::minAreaRect(contours[i]);
        center_rect_h_w_div = min_center_rect.size.height / min_center_rect.size.width;
        center_rect_h_w_div = center_rect_h_w_div > 1 ? center_rect_h_w_div : 1 / center_rect_h_w_div;


       // cout << "中心图第" << i << "个面积" << area_of_contour << "长宽比" << center_rect_h_w_div << endl;
        ////找中心点rm
        if ((area_of_contour >= 70) && (area_of_contour <= 180) && center_rect_h_w_div > 0.7 &&center_rect_h_w_div < 1.5 )
        {
            min_RM_rect = min_center_rect;
            // cv::circle(binary,Point(min_center_rect.center.x,min_center_rect.center.y),80,cv::Scalar(0,0,255),4);

            //////////////////////////////////找到和中心点大小相同的之后判断是否是中心点///////////////////////////
            //////////////////////////////////// 判断是否符合 SVM 要求  ///////////////////////////////////////
//            center_roi = min_RM_rect.boundingRect();
//            center_roi = center_roi + cv::Point(-3, -3) + cv::Size(6, 6);
//            try
//            {
//                center_bgr_roi = binary(center_roi);
//            }
//            catch (cv::Exception)
//            {
//                continue;
//            }
//            center_bgr_roi = binary(center_roi);
            center_svm_result=1;
            //center_svm_result = EA_SVM_detect(center_bgr_roi);
          //  cout << "SVM预测值  " << center_svm_result;
            if (center_svm_result != 1)
            {
                std::cout << "center SVM false" << std::endl;
                continue;
            }
            RM_center=min_center_rect.center;
            RM_center.y+=3;
#ifdef  show
            cv::circle(binary,Point (RM_center.x,RM_center.y),3,cv::Scalar(0,0,255),2);
#endif
            center_rm_founded_flag = 1;
            if(B_S==1)
            {
                if (BIG_Flag == 0)
                {
                    Big_time_start= (double)getTickCount();
             //       cout<<"开始时间"<<Big_time_start<<endl;
                    BIG_Flag=1;
                }

            }
            break;
            //continue;
        }

    }
    //////////////////开始找装甲板
    if(center_rm_founded_flag==1)
    {
        findContours(image_armor, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        std::sort(contours.begin(), contours.end(), ContoursSortFun);
        for (int i = 0; i < contours.size(); i++)
        {
            if (area_of_contour < 10)
                continue;
            if(area_of_contour>800)
                break;
            area_of_contour = contourArea(contours[i]);
            center_rect_h_w_div = min_center_rect.size.height / min_center_rect.size.width;
            center_rect_h_w_div = center_rect_h_w_div > 1 ? center_rect_h_w_div : 1 / center_rect_h_w_div;
        //    cout << "第" << i << "个面积" << area_of_contour << "长宽比" << center_rect_h_w_div << endl;


            if ( area_of_contour> 100&&area_of_contour< 700&& center_rect_h_w_div > 0.7 &&center_rect_h_w_div < 2 )
            {

                Point2f rect[4];

                min_armor_rect = minAreaRect(Mat(contours[i]));//获取最小外接矩阵

#ifdef show

//                circle(binary, Point(min_armor_rect.center.x, min_armor_rect.center.y), 5, Scalar(255, 0, 0), -1,
//                       8);  //绘制最小外接矩形的中心点
//                min_armor_rect.points(rect);  //把最小外接矩形四个端点复制给rect数组
//
//                for (int j = 0; j < 4; j++)
//                {
//                       line(binary, rect[j], rect[(j + 1) % 4], Scalar(0, 255, 0), 2, 8);  //绘制最小外接矩形每条边
//                }
#endif
                armor_center = min_armor_rect.center;
                ///////////////判定旋转方向//////////////
                if (Rotation_order == -1) {
                    if ((int) speeds.size() < 5) {
                        speeds.push_back(armor_center);
                        continue;
                    } else {
                        last_point = speeds[0];
                        now_point = speeds[4];
                        last_point = last_point - RM_center;
                        now_point = now_point - RM_center;
                        last_point.y = -last_point.y;
                        now_point.y = -now_point.y;
                        if (last_point.x * now_point.y - last_point.y * now_point.x < 0)
                            Rotation_order = 0;
                        else
                            Rotation_order = 1;
                 //       cout << "旋转方向" << Rotation_order << endl;

                    }
                }
                ///////////已经找到中心点和装甲板的点//////////////////
                ////////////RM_center  与 armor_center/////////////
                if (B_S == 1)
                {      // Big_time_now = (double) getCurrentTime();
                        Big_time = ((double)getTickCount() - Big_time_start) / getTickFrequency();
                       cout<<"已经运行时间 "<<Big_time<<endl;
                }
#ifdef show
                circle(binary, Point(armor_center.x, armor_center.y), 5, Scalar(255, 0, 0), -1, 8);  //绘制最小外接矩形的中心点
#endif
                ///////////////////////////角度预测///////////////////////
#ifdef angle_test
                armor_center = armor_predict_angle(RM_center,armor_center);
#endif
                //                    ////////////////////////////////转换极坐标
                pixel_list.center_point = RM_center;
                // pixel_list.armor_point  = min_center_rect.center;
                pixel_list.armor_point  = armor_center;
                pixel_to_polar( pixel_list, polar_list );
             //   std::cout << " polar_angle-------------------:  " << polar_list.polar_angle << std::endl;
                /////////////////////////////重力补偿&&弹道补偿/////////////////////////
                point = armor_predict_gravity(pixel_list.armor_point,polar_list.polar_angle);

#ifdef show
                //cv::circle(binary,Point (RM_center.x,RM_center.y),polar_list.polar_radius,cv::Scalar(0,0,255),4);
                circle(binary, Point(point.x, point.y), 5, Scalar(0, 255, 0), -1, 8);  //绘制最小外接矩形的中心点
#endif
                point=point*2;
                point = Pixel2Angle(point);
                point.x=point.x*0.60;
                point.y=point.y*0.50;
                flag = 1;

#ifdef show
                char text[255];
                sprintf(text, "angle %f", polar_list.polar_angle);
                putText (binary, text, Point(50,50),cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),1,5);
                //////////////////////////////////////////////////附带视频文


#endif
               break;
            }

        }

    }
    else

        cout<<"未找到中心点"<<endl;
}


int Energy::EA_SVM_detect(cv::Mat &img)
{
    int result = 0;
    cvtColor(img, img, COLOR_BGR2GRAY);
    imshow("grey",img);

    vector<float> des_temp;
    resize(img, img, Size(40, 40));
    pHog->compute(img, des_temp, Size(1, 1), Size(0, 0));

    result = (int)model_hog->predict(des_temp);

    return result;
}
////////////////////////图像上点转换到云台需要转动角度值
cv::Point2f Energy::Pixel2Angle(cv::Point2f &connor)
{
    Eigen::Vector3d pixel;
    Eigen::Vector3d P_cam, P_distortion;
    pixel << connor.x, connor.y, 1;
    P_cam = cameraMatrix.inverse() * pixel;
    double r2 = pow(P_cam.x(), 2) + pow(P_cam.y(), 2);
    P_distortion.x() = P_cam.x() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + 2 * distCoeffs[3] * P_cam.x() * P_cam.y() + distCoeffs[4] * (r2 + 2 * pow(P_cam.x(), 2 ) );

    P_distortion.y() = P_cam.y() * ( 1 + distCoeffs[0] * pow( r2, 1 ) + distCoeffs[1] * pow( r2, 2 ) + distCoeffs[2] * pow( r2, 3 ) )
                       + distCoeffs[3] * ( r2 + 2 * pow( P_cam.y(), 2 ) ) + 2 * distCoeffs[4] * P_cam.x() * P_cam.y();
    P_distortion.z() = 1;

    Eigen::Vector3d P_hold = calMatrix.rotation().inverse() * (P_distortion - calMatrix.translation());

    cv::Point2f angles;

    angles.x =  (atan2(P_hold(0), 1) * 180.0 / M_PI)/1.1;

    angles.y = -(atan2(-P_hold(1), 1) * 180.0 / M_PI)/1.1;

    std::cout << "angles:   " << angles << std::endl;

    return angles;
}
//////////////////////图像坐标系转换到极坐标
void Energy::pixel_to_polar( struct s_pixel &pixel_list, struct s_polar &polar_list )
{
    cv::Point2f cartesian_coordinates_armor_center_point;
    double polar_r, polar_angle;
    cartesian_coordinates_armor_center_point = cartesian_coordinates_transformation( pixel_list.center_point, pixel_list.armor_point );

    polar_r = sqrt(pow(cartesian_coordinates_armor_center_point.x, 2) +pow(cartesian_coordinates_armor_center_point.y, 2));
    polar_angle = 57.2957805 * atan2(cartesian_coordinates_armor_center_point.y, cartesian_coordinates_armor_center_point.x);
    polar_angle = polar_angle_transformation( polar_angle );
    polar_list.polar_angle = polar_angle;
    polar_list.cc_armor_point = cartesian_coordinates_armor_center_point;
    polar_list.polar_radius = polar_r;
}

double Energy::polar_angle_transformation( double angle )
{
    if (angle < 0)
    {
        angle = 360 + angle;
    }
    return angle;
}

cv::Point2f Energy::cartesian_coordinates_transformation(cv::Point2f &center_point, cv::Point2f &armor_center_point )
{
    cv::Point2f cartesian_coordinates_armor_center_point(-1, -1);
    cartesian_coordinates_armor_center_point.x = armor_center_point.x - center_point.x;
    cartesian_coordinates_armor_center_point.y = -armor_center_point.y + center_point.y;
    return cartesian_coordinates_armor_center_point;
}

bool Energy::ContoursSortFun(vector<cv::Point> contour1, vector<cv::Point> contour2) {
    return (cv::contourArea(contour1) < cv::contourArea(contour2));
}
////////////////////////////重力和弹道补偿
cv::Point2f Energy::armor_predict_gravity(cv::Point2f &sp,double angle)

{
    int predict_gravity_x=0;
    int predict_gravity_y=0;
    if ( angle >= 0 && angle < 10 )
    {
        sp.x = sp.x + predict_gravity_x +  10;
        sp.y = sp.y + predict_gravity_y -70;
    }
    else if ( angle >= 10 && angle < 20  )
    {
        sp.x = sp.x + predict_gravity_x+18;
        sp.y = sp.y + predict_gravity_y -77;    }
    else if ( angle >= 20 && angle < 30  )
    {
        sp.x = sp.x + predict_gravity_x +16;
        sp.y = sp.y + predict_gravity_y -72 ;
    }
    else if ( angle >= 30 && angle < 40 )
    {
        sp.x = sp.x + predict_gravity_x +16;
        sp.y = sp.y + predict_gravity_y-72;
    }
    else if ( angle >= 40 && angle < 50 )
    {
        sp.x = sp.x + predict_gravity_x +16;
        sp.y = sp.y + predict_gravity_y -70;
    }
    else if ( angle >= 50 && angle < 60 )
    {
        sp.x = sp.x + predict_gravity_x +12;
        sp.y = sp.y + predict_gravity_y - 76;
    }
    else if ( angle >= 60 && angle < 70 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -74;
    }
    else if ( angle >= 70 && angle < 80)
    {
        sp.x = sp.x + predict_gravity_x + 10;
        sp.y = sp.y + predict_gravity_y -72;
    }
    else if ( angle >= 80 && angle < 90)
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -76;
    }
    else if ( angle >= 90 && angle < 100 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y - 75 ;
    }
    else if ( angle >= 100 && angle < 110 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y  - 78;
    }
    else if ( angle >= 110 && angle < 120 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -75;
    }
    else if ( angle >= 120 && angle < 130 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -75;
    }
    else if ( angle >= 130 && angle < 140 )
    {
        sp.x = sp.x + predict_gravity_x +8;
        sp.y = sp.y + predict_gravity_y -74 ;
    }
    else if ( angle >= 140 && angle < 150 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y -70;
    }
    else if ( angle >= 150 && angle < 160 )
    {
        sp.x = sp.x + predict_gravity_x + 7 ;
        sp.y = sp.y + predict_gravity_y -68;
    }
    else if ( angle >= 160 && angle < 170 )
    {
        sp.x = sp.x + predict_gravity_x + 10 ;
        sp.y = sp.y + predict_gravity_y -67;
    }
    else if ( angle >= 170 && angle < 180 )
    {
        sp.x = sp.x + predict_gravity_x + 8;
        sp.y = sp.y + predict_gravity_y -65;
    }
    else if ( angle >= 180 && angle < 190 )
    {
        sp.x = sp.x + predict_gravity_x +8 ;
        sp.y = sp.y + predict_gravity_y - 65;
    }
    else if ( angle >= 190 && angle < 200 )
    {
        sp.x = sp.x + predict_gravity_x + 8;
        sp.y = sp.y + predict_gravity_y - 65;
    }
    else if ( angle >= 200 && angle < 210 )
    {
        sp.x = sp.x + predict_gravity_x +5;
        sp.y = sp.y + predict_gravity_y-65;
    }
    else if ( angle >= 210 && angle < 220 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y- 66;
    }
    else if ( angle >= 220 && angle < 230 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y -65;
    }
    else if ( angle >= 230 && angle < 240 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y -65;
    }
    else if ( angle >= 240 && angle < 250 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y -65;
    }
    else if ( angle >= 250 && angle < 260 )
    {
        sp.x = sp.x + predict_gravity_x + 6;
        sp.y = sp.y + predict_gravity_y - 65;
    }
    else if ( angle >= 260 && angle < 270 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y - 65;
    }
    else if ( angle >= 270 && angle < 280 )
    {
        sp.x = sp.x + predict_gravity_x +6;
        sp.y = sp.y + predict_gravity_y -60;
    }
    else if ( angle >= 280 && angle < 290 )
    {
        sp.x = sp.x + predict_gravity_x +12;
        sp.y = sp.y + predict_gravity_y  - 62;
    }
    else if ( angle >= 290 && angle < 300 )
    {
        sp.x = sp.x + predict_gravity_x + 12;
        sp.y = sp.y + predict_gravity_y  - 58;

    }
    else if ( angle >= 300 && angle < 310 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -60;
    }
    else if ( angle >= 310 && angle < 320 )
    {
        sp.x = sp.x + predict_gravity_x  +12;
        sp.y = sp.y + predict_gravity_y -60;
    }
    else if ( angle >= 320 && angle < 330 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -67;
    }
    else if ( angle >= 330 && angle < 340 )
    {
        sp.x = sp.x + predict_gravity_x +10;
        sp.y = sp.y + predict_gravity_y -70;
    }
    else if ( angle >= 340 && angle < 350 )
    {
        sp.x = sp.x + predict_gravity_x +20;
        sp.y = sp.y + predict_gravity_y -75;
    }
    else if ( angle >= 350 && angle < 360 )
    {
        sp.x = sp.x + predict_gravity_x +20;
        sp.y = sp.y + predict_gravity_y  - 75;
    }
    return sp;


}
cv::Point2f Energy::armor_predict_angle(cv::Point2f &RM_rect,cv::Point2f &center)
{
#ifdef new_code

    if(B_S==0)
    {if (Rotation_order == 0)
            curAngle = 180 / 8;                             /////此处要调   子弹拨动和过去的时间 会转过多少度
        else
            curAngle = -1 *180 / 8;
    }
    else if (B_S==1)
        /////////////////////////////////////////////大能量机关根据时间获得角度补偿////////////////////////////////////////
    {       double time_compensate=0.45;
    //////////////////////////////////正弦积分补偿////////////////////
        curAngle = -0.785/1.884*(cos(1.884*(Big_time+time_compensate))-cos(1.884*Big_time))+1.305*time_compensate;
        if (Rotation_order == 0)
            curAngle = curAngle*57.3;
        else
            curAngle = -curAngle*57.3;
    }
    Mat rot_mat=getRotationMatrix2D(RM_rect,curAngle,1);
    float sinA=rot_mat.at<double>(0,1);//sin(curAngle);
    float cosA=rot_mat.at<double>(0,0);//cos(curAngle);
    float xx=-(RM_rect.x-center.x);
    float yy=-(RM_rect.y-center.y);
    Point2f resPoint=Point2f(RM_rect.x+cosA*xx-sinA*yy,RM_rect.y+sinA*xx+cosA*yy);
    /////////////////////////////角度补偿
#ifdef show
    circle(binary,resPoint,1,Scalar(0,255,0),10);
#endif
    return resPoint;
#endif

}

//double Energy::Big_predict_angle()
//{  int T=2*pi/1.884*1000;
//    double time_compensate=0.35;
//    for (int i=0;i<T;i++)
//    {   double x;
//        x=double(i);
//        double j=x/1000;
//        pre[i]=-0.785/1.884*(cos(1.884*(j+time_compensate))-cos(1.884*j))+1.305*time_compensate;
//
//    }
//    return 0;
//}