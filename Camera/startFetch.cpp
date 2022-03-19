#include "Camera/startFetch.hpp"
#include "ForDefine.h"

using namespace std;

//尝试连接相机,(相当于主函数，抓图，调用回调函数并上传图片)
bool DahuaCamera::connectCamera(std::string &filename,
                                preProcessFunction preProcess)
                                {
#ifdef LOG_DATA
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    fout.open("../../Robomaster2020/LOG/cam_node_1_Fetch" + str_time + ".log");
#endif
    //这一部分为读取配置参数
    getCamParaFromYml(filename, DahuaCamera::daHuaPara);
    if (daHuaPara.IsReadYmlSucc != true) {
        std::cerr << "Failed to get the configur file " << std::endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        //char name[256]={0};
        //std::stringstream ss;
        //ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
        //std::string str_time = ss.str();
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "Failed to get the configur file." << std::endl << std::endl;
#endif
        return false;
    }

    /* 发现设备 */
    CSystem &systemObj = CSystem::getInstance();//获取实例
    TVector<ICameraPtr> vCameraPtrList;
    bool isDiscoverySuccess = systemObj.discovery(vCameraPtrList);
    if (!isDiscoverySuccess) {
        std::cout << "[error-001]: find device fail." << endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "[error-001]: find device fail." << std::endl << std::endl;
#endif
        return false;
    }

    if (vCameraPtrList.size() == 0)
    {
        std::cout << "[error-002]: no devices." << endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "[error-002]: find device fail." << std::endl << std::endl;
#endif
        return false;
    }
    /* 连接相机 */
    if (!vCameraPtrList[0]->connect()) {
        std::cout << "[error-003]: connect camera failed." << endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "[error-003]: find device fail." << std::endl << std::endl;
#endif
        return false;
    } else {
        std::cout << "connect camera success! " << endl;
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "connect camera success!" << std::endl << std::endl;
#endif
    }

    //获得相机指针
    DahuaCamera::cameraSptr = vCameraPtrList[0];

    /* 按照配置文件进行设置相机参数 */
    int IsSetCamParaSucc = 0;
    camParaConfig(DahuaCamera::cameraSptr, DahuaCamera::daHuaPara, IsSetCamParaSucc);
    if (IsSetCamParaSucc != 0) {
        std::cerr << "[error-004]: failed to set camera para" << endl;
        DahuaCamera::cameraSptr->disConnect();
#ifdef LOG_DATA

        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "[error-004]: failed to set camera para" << std::endl << std::endl;
#endif
        return false;
    } else {
        std::cout << "succeed to set camera para" << endl;
#ifdef LOG_DATA

        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "succeed to set camera para" << std::endl << std::endl;
#endif
    }

    /* 创建流对象 */
    IStreamSourcePtr streamPtr = systemObj.createStreamSource(DahuaCamera::cameraSptr);

    if (NULL == streamPtr) {
        std::cerr << "create stream obj  fail." << endl;
        DahuaCamera::cameraSptr->disConnect();
#ifdef LOG_DATA

        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "create stream obj  fail." << std::endl << std::endl;
#endif
        return false;
    }
    /* 停止抓图 并休眠1ms */
    streamPtr->stopGrabbing();
    usleep(1000);

    /* 开始取图 */
    bool isStartGrabbingSuccess = streamPtr->startGrabbing();//
    /// \~chinese
    /// \brief 开始抓图
    /// \param [in] maxImagesGrabbed 允许最多的抓图数，达到指定抓图数后停止抓图，如果为0，表示忽略此参数连续抓图
    /// \param [in] strategy 抓图策略,默认按缓存队列中的顺序抓图
    /// \return 返回抓图是否成功
    /// \~english
    /// \brief Start grabbing
    /// \param [in] maxImagesGrabbed Maximum images allowed to grab, once it reaches the limit then stop grabbing; If it is 0, then ignore this parameter and start grabbing continuously
    /// \param [in] strategy Image grabbing strategy; Image grabbing according to the order in buffer queue is default
    /// \return success:true, fail:false
    ///virtual bool startGrabbing(uint64_t maxImagesGrabbed = 0,EGrabStrategy strategy = grabStrartegySequential) = 0;

    if (!isStartGrabbingSuccess) {
        std::cerr << "Start Grabbing fail." << endl;
        DahuaCamera::cameraSptr->disConnect();
#ifdef LOG_DATA

        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "Start Grabbing fail." << std::endl << std::endl;
#endif
        return false;
    }

    /*创建取流线程*/
    //为什么回调函数没有参数？
    //

    //问题找到callback回调函数中的img参数;
    //推测在此函数中，将preprocess输出的函数传递给了callback函数，再传递给上传器
    Dahua::Memory::TSharedPtr<StreamRetrieve> streamThreadSptr(new StreamRetrieve(cameraSptr,
                                                                                  daHuaPara,
                                                                                  streamPtr,
                                                                                  boost::bind(DahuaCamera::callback, _1,
                                                                                              _2),
                                                                                  preProcess));
    //prePreocess函数于streamRetrieve中将cv::Mat _mat
    //转换为cv::Mat pre_out
    if (NULL == streamThreadSptr) {
        printf("create thread obj failed.\n");
        streamPtr->stopGrabbing();
        DahuaCamera::cameraSptr->disConnect();
#ifdef LOG_DATA
        now = std::chrono::system_clock::now();
        now_c = std::chrono::system_clock::to_time_t(now);
        fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
        fout << "create thread obj failed." << std::endl << std::endl;
#endif
        return false;
    }
    streamThreadSptr->join();
    return true;
}

//不用修改
void DahuaCamera::callback(cv::Mat &Src, cv::Mat &Pre) {
    if (Src.empty() || Pre.empty()) {
        return;
    }
//    cout<<"get image."<<endl;
    static publisher pub1("camera_Src_1", Src);
    static publisher pub2("camera_Pre_1", Pre);

    pub1.braodcast(Src);
    pub2.braodcast(Pre);
}
//默认的预处理函数,勿动


void DahuaCamera::nothing(cv::Mat &in, cv::Mat &out) {
#ifdef SRC_MODE
    cv::resize(in,out,cv::Size(),0.5,0.5);
    cv::cvtColor(out, out, cv::COLOR_BGR2HSV);
#else
#ifdef CUDA
    cv::cuda::GpuMat in_(1280,1024,CV_8UC3),
            out_(640,512,CV_8UC3);
    in_.upload(in);
    cv::cuda::resize(in_,in_,cv::Size(),0.5,0.5);
    cv::cuda::cvtColor(in_,out_,cv::COLOR_BGR2HSV);
    in_.download(in);
    out_.download(out);
#else
    cv::resize(in, in, cv::Size(), 0.5, 0.5);
    cv::cvtColor(in, out, cv::COLOR_BGR2HSV);
#endif
#endif
    cv::inRange(out, cv::Scalar(0, 0, 50), cv::Scalar(180, 255, 255), out);
}