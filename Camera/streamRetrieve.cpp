#include <stdlib.h>
#include "Camera/StreamRetrieve.h"
#include "Camera/camParaConfig.h"
#include "Camera/shm.hpp"
#include "ForDefine.h"

#ifdef ENERGY_TEST
    int receive[3] = {2, 0, 1};
#elif defined(AUTOHIT_TEST)
    int receive[3] = {1, 0, 1};
#else
    int receive[3] = {1, 0, 1};
#endif

//int Mode_last = 0;
//void mode_subCB(const std::vector<cv::Point2f> &data);
int last_mode = 0;


StreamRetrieve::StreamRetrieve(ICameraPtr &cameraSptr, daHuaPara_str &daHuaPara, IStreamSourcePtr &streamSptr,
                               ImgCallback imgCB, preProcessFunction preCB) :
        _cameraSptr(cameraSptr), _daHuaPara(daHuaPara), m_streamSptr(streamSptr), imgCallback(imgCB), preProcess(preCB),
        Width(daHuaPara.imgWidth), Height(daHuaPara.imgHeight) {
    pro_thread = boost::thread(boost::bind(&StreamRetrieve::Proc, this));
}

void StreamRetrieve::join() {
    pro_thread.join();
}

inline long getCurrentTime_() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}


void StreamRetrieve::Proc() {
    std::cout << "camera runing..." << std::endl;
#ifdef LOG_DATA
    now = std::chrono::system_clock::now();
    now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();
    fout.open("../../Robomaster2020/LOG/cam_node_1_Stream" + str_time + ".log");
    fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
    fout << "camera runing..." << std::endl << std::endl;
#endif
    int frame_error_cnt = 0;
    static shm_publisher_create<int> mode_pub("car_mode", 3);
    mode_pub.broadcast(receive);
    static shm_subscriber<int> mode_sub("car_mode");
    ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

#ifdef VIDEO_WITER
    //cv::Size videoSize(1280,1024);
    cv::VideoWriter writer_armor;
    int FrameNumofArmor = 5; //装甲识别模式，每15帧保存一帧图像
    int CntOfArmor = 0;
    int armor_cnt=0;
    std::string videodirname = "../../Robomaster2020/Videos/cam_node_1"+str_time+"/";
    const char * videodirnamestr = videodirname.c_str();
    mkdir(videodirnamestr,S_IRUSR | S_IWUSR | S_IXUSR);
    std::string video_armor_name = videodirname+"Realtime_Armor.avi";
    writer_armor.open(video_armor_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(1280,1024));
    armor_cnt=0;
    while(!writer_armor.isOpened()){
        armor_cnt++;
        writer_armor.open(video_armor_name,cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, cv::Size(1280,1024));
        if(armor_cnt>=10){
            break;
        }
    }
#endif
    float average_fps = 0;
    int nums = 1;
    while (1) {
        long whileStart = getCurrentTime_();
        mode_sub.get(receive);
        if (receive[0] != last_mode) {
            cout << "mode change" << endl;
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "mode change" << std::endl << std::endl;
#endif
            receive[0] = 2;
            if (receive[0] == 1) {
                ArmorModelParaConfig_runtime(_cameraSptr, _daHuaPara);
            } else if (receive[0] == 2) {
                RuneModelParaConfig_runtime(_cameraSptr, _daHuaPara);
            }
        }
        last_mode = receive[0];
        /********************************************/
        CFrame frame;//创建cFrame类对象，用getFrame函数从大华相机读取图片
        if (!m_streamSptr) {
            printf("m_streamPtr is NULL.\n");
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "m_streamPtr is NULL." << std::endl << std::endl;
#endif
            return;
        }
        bool isSuccess = m_streamSptr->getFrame(frame, 200);
        /// \~chinese
        /// \brief 获取一帧图像，该接口不支持多线程调用
        /// \param [out] frame 一帧图像
        /// \param [in]  timeoutMS 获取一帧图像的超时时长,单位MS,当值为INFINITE时表示无限等待
        /// \return 返回是否成功
        ///virtual bool getFrame(CFrame &frame,uint32_t timeoutMS = INFINITE) const = 0;
        if (!isSuccess) {
            frame_error_cnt++;
            if (frame_error_cnt > 50) {
                printf("\nFrame fail, maybe the USB connecttion fail\n");
#ifdef LOG_DATA
                now = std::chrono::system_clock::now();
                now_c = std::chrono::system_clock::to_time_t(now);
                fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                fout << "Frame fail, maybe the USB connecttion fail." << std::endl << std::endl;
#endif
                return;
            } else {
                printf("getFrame fail.\n");
#ifdef LOG_DATA
                now = std::chrono::system_clock::now();
                now_c = std::chrono::system_clock::to_time_t(now);
                fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                fout << "getFrame fail." << std::endl << std::endl;
#endif
                continue;
            }
        } else
            frame_error_cnt = 0;

        bool isValid = frame.valid();
        /// brief 是否有效
        /// \return 返回该帧是否有效
        /// \retval < 0 frame invalid
        /// \retval 0   frame valid
        ///bool valid() const;
        if (!isValid) {
            frame_error_cnt++;
            if (frame_error_cnt > 2000) {
                printf("\nframe invalid, and return...\n");
#ifdef LOG_DATA
                now = std::chrono::system_clock::now();
                now_c = std::chrono::system_clock::to_time_t(now);
                fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                fout << "frame invalid, and return..." << std::endl << std::endl;
#endif
                return;
            } else {
                printf("frame is invalid!\n");
#ifdef LOG_DATA
                now = std::chrono::system_clock::now();
                now_c = std::chrono::system_clock::to_time_t(now);
                fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
                fout << "frame is invalid" << std::endl << std::endl;
#endif
                continue;
            }
        } else
            frame_error_cnt = 0;

        //读取彩色图
        uint8_t *pRGBbuffer = NULL;//字面意思RGB缓冲区，临时储存区
        int nRgbBufferSize = 0;//初始化
        nRgbBufferSize = frame.getImageHeight() * frame.getImageWidth() * 3;//获取图像大小，三通道图图像，所以×3
        pRGBbuffer = (uint8_t *) malloc(nRgbBufferSize);//malloc,动态内存分配，向系统申请nRgbBufferSize字节空间

        if (pRGBbuffer == NULL) {
            printf("RGBbuffer malloc failed.\n");//临时内存分配失败
            free(pRGBbuffer);//释放空间
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "RGBbuffer malloc failed." << std::endl << std::endl;
#endif
            continue;
        }

        ///opt/DahuaTech/MVviewer/include/Media/ImageConvert.h
        IMGCNV_SOpenParam openParam;
        openParam.width = frame.getImageWidth();
        openParam.height = frame.getImageHeight();
        openParam.paddingX = frame.getImagePadddingX();
        openParam.paddingY = frame.getImagePadddingY();
        openParam.dataSize = frame.getImageSize();
        openParam.pixelForamt = gvspPixelBayRG8;

        unsigned char *pRgbFrameBuf = new(std::nothrow)  unsigned char[Width * Height * 1];

        //frame.getImage,给出图片数据内存首地址
        memcpy(pRgbFrameBuf, frame.getImage(), frame.getImageSize());


        //转换图片为BGR格式
        /**
*  ~chinese
*  @brief  转换为BGR24的转换函数
*  @param[in] pSrcData		：源数据
*  @param[in] pOpenParam	：IMGCNV_SOpenParam结构体,格式转换所需要的参数
*  @param[out]pDstData		：转换后的数据
*  @param[out]pDstDataSize	：转换后数据的长度
*  @Return:   IMGCNV_ERR_E  : 图像格式转换接口返回值
*  - IMGCNV_SUCCESS 表示执行成功
*  - 其他值见IMGCNV_ERR_E枚举
*  特别说明
*  像素格式为YUV411Packed的时，图像宽须能被4整除
*  像素格式为YUV422Packed的时，图像宽须能被2整除
*  像素格式为YUYVPacked的时，图像宽须能被2整除
*  转换后的图像:数据存储是从最上面第一行开始的，这个是相机数据的默认存储方向
         *  */

        //pRgbFrameBuf:摄像头原始数据
        IMGCNV_EErr status = IMGCNV_ConvertToBGR24(pRgbFrameBuf, &openParam, pRGBbuffer, &nRgbBufferSize);
        if (IMGCNV_SUCCESS != status) {
            printf("IMGCNV_ConvertToBGR24 failed.\n");
            delete pRgbFrameBuf;
#ifdef LOG_DATA
            now = std::chrono::system_clock::now();
            now_c = std::chrono::system_clock::to_time_t(now);
            fout << std::put_time(std::localtime(&now_c), "%F %T") << std::endl;
            fout << "IMGCNV_ConvertToBGR24 failed." << std::endl << std::endl;
#endif
            return;
        }
        delete pRgbFrameBuf;
        //得到BGR24格式的数据pRGBbuffer
        cv::Mat pre_out;
        cv::Mat _mat(frame.getImageHeight(), frame.getImageWidth(), CV_8UC3, pRGBbuffer, frame.getImageWidth() * 3);

#ifdef VIDEO_WITER
        CntOfArmor++;
        if(CntOfArmor%FrameNumofArmor == 0)
        {
            CntOfArmor = 0;
            writer_armor << _mat;
        }
#endif

        long t1 = getCurrentTime_();
        preProcess(_mat, pre_out);
        long t2 = getCurrentTime_();
        std::cout << "nums:" << nums << std::endl << "pre_imProcessing:" << (t2 - t1) / 1000.0 << std::endl;
        imgCallback(_mat, pre_out);
        free(pRGBbuffer);
        long whileEnd = getCurrentTime_();
        long Time = whileEnd - whileStart;
        float fps = 1000000.0 / Time;
        std::cout << "FPS:" << fps << std::endl;
        average_fps = ((nums - 1) * average_fps / nums) + (fps / nums);
        std::cout << "average_fps:" << average_fps << std::endl;
        nums++;
    }
}
