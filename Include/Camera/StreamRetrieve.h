#ifndef _STREAMRETRIEVE_H
#define _STREAMRETRIEVE_H
#include <fstream>
#include "GenICam/StreamSource.h"
#include "GenICam/System.h"
#include "Media/VideoRender.h"
#include "Media/ImageConvert.h"
#include <opencv2/opencv.hpp>
#include "boost/bind.hpp"
#include "boost/function.hpp"
#include "boost/thread.hpp"
#include "camParaConfig.h"

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace cv;

//下面两函数可能来自Dahua动态库
//但在Infra GenIcam命名空间均没找到
//boost::function
// boost::function是一个函数对象的“容器”，概念上像是C/C++中函数指针类型的泛化，是一种“智能函数指针”。
// 它以对象的形式封装了原始的函数指针或函数对象，能够容纳任意符合函数签名的可调用对象。
// 因此，它可以被用于回调机制，暂时保管函数或函数对象，在之后需要的时机在调用，使回调机制拥有更多的弹性。


typedef boost::function<void(cv::Mat &imgSrc,cv::Mat&imagePre)> ImgCallback;
typedef boost::function<void(cv::Mat& src, cv::Mat& dst)> preProcessFunction;

class StreamRetrieve
{
public:
	StreamRetrieve(ICameraPtr &cameraSptr,daHuaPara_str &daHuaPara, IStreamSourcePtr& streamSptr,
				   ImgCallback imgCB,preProcessFunction preCB);
	void join();
private:
    std::chrono::system_clock::time_point now;
    std::time_t now_c;
    std::ofstream fout;
	void  threadProc();

	cv::Mat          _mat;
	IStreamSourcePtr m_streamSptr;
	ImgCallback 	 imgCallback;
	preProcessFunction 	 preProcess;
	boost::thread  	 pro_thread;
	ICameraPtr 		 _cameraSptr;
	daHuaPara_str 	 _daHuaPara;
	int Height;
	int Width;

	void Proc();
};



#endif