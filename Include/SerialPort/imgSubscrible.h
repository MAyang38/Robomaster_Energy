#ifndef IMG_SUB_H
#define IMG_SUB_H

#include <boost/date_time.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>


typedef struct
{
    cv::Size  size;
    int       type;
    int       seq;
    boost::interprocess::managed_shared_memory::handle_t  handle;
    boost::posix_time::time_duration      timestamp;
} CubotImgPtr;

typedef struct
{
    int       seq;
    cv::Mat   img;
    boost::posix_time::time_duration  timestamp;
} CubotImg;

namespace cubot
{

    using namespace cv;
    using namespace std;
    namespace ip  = boost::interprocess;
    typedef boost::function<void(const CubotImg&)> ImgCallback;

    class ImgSub
    {
        public:
             ImgSub(const char *shmname, ImgCallback imgCB);
             void  subJoin();
            ~ImgSub();

        private:
            int final_version;
            ip::managed_shared_memory *msm;
            ip::interprocess_mutex *inter_mtx;

            string shm_name;
            boost::thread rec_thread;
            ImgCallback imgcallback_;
            CubotImg    sensor_img;
            void recspin();

    };
}



#endif
