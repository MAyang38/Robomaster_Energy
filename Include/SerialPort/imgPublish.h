#ifndef IMG_PUBLISH_H
#define IMG_PUBLISH_H

#include <boost/date_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/timer.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

typedef struct
{
    cv::Size  size;
    int       type;
    int       seq;
    boost::interprocess::managed_shared_memory::handle_t  handle;
    boost::posix_time::time_duration      timestamp;
} CubotImgPtr;

namespace cubot
{
    using namespace  boost::interprocess;
    using namespace  boost::posix_time;
    using namespace  std;
    using namespace  cv;

    class ImgPub
    {

    public:
        ImgPub( const char *shmname, const Mat &img);
        void pub(const Mat &captured_image);

        ~ImgPub(){
            boost::interprocess::named_mutex::remove(knock_name.c_str());
            boost::interprocess::shared_memory_object::remove(ImgPub::shm_name.c_str());
        };
    private:
        boost::interprocess::managed_shared_memory *segment;
        string image_name;
        string shm_name;
        string knock_name;
        string data_message;
        int* shm_image_message;
        char* shm_img;
        boost::interprocess::named_mutex  *img_mtx;
        ptime epoch;
    };

    ImgPub::ImgPub(const char *shmname, const cv::Mat &img)
    {
        ImgPub::image_name=string(shmname)+"_image";
        ImgPub::shm_name = shmname;
        ImgPub::knock_name = string(shmname) + "_lock";
        ImgPub::data_message = string(shmname) + "_message";
        boost::interprocess::named_mutex::remove(knock_name.c_str());
        boost::interprocess::shared_memory_object::remove(shmname);
        ImgPub::segment = new boost::interprocess::managed_shared_memory (boost::interprocess::create_only,ImgPub::shm_name.c_str(), img.total() * img.elemSize() + 1024);
        ImgPub::img_mtx = new boost::interprocess::named_mutex(boost::interprocess::create_only, ImgPub::knock_name.c_str());
        ImgPub::shm_img = ImgPub::segment->construct<char>(ImgPub::image_name.c_str())[img.total() * img.elemSize()](0);
        ImgPub::shm_image_message = ImgPub::segment->construct<int>(ImgPub::data_message.c_str())[3](0);
    }

    void ImgPub::pub(const cv::Mat &captured_image)
    {
        ImgPub::img_mtx->lock();

        memcpy(ImgPub::shm_img, captured_image.data, captured_image.total()*captured_image.elemSize());
        *ImgPub::shm_image_message = captured_image.cols;
        *(ImgPub::shm_image_message+1) = captured_image.rows;
        *(ImgPub::shm_image_message+2) = captured_image.type();

        ImgPub::img_mtx->unlock();
    }
}
#endif
