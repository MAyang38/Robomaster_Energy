
#ifndef LOCALCOMMU_SUBSCRIBEASYNC_H
#define LOCALCOMMU_SUBSCRIBEASYNC_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

namespace cubot
{
    using namespace std;
    using namespace cv;

    typedef boost::function<void(const vector<cv::Point2f> &)> socketCB;
    class Subscrible
    {
    private:
        boost::asio::io_service         sub_io;
        boost::asio::ip::udp::socket    subsocket;
        boost::asio::ip::udp::endpoint  local_add;
        boost::system::error_code       ignored_error;

        vector<cv::Point2f>             connors;
        boost::thread                   read_thread;
        socketCB                        socketcallback_;
        void read();

    public:
        Subscrible(socketCB cb);
        ~Subscrible();

        int   send(vector<uint8_t> &send_buf);
        void  readJoin();
    };
}

#endif //LOCALCOMMU_SUBSCRIBEASYNC_H
