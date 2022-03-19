#ifndef LOCALCOMMU_PUBLISHASYNC_H
#define LOCALCOMMU_PUBLISHASYNC_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

namespace cubot {
    using namespace std;
    using namespace cv;

    typedef boost::function<void(const vector<uint8_t> &)> pub_callback_;

    class Publish_udp {
    private:
        boost::asio::io_service         pub_io;
        boost::asio::ip::udp::endpoint  localEp;
        boost::asio::ip::udp::socket    pub_socket;
        boost::system::error_code       ignored_error;
        pub_callback_                   pub_callback;
        boost::thread                   pub_thread;
        vector<uint8_t>                 Mode;

        void proc();
        void run();

    public:
        Publish_udp(pub_callback_ callback);
        ~Publish_udp();

        std::size_t pub(vector<cv::Point2f> &points);
        void pubJoin();
    };
}
#endif //LOCALCOMMU_PUBLISHASYNC_H
