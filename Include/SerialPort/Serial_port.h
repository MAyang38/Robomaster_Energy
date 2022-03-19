#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>

namespace cubot{
    typedef boost::function<void(const uint8_t *)> sp_callback_;

    class serial_port{
    public:

        serial_port(sp_callback_ cb, const std::string& sp_dev, const int &baud_rate);
        ~serial_port();

        int SerialPublish(const std::vector<cv::Point2f> &connors);
        void spJoin();

    private:
        boost::system::error_code  error_sp;
        boost::asio::io_service    io_sp;
        boost::asio::serial_port   sp;
        boost::thread              t0;
        sp_callback_               sp_callback;

        uint8_t                    recv_buf[1] = {0};
        void read();
        std::size_t write_sync(uint8_t *send_buf, int send_size);
    };
}


#endif //SERIAL_PORT_H
