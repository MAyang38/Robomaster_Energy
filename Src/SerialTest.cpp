//
// Created by robostar on 2019/9/19.
//
#include "img_progress.h"
#include "SubscribeAsync.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void callback(const cv::Point2f &angle) {};

int main() {
    //cubot::serial_port sp(sp_callback, "/dev/ttyUSB0", 115200);
    boost::asio::io_service io_sp;
    boost::asio::serial_port sp1(io_sp, "/dev/ttyUSB0");
    sp1.set_option(boost::asio::serial_port::baud_rate(115200));
    sp1.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp1.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp1.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp1.set_option(boost::asio::serial_port::character_size(8));

    boost::system::error_code error_sp;
    int index=0;
    /*
    uint8_t read_buf[5] = {0};
    boost::asio::buffer(read_buf, 5);


    while (1) {
        sp1.read_some(boost::asio::buffer(read_buf, 5), error_sp);
        if(read_buf[0]==0xaa&&read_buf[4]==0xdd) {
            for (int i = 0; i < 5; i++) {
                if (i == 0)
                    std::cout << "//////////////////////////////读取结果start: "<<index<<"//////////////////////////" << std::endl;
                std::cout << (int) read_buf[i] << std::endl;
                index++;
                if (i == 4)
                    std::cout << "///////////////////////读取结果end/////////////////////////////////////////////////" << std::endl;
            }
        }
    }*/


    uint8_t recv_buf[1] = {0};
    boost::asio::buffer(recv_buf, 1);
    int RecFlag=-1;
    int mode[3]={0};
    sp1.read_some(boost::asio::buffer(recv_buf, 1), error_sp);
        for (int i = 0;; i++) {
            int SizeRec = sp1.read_some(boost::asio::buffer(recv_buf, 1), error_sp);
            if (SizeRec > 0) {
                if (RecFlag == -1 && recv_buf[0] == 0xaa) {
                    std::cout << "/////////////////////读取结果start: " << index << "///////////////////////////////"
                              << std::endl;
                    RecFlag = 0;
                } else if (RecFlag == 0) {
                    mode[0] = recv_buf[0];
                    RecFlag = 1;
                } else if (RecFlag == 1)
                {
                    mode[1] = recv_buf[0];
                    RecFlag = 2;

                } else if (RecFlag == 2)
                {
                    mode[2] = recv_buf[0];
                    RecFlag = 3;
                    //mode_pub.broadcast(mode);
                } else if (RecFlag == 3 && recv_buf[0] == 0xdd) {
                    std::cout << "///////////////////////读取结果end//////////////////////////////////////////////////"
                              << std::endl;
                    RecFlag = -1;
                }
                std::cout << (int) recv_buf[0] << std::endl;
//            usleep(1000);
            }
        }
            std::cout << (int) recv_buf[0] << std::endl;
            return 0;
        }

