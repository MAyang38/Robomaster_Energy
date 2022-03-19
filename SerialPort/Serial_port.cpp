#include "Serial_port.h"
#include "shm.hpp"

namespace cubot {

    serial_port::serial_port(sp_callback_ cb, const std::string &sp_dev, const int &baud_rate)
            : sp_callback(cb),
              sp(io_sp, sp_dev) {
        sp.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        sp.set_option(boost::asio::serial_port::character_size(8));

        t0 = boost::thread(boost::bind(&serial_port::read, this));

        io_sp.run();
    }

    serial_port::~serial_port()
    {
    }

    std::size_t serial_port::write_sync(uint8_t *send_buf, int send_size) {
        return sp.write_some(boost::asio::buffer(send_buf, send_size), error_sp);
    }

    void serial_port::spJoin() {
        t0.join();
    }

    void serial_port::read() {
        //std::cout<<"starting receiving data"<<std::endl;
        static shm_publisher_open<int> mode_pub("car_mode");
        static int RecFlag = -1;

        for(int index=1;;index++) {
            //std::cout<<"new while"<<std::endl;

            int mode[3];
            //std::cout<<(int)recv_buf[0]<<"!!!!!!!!!!!!!!!!!!!!!!1"<<std::endl;
            int recSize=sp.read_some(boost::asio::buffer(recv_buf, 1), error_sp);
            if(recSize>0)
            {
                if (RecFlag == -1 && recv_buf[0] == 0xaa) {
                    //::cout << "/////////////////////读取结果start: " << index << "///////////////////////////////"
                             // << std::endl;
                    RecFlag = 0;
                } else if (RecFlag == 0) {

                    mode[0] = recv_buf[0];
                    RecFlag = 1;
                } else if (RecFlag == 1) {
                    mode[1] = recv_buf[0];
                    RecFlag = 2;

                } else if (RecFlag == 2) {
                    mode[2] = recv_buf[0];
                    RecFlag = 3;
                    mode_pub.broadcast(mode);
                } else if (RecFlag == 3 && recv_buf[0] == 0xdd) {
                    //std::cout << "///////////////////////读取结果end//////////////////////////////////////////////////"
                              // std::endl;
                    RecFlag = -1;
                }
               // std::cout << (int) recv_buf[0] << std::endl;
//            usleep(1000);
            }
        }
    }


    int serial_port::SerialPublish(const std::vector<cv::Point2f> &connors) {
        //std::cout << connors[0] << std::endl;
        //if((connors[0].x==0)&&(connors[0].y==0))
        //return 0;

        //connors'size() is not more than 3
        int x = int(connors[0].x * 300);
        int y = int(connors[0].y * 300);
        int z = int(connors.back().y);
        //std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
       // std::cout<<connors[0]<<"!!!"<<std::endl;


        uint8_t x_h = x >> 8;
        uint8_t x_l = x;
        uint8_t y_h = y >> 8;
        uint8_t y_l = y;
        //uint8_t z_h = z >> 8;
        //uint8_t z_l = z;

        uint8_t send_buf[8] = {0};
        send_buf[0] = 0xaa;
        send_buf[1] = x_h;
        send_buf[2] = x_l;
        send_buf[3] = y_h;
        send_buf[4] = y_l;
        send_buf[5] = z;
        //send_buf[6] = z_l;
        send_buf[6] = 0xdd;
        /*****************************/
        /*
        uint16_t a = send_buf[2]|(send_buf[1]<<8);
        uint16_t b = send_buf[4]|(send_buf[3]<<8);
        std::cout<<"转换后16进制"<<a<<" "<<b<<std::endl;
         */

        int isSpSucss = write_sync(send_buf, 7);
        if (isSpSucss > 0)

        {}//std::cout << " SerialPort publish " << isSpSucss << " bytes sucessfully." << std::endl;
        else
        {}
            //std::cerr << "serial port connects failed...\n";
        return isSpSucss;
    }
}
