//
// Created by snow on 18-4-12.
//
#include "PublishAsync.h"

namespace cubot {

    Publish_udp::Publish_udp(pub_callback_ callback)//上传器
            : pub_callback(callback),
              pub_socket(pub_io),
              localEp(boost::asio::ip::address::from_string("127.0.0.1"), 4321) {
        //////////
//        if(pub_socket.is_open())
//        {
//            pub_socket.shutdown(boost::asio::ip::udp::socket::shutdown_receive);
//            pub_socket.close();
//        }

        pub_socket.open(localEp.protocol());
        this->run();

        Mode.resize(3);
        pub_thread = boost::thread(boost::bind(&Publish_udp::proc, this));
    }


    Publish_udp::~Publish_udp() {
        ;
    }

    void Publish_udp::proc() {
        while (1) {
            //cout<<123<<endl;
            int recv_size = pub_socket.receive_from(boost::asio::buffer(Mode, 3), localEp);
            //cout<<456<<endl;

            if (recv_size > 0)
                pub_callback(Mode);
            usleep(2000);
        }
    }

    void Publish_udp::run() {
        pub_io.run();
    }

    std::size_t Publish_udp::pub(vector<cv::Point2f> &points) {
        int size0 = points.size() * sizeof(Point2f);
        size_t isSendSucss = pub_socket.send_to(boost::asio::buffer(points, size0), localEp);
        return isSendSucss;
    }

    void Publish_udp::pubJoin() {
        pub_thread.join();
    }
}
