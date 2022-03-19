#include "SubscribeAsync.h"

namespace cubot {
    Subscrible::Subscrible(socketCB cb)
            : subsocket(sub_io),
              socketcallback_(cb),
              local_add(boost::asio::ip::address::from_string("127.0.0.1"), 4321) {
        //new-628
        //
        if (subsocket.is_open()) {
            subsocket.shutdown(boost::asio::ip::udp::socket::shutdown_receive);
            subsocket.close();
        }
        subsocket.open(local_add.protocol());
        subsocket.bind(local_add);

        read_thread = boost::thread(boost::bind(&Subscrible::read, this));
        sub_io.run();
    }

    Subscrible::~Subscrible() {
        ;
    }

    void Subscrible::read() {
        while (1) {
            connors.resize(3);

            int size0 = connors.size() * sizeof(Point2f);
            int recvFlag = subsocket.receive_from(boost::asio::buffer(connors, size0), local_add);
            //cout<<"recvFlag is "<<recvFlag<<endl;
            if (recvFlag > 0)
                socketcallback_(connors);
            //usleep(5000);
            connors.clear();
        }
    }

    int Subscrible::send(vector<uint8_t> &send_buf) {
        size_t isSendSucss = subsocket.send_to(boost::asio::buffer(send_buf, 1), local_add);
        return isSendSucss;
    }

    void Subscrible::readJoin() {
        read_thread.join();
    }


}

