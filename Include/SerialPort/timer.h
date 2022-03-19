#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

namespace cubot
{
    using namespace std;
    typedef boost::function<void()> TimeCallback;

    class creatTimerCallback {
    public:
        creatTimerCallback(TimeCallback timecallback, int mill) :
                millisecond(mill),
                epoch(boost::gregorian::date(1970, boost::gregorian::Jan, 1)),
                timer_(io, boost::posix_time::millisec(millisecond)),
                timecallback_(timecallback)
        {
            timer_.async_wait(boost::bind(&creatTimerCallback::PeriodicCallback, this));
            io_thread = boost::thread(boost::bind(&creatTimerCallback::service_run, this));
        }

        void PeriodicCallback();


        ~creatTimerCallback()
        {
            io_thread.join();
        }
        void timerJoin();

    private:
        void service_run() {
            io.run();
        }

        int millisecond;
        boost::thread io_thread;
        const boost::posix_time::ptime epoch;
        boost::asio::io_service io;
        boost::asio::deadline_timer timer_;
        TimeCallback timecallback_;

    };

}
#endif
