#pragma once
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>

template<class msg_T>
class Subscriber
{
public:
    Subscriber(
        ros::NodeHandle &node_handle,
        const std::string &topic,
        const uint32_t queue_size
    ) {
        this->_mtx.reset(new std::mutex);
        this->_sub.reset(new ros::Subscriber(node_handle.subscribe<msg_T>(topic, queue_size, &Subscriber::_callback, this)));
    }

    virtual boost::shared_ptr<const msg_T> get_msg() {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        return this->_msg;
    }

protected:
    boost::shared_ptr<ros::Subscriber> _sub;
    boost::shared_ptr<std::mutex> _mtx;

    boost::shared_ptr<const msg_T> _msg;

    virtual void _callback(const boost::shared_ptr<const msg_T> &msg) {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        this->_msg = msg;
    }
};

template<class M0, class M1>
class TimeSyncSubscriber
{
public:
    TimeSyncSubscriber(
        ros::NodeHandle &node_handle,
        const std::string &topic0,
        const std::string &topic1,
        const uint32_t queue_size
    ) {
        this->_mtx.reset(new std::mutex);
        this->_sub0.reset(new message_filters::Subscriber<M0>(node_handle, topic0, 1));
        this->_sub1.reset(new message_filters::Subscriber<M1>(node_handle, topic1, 1));
        this->_sync.reset(new message_filters::TimeSynchronizer<M0, M1>(*this->_sub0, *this->_sub1, queue_size));
        this->_sync->registerCallback(boost::bind(&TimeSyncSubscriber::_callback, this, _1, _2));
    };

    void get_msgs(boost::shared_ptr<const M0> &msg0, boost::shared_ptr<const M0> &msg1)
    {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        msg0 = this->_msg0;
        msg1 = this->_msg1;
    };

protected:
    boost::shared_ptr<std::mutex> _mtx;
    boost::shared_ptr<message_filters::Subscriber<M0> > _sub0;
    boost::shared_ptr<message_filters::Subscriber<M1> > _sub1;
    boost::shared_ptr<message_filters::TimeSynchronizer<M0, M1> > _sync;

    boost::shared_ptr<const M0> _msg0;
    boost::shared_ptr<const M1> _msg1;

    void _callback(
        const boost::shared_ptr<const M0> &msg0,
        const boost::shared_ptr<const M1> &msg1
    ) {
        std::lock_guard<std::mutex> lock(*this->_mtx);
        this->_msg0 = msg0;
        this->_msg1 = msg1;
    }
};
