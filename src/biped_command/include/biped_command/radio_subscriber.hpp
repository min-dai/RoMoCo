#ifndef RADIO_SUBSCRIBER_HPP
#define RADIO_SUBSCRIBER_HPP

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <mutex>

class RadioSubscriber {
public:
    RadioSubscriber(ros::NodeHandle& nh, const std::string& topic)
    {
        sub_ = nh.subscribe(topic, 1, &RadioSubscriber::callback, this);
    }

    Eigen::VectorXd fake_radio() {
        std::lock_guard<std::mutex> lock(mutex_);
        return fake_radio_;
    }

    int mode_command()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (fake_radio_.size() == 0)
        {
            return -1000; // no data yet
        }
        return static_cast<int>(fake_radio_(0));
    }

private:
    ros::Subscriber sub_;
    Eigen::VectorXd fake_radio_;
    std::mutex mutex_;

    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t size = msg->data.size();
        fake_radio_ = Eigen::VectorXd(size);
        for (size_t i = 0; i < size; ++i) {
            fake_radio_(i) = msg->data[i];
        }
    }


};

#endif // RADIO_SUBSCRIBER_HPP