#ifndef RADIO_SUBSCRIBER_HPP
#define RADIO_SUBSCRIBER_HPP

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <mutex>

class RadioSubscriber : public rclcpp::Node
{
public:
    RadioSubscriber(std::string node_name)
    : Node(node_name)
    {
        sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "radio_slider_values", 1, std::bind(&RadioSubscriber::callback, this, std::placeholders::_1));
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    Eigen::VectorXd fake_radio_;
    std::mutex mutex_;

    void callback(const std_msgs::msg::Float64MultiArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t size = msg->data.size();
        fake_radio_ = Eigen::VectorXd(size);
        for (size_t i = 0; i < size; ++i) {
            fake_radio_(i) = msg->data[i];
        }
    }


};

#endif // RADIO_SUBSCRIBER_HPP