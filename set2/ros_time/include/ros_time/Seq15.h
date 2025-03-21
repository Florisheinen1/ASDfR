#ifndef SEQ15_H
#define SEQ15_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <fstream>

using namespace std::chrono;
using std::placeholders::_1;

class Seq15 : public rclcpp::Node
{
public:
    Seq15();
    ~Seq15();

private:

    /// @brief is called every 1 ms to publish the message
    void timer_callback();

    /// @brief Called when message is recived from subscriber ie loop15
    /// @param msg is the data of timestap that was published earlier  
    void callback(const std_msgs::msg::String::SharedPtr msg);

    // ROS 2 interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Internal state variables
    long last_sent_;
    double prev_rtt_;
    std::ofstream log_file_;
};

#endif // SEQ15_H
