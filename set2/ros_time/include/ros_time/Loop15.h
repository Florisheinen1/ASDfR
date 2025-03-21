#ifndef LOOP15_H
#define LOOP15_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Loop15 : public rclcpp::Node
{
public:
    Loop15();

private:
    // Called when message is received from subscriber (seq15_topic)
    void callback(const std_msgs::msg::String::SharedPtr msg);

    // ROS 2 interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif // LOOP15_H
