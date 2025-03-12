#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono;
using std::placeholders::_1;

class Loop15 : public rclcpp::Node {
public:
    Loop15() : Node("loop15") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "seq15_topic", 10, std::bind(&Loop15::message_callback, this, _1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("loop15_topic", 10);

        RCLCPP_INFO(this->get_logger(), "Loop15 node started.");
    }

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());

        auto response_msg = std_msgs::msg::String();
        response_msg.data = msg->data; // Echo back the received message

        publisher_->publish(response_msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Loop15>());
    rclcpp::shutdown();
    return 0;
}
