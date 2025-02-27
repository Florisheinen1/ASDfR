#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LightPrinter : public rclcpp::Node {
public:
	LightPrinter() : Node("lightprinter") {

	auto topic_callback = [this](std_msgs::msg::String::SharedPtr msg) -> void {
		RCLCPP_INFO(this->get_logger(), "The lights are: %s", msg->data.c_str());
	};

	subscription_ = this->create_subscription<std_msgs::msg::String>("/lights", 10, topic_callback);

}

private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LightPrinter>());
	rclcpp::shutdown();
	return 0;
}