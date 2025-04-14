#include "rclcpp/rclcpp.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

class WheelTester : public rclcpp::Node
{
public:
	WheelTester()
	: Node("wheel_tester"), phase_(0)
	{
		publisher_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&WheelTester::publish_message, this)
		);
		start_time_ = now();
	}

private:
	void publish_message()
	{
		auto now_time = now();
		auto elapsed = now_time - start_time_;
		auto msg = xrf2_msgs::msg::Ros2Xeno();

		switch (phase_) {
			case 0: // Move straight for 2s
				if (elapsed < rclcpp::Duration(2s)) {
					msg.left_wheel_speed = -300.0;
					msg.right_wheel_speed = 300.0;
				} else {
					phase_ = 1;
					start_time_ = now_time;
					return;
				}
				break;
			case 1: // Rotate left for 1s
				if (elapsed < rclcpp::Duration(1s)) {
					msg.left_wheel_speed = 300.0;
					msg.right_wheel_speed = 300.0;
				} else {
					phase_ = 2;
					start_time_ = now_time;
					return;
				}
				break;
			case 2: // Rotate right for 1s
				if (elapsed < rclcpp::Duration(1s)) {
					msg.left_wheel_speed = -300.0;
					msg.right_wheel_speed = -300.0;
				} else {
					phase_ = 3;
					start_time_ = now_time;
					return;
				}
				break;
			case 3: // Halt
				msg.left_wheel_speed = 0.0;
				msg.right_wheel_speed = 0.0;
				// You could optionally stop the timer here if you want
				// timer_->cancel();
				break;
		}

		RCLCPP_INFO(
			this->get_logger(), 
			"Phase %d - Publishing: left=%.1f, right=%.1f", 
			phase_, msg.left_wheel_speed, msg.right_wheel_speed
		);

		publisher_->publish(msg);
	}

	int phase_;
	rclcpp::Time start_time_;
	rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WheelTester>());
	rclcpp::shutdown();
	return 0;
}
