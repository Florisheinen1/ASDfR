#include "rclcpp/rclcpp.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include <chrono>

class WheelTester : public rclcpp::Node
{
public:
	WheelTester()
		: Node("wheel_tester"), phase_(0)
	{

		this->is_in_action = false;

		auto xeno_state_callback = [this](std_msgs::msg::Int32::SharedPtr current_state) -> void
		{
			if (current_state.data == 3)
			{
				// Done initializing. Publish RUN command!

				auto command = std_msgs::msg::Int32();
				command.data = 2;
				state_publisher_.publish(command);

				RCLCPP_INFO(this->get_logger(), "Published START command");
			}
		};

		auto action_callback = [this](std_msgs::msg::Empty::SharedPtr message) -> void
		{
			start_time = std::chrono::steady_clock::now();
			is_in_action = true;
		};

		state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/Command", 10);
		state_subscription_ = this->create_subscription<std_msgs::msg::Int32>("/XenoState", 10, xeno_state_callback);

		action_subscription_ = this->create_subscription<std_msgs::msg::Empty>("/line_and_turn", 10, action_callback);

		publisher_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);
		timer_ = this->create_wall_timer(
			std::chrono::milliseconds(100),
			std::bind(&WheelTester::publish_message, this));
	}

private:
	void publish_message()
	{
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<int> time_since_start = now - this->start_time;
		auto elapsed_seconds = time_since_start.count();

		auto msg = xrf2_msgs::msg::Ros2Xeno();
		msg.left_wheel_speed = 0;
		msg.right_wheel_speed = 0;

		if (this->is_in_action)
		{
			if (elapsed_seconds < 5)
			{
				msg.left_wheel_speed = 40;
				msg.right_wheel_speed = 40;
			}
			else if (elapsed_seconds < 10)
			{
				msg.left_wheel_speed = 40;
				msg.right_wheel_speed = -40;
			}
			else
			{
				this->is_in_action = false;
			}
		}

		RCLCPP_INFO(
			this->get_logger(),
			"Left=%.1f, Right=%.1f",
			msg.left_wheel_speed, msg.right_wheel_speed);

		publisher_->publish(msg);
	}

	rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_;

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscription_;

	rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr action_subscription_;

	rclcpp::TimerBase::SharedPtr timer_;

	std::chrono::steady_clock::time_point start_time;
	bool is_in_action;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WheelTester>());
	rclcpp::shutdown();
	return 0;
}
