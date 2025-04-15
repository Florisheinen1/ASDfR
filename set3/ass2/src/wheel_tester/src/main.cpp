#include "rclcpp/rclcpp.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

using std::placeholders::_1;

class WheelTester : public rclcpp::Node
{
public:
	WheelTester()
		: Node("wheel_tester"), toggle_(false)
	{

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

		publisher_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 10);

		state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/Command", 10);
		state_subscription_ = this->create_subscription<std_msgs::msg::Int32>("/XenoState", 10, xeno_state_callback);

		timer_ = this->create_wall_timer(
			std::chrono::seconds(5),
			std::bind(&WheelTester::publish_message, this));
	}

private:
	void publish_message()
	{
		auto msg = xrf2_msgs::msg::Ros2Xeno();

		if (toggle_)
		{
			msg.left_wheel_speed = 300.0;
			msg.right_wheel_speed = 0.0;
		}
		else
		{
			msg.left_wheel_speed = 0.0;
			msg.right_wheel_speed = 300.0;
		}
		toggle_ = !toggle_;

		RCLCPP_INFO(this->get_logger(), "Publishing: left=%.1f, right=%.1f", msg.left_wheel_speed, msg.right_wheel_speed);
		publisher_->publish(msg);
	}

	bool toggle_;
	rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr publisher_;

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscription_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WheelTester>());
	rclcpp::shutdown();
	return 0;
}
