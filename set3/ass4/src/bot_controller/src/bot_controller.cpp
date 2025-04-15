#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/int32.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#include <utility>
#include <cmath>
#include <algorithm>

// Around 3 cm/s
#define SPEED 0.32

class BotController : public rclcpp::Node
{
public:
	BotController() : Node("bot_controller")
	{

		auto xeno_state_callback = [this](std_msgs::msg::Int32::SharedPtr current_state) -> void
		{
			if (current_state->data == 3)
			{
				// Done initializing. Publish RUN command!
				auto command = std_msgs::msg::Int32();
				command.data = 2;
				state_publisher_->publish(command);

				RCLCPP_INFO(this->get_logger(), "Published START command");
			}
		};

		state_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/Command", 10);
		state_subscription_ = this->create_subscription<std_msgs::msg::Int32>("/Xenomai_state", 10, xeno_state_callback);

		subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
			"/trackpos", 10, std::bind(&BotController::point_callback, this, std::placeholders::_1));

		setpoint_pub_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("/Ros2Xeno", 10);

		RCLCPP_INFO(this->get_logger(), "bot_controller Node Started (Subscribed to /trackpos)");
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
	rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr setpoint_pub_; // TODO: Change this to Ros2Xeno

	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_publisher_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_subscription_;


	void point_callback(const geometry_msgs::msg::Point::SharedPtr point_msg)
	{
		int x_pos = static_cast<int>(point_msg->x);
		int y_pos = static_cast<int>(point_msg->y);

		if (x_pos == 0 && y_pos == 0)
		{
			RCLCPP_WARN(this->get_logger(), "No green object detected. Skipping frame.");
			return;
		}

		int corrected_x = std::clamp(x_pos, -150, 150);
		int corrected_y = std::clamp(y_pos, -100, 100);

		double ratio_x = (double)corrected_x / 460.0; // Arrives at around 3 cm per second, but then in radians per second
		double ratio_y = (double)corrected_y / 460.0;

		publish_setpoint(ratio_x + 0.1, -ratio_x + 0.1);
	}

	void publish_setpoint(double left_wheel_speed, double right_wheel_speed)
	{
		auto wheel_speeds = xrf2_msgs::msg::Ros2Xeno();
		wheel_speeds.left_wheel_speed = left_wheel_speed;
		wheel_speeds.right_wheel_speed = right_wheel_speed;
		setpoint_pub_->publish(wheel_speeds);

		RCLCPP_INFO(this->get_logger(), "Left: %.2f, Right: %.2f", left_wheel_speed, right_wheel_speed);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BotController>());
	rclcpp::shutdown();
	return 0;
}
