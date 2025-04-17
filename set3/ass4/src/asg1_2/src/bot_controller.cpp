#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "example_interfaces/msg/float64.hpp"
#include <cmath>

class BotController : public rclcpp::Node
{
public:
	BotController() : Node("bot_controller")
	{
		subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
			"/trackpos", 10, std::bind(&BotController::point_callback, this, std::placeholders::_1));

		left_wheel_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
			"/input/left_motor/setpoint_vel", 10);
		right_wheel_pub_ = this->create_publisher<example_interfaces::msg::Float64>(
			"/input/right_motor/setpoint_vel", 10);

		RCLCPP_INFO(this->get_logger(), "bot_controller Node Started (No sensitivity scaling)");
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
	rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_pub_;
	rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_pub_;

	const double max_wheel_speed_ = 1.0; // Maximum wheel speed in rad/s

	void point_callback(const geometry_msgs::msg::Point::SharedPtr msg)
	{
		int rel_x = static_cast<int>(msg->x);
		int rel_y = static_cast<int>(msg->y);

		if (rel_x == 0 && rel_y == 0)
		{
			RCLCPP_WARN(this->get_logger(), "No object detected. Stopping.");
			publish_wheel_velocities(0.0, 0.0);
			return;
		}

		// Directly use y as forward speed and x as turn adjust
		double forward_speed = std::clamp(-static_cast<double>(rel_y), -max_wheel_speed_, max_wheel_speed_);
		double turn_adjust = std::clamp(-static_cast<double>(rel_x), -max_wheel_speed_, max_wheel_speed_);

		double left_speed = std::clamp(forward_speed + turn_adjust, -max_wheel_speed_, max_wheel_speed_);
		double right_speed = std::clamp(forward_speed - turn_adjust, -max_wheel_speed_, max_wheel_speed_);

		publish_wheel_velocities(left_speed, right_speed);

		RCLCPP_INFO(this->get_logger(),
					"rel_x: %+4d, rel_y: %+4d | L: %.2f, R: %.2f",
					rel_x, rel_y, left_speed, right_speed);
	}

	void publish_wheel_velocities(double left, double right)
	{
		example_interfaces::msg::Float64 left_msg;
		example_interfaces::msg::Float64 right_msg;
		left_msg.data = left;
		right_msg.data = right;

		left_wheel_pub_->publish(left_msg);
		right_wheel_pub_->publish(right_msg);
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BotController>());
	rclcpp::shutdown();
	return 0;
}
