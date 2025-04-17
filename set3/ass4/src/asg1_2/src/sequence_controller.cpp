#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tuple>

class SetpointNode : public rclcpp::Node
{
public:
	SetpointNode() : Node("sequence_controller"), step_index_(0)
	{
		cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
			"/input/twist", 10);

		this->declare_parameter("use_twist_cmd", true);
		use_twist_cmd_ = this->get_parameter("use_twist_cmd").as_bool();

		if (!use_twist_cmd_)
		{
			RCLCPP_ERROR(this->get_logger(), "ERROR: use_twist_cmd must be true for this node!");
			rclcpp::shutdown();
		}

		// setting timer
		timer_ = this->create_wall_timer(
			std::chrono::seconds(3), std::bind(&SetpointNode::publish_setpoints, this));
		// Setpoint sequence
		setpoint_sequence_ = {
			{0.5, 0.0, 5},	// forward
			{0.0, 0.0, 3},	// Stop
			{-0.5, 0.0, 5}, // Backward
			{0.0, 0.0, 3},	// Stop
		};

		duration_counter_ = std::get<2>(setpoint_sequence_[0]);

		RCLCPP_INFO(this->get_logger(), "Setpoint sequence is publishing");
	}

private:
	void publish_setpoints()
	{
		if (step_index_ >= setpoint_sequence_.size())
		{
			RCLCPP_INFO(this->get_logger(), "Setpoint sequence complete.");
			return;
		}

		auto msg = geometry_msgs::msg::Twist();
		msg.linear.x = std::get<0>(setpoint_sequence_[step_index_]);
		msg.angular.z = std::get<1>(setpoint_sequence_[step_index_]);

		cmd_vel_pub_->publish(msg);

		RCLCPP_INFO(this->get_logger(), "Published setpoint: Linear=%.2f, Angular=%.2f (Step %zu)",
					msg.linear.x, msg.angular.z, step_index_);

		if (--duration_counter_ <= 0)
		{
			step_index_++;
			if (step_index_ < setpoint_sequence_.size())
			{
				duration_counter_ = std::get<2>(setpoint_sequence_[step_index_]);
			}
		}
	}

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::vector<std::tuple<double, double, int>> setpoint_sequence_;
	size_t step_index_;
	int duration_counter_;
	bool use_twist_cmd_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SetpointNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
