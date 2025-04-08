#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <utility>
#include <cmath>

class BotController : public rclcpp::Node {
public:
    BotController() : Node("bot_controller"), x_relbot_{0.0, 0.0} {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/trackpos", 10, std::bind(&BotController::point_callback, this, std::placeholders::_1));

        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/relbot/setpoint", 10);

        RCLCPP_INFO(this->get_logger(), "bot_controller Node Started (Subscribed to /trackpos)");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr setpoint_pub_;
    
    double tau_ = 1.0;
    double dt_ = 0.1;
    std::pair<double, double> x_relbot_;  // (x, θz)

    void point_callback(const geometry_msgs::msg::Point::SharedPtr point_msg) {
        int x_pos = static_cast<int>(point_msg->x);
        int y_pos = static_cast<int>(point_msg->y);

        if (x_pos == 0 && y_pos == 0) {
            RCLCPP_WARN(this->get_logger(), "No green object detected. Skipping frame.");
            return;
        }

        double x_light = static_cast<double>(x_pos);
        double theta_light = compute_theta_from_x(x_pos);

        double x_error = x_light - x_relbot_.first;
        double theta_error = theta_light - x_relbot_.second;

        double x_set = x_relbot_.first + (dt_ / tau_) * x_error;
        double theta_set = x_relbot_.second + (dt_ / tau_) * theta_error;

        x_relbot_ = {x_set, theta_set};
        publish_setpoint(x_set, theta_set);
    }

    void publish_setpoint(double x_set, double theta_set) {
        auto pose_msg = geometry_msgs::msg::Pose2D();
        pose_msg.x = x_set;
        pose_msg.theta = theta_set;
        setpoint_pub_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(),
            "[PUBLISH] Setpoint → x: %.2f, θz: %.2f", x_set, theta_set);
    }

    /// Assume screen width spans [-π, π] from left to right for θ mapping
    double compute_theta_from_x(int x_relative) const {
        const double image_width_half = 640.0 / 2.0; // adjust if needed
        return (x_relative / image_width_half) * M_PI;
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BotController>());
    rclcpp::shutdown();
    return 0;
}
