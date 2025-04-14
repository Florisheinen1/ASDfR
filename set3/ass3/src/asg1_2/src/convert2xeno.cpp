#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "xrf2_msgs/msg/ros2_xeno.hpp"

class Convert2Xeno : public rclcpp::Node {
public:
    Convert2Xeno() : Node("convert2xeno") {
        left_sub_ = this->create_subscription<example_interfaces::msg::Float64>(
            "/input/left_motor/setpoint_vel", 10,
            [this](const example_interfaces::msg::Float64::SharedPtr msg) {
                left_vel_ = msg->data;
                publish_combined();
            });

        right_sub_ = this->create_subscription<example_interfaces::msg::Float64>(
            "/input/right_motor/setpoint_vel", 10,
            [this](const example_interfaces::msg::Float64::SharedPtr msg) {
                right_vel_ = msg->data;
                publish_combined();
            });

        xeno_pub_ = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("ros2xeno", 10);

        RCLCPP_INFO(this->get_logger(), "convert2xeno node started. Listening and publishing to ros2xeno.");
    }

private:
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr left_sub_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr right_sub_;
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr xeno_pub_;

    double left_vel_ = 0.0;
    double right_vel_ = 0.0;

    void publish_combined() {
        xrf2_msgs::msg::Ros2Xeno msg;
        msg.left_wheel_speed = left_vel_;
        msg.right_wheel_speed = right_vel_;
        xeno_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published Ros2Xeno -> L: %.2f, R: %.2f", left_vel_, right_vel_);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Convert2Xeno>());
    rclcpp::shutdown();
    return 0;
}
