#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <utility>

class ImgSqControl : public rclcpp::Node {
public:
    ImgSqControl() : Node("imgsqcontrol"), x_relbot_{0.0, 0.0} {
        // Subscribe to the moving camera output
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/moving_camera_output", 10, std::bind(&ImgSqControl::image_callback, this, std::placeholders::_1));

        // Publisher for controlling RELbot setpoint
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("/relbot/setpoint", 10);

        RCLCPP_INFO(this->get_logger(), "ImgSqControl Node Started (Using Moving Camera Output)");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr setpoint_pub_;
    
    double tau_ = 1.0; // Time constant τ
    std::pair<double, double> x_relbot_; // RELbot pose (x, θz)
    double dt_ = 0.1; // Time step for integration

    /// @brief Callback function for processing the moving camera image
    void image_callback(sensor_msgs::msg::Image::SharedPtr image) {
        auto object_center = get_brightest_region_center(image);
        double x_light = static_cast<double>(object_center.first);
        double theta_light = compute_theta(object_center.first, image->width);

        if (x_light == 0.0 && theta_light == 0.0) {
            RCLCPP_WARN(this->get_logger(), "No bright object detected. Stopping.");
            return;
        }

        // Compute error
        double x_error = x_light - x_relbot_.first;
        double theta_error = theta_light - x_relbot_.second;

        // First-order control using Forward Euler integration
        double x_set = x_relbot_.first + (dt_ / tau_) * x_error;
        double theta_set = x_relbot_.second + (dt_ / tau_) * theta_error;

        // Update RELbot state (assumption: RELbot follows setpoint well)
        x_relbot_ = {x_set, theta_set};

        // Publish new setpoint
        publish_setpoint(x_set, theta_set);
    }

    /// @brief Publish RELbot setpoint
    void publish_setpoint(double x_set, double theta_set) {
        auto pose_msg = geometry_msgs::msg::Pose2D();
        pose_msg.x = x_set;
        pose_msg.theta = theta_set;
        setpoint_pub_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Setpoint Updated: x=%.2f, θz=%.2f", x_set, theta_set);
    }

    /// @brief Finds the center of the brightest region in the image
    std::pair<int, int> get_brightest_region_center(sensor_msgs::msg::Image::SharedPtr image) const {
        int width = image->width;
        int height = image->height;
        int step = image->step;

        int sum_x = 0, sum_y = 0, pixel_count = 0;
        uint8_t brightness_threshold = 200;

        for (int row = 0; row < height; row++) {
            for (int column = 0; column < width; column++) {
                int index = column + row * step;
                if (image->data[index] > brightness_threshold) {
                    sum_x += column;
                    sum_y += row;
                    pixel_count++;
                }
            }
        }

        if (pixel_count == 0) return {0, 0};
        return {sum_x / pixel_count, sum_y / pixel_count};
    }

    /// @brief Compute θ_light based on the object's x-position in the image
    double compute_theta(int x, int image_width) const {
        double screen_center = image_width / 2.0;
        return (x - screen_center) * (M_PI / screen_center); // Scale to [-π, π]
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgSqControl>());
    rclcpp::shutdown();
    return 0;
}
