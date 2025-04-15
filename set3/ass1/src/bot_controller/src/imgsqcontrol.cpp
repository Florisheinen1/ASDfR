#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <utility>

class ImgSqControl : public rclcpp::Node {
public:
    ImgSqControl() : Node("imgsqcontrol") {
        // Subscribe to the greyscale image topic
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/greyscale", 10, std::bind(&ImgSqControl::image_callback, this, std::placeholders::_1));

        // Publisher for controlling robot motion
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/input/twist", 10);
        RCLCPP_INFO(this->get_logger(), "ImgSqControl Node Started (Using Greyscale Input)");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    /// @brief Callback function for processing the greyscale image
    void image_callback(sensor_msgs::msg::Image::SharedPtr greyscale) {
        auto object_center = get_brightest_region_center(greyscale);
        int x = object_center.first;
        int y = object_center.second;
        int image_width = greyscale->width;

        if (x == 0 && y == 0) {
            RCLCPP_WARN(this->get_logger(), "No bright object detected. Stopping.");
            publish_velocity(0.0, 0.0);
            return;
        }

        int screen_center = image_width / 2;
        double angular_z = 0.0;
        double linear_x = 0.0;

        if (x < screen_center - 50) {
            angular_z = 0.5;  // Turn left
            RCLCPP_INFO(this->get_logger(), "Turning LEFT");
        } else if (x > screen_center + 50) {
            angular_z = -0.5; // Turn right
            RCLCPP_INFO(this->get_logger(), "Turning RIGHT");
        } else {
            linear_x = 0.5;   // Move forward
            RCLCPP_INFO(this->get_logger(), "Moving FORWARD");
        }

        publish_velocity(linear_x, angular_z);
    }

    /// @brief Publish Twist command to move the robot
    void publish_velocity(double linear_x, double angular_z) {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_vel_pub_->publish(twist_msg);
    }

    /// @brief Finds the center of the brightest region in the greyscale image
    std::pair<int, int> get_brightest_region_center(sensor_msgs::msg::Image::SharedPtr greyscale) const {
        int width = greyscale->width;
        int height = greyscale->height;
        int step = greyscale->step;

        int sum_x = 0, sum_y = 0, pixel_count = 0;
        uint8_t brightness_threshold = 200; // Adjust this value for sensitivity

        for (int row = 0; row < height; row++) {
            for (int column = 0; column < width; column++) {
                int index = column + row * step;
                if (greyscale->data[index] > brightness_threshold) {  // Consider bright pixels only
                    sum_x += column;
                    sum_y += row;
                    pixel_count++;
                }
            }
        }

        if (pixel_count == 0) return {0, 0}; // No bright object detected
        return {sum_x / pixel_count, sum_y / pixel_count}; // Return center of bright region
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImgSqControl>());
    rclcpp::shutdown();
    return 0;
}
