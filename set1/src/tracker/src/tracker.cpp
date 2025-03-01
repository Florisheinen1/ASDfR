#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>
#include <algorithm>
#include <float.h>
#include <utility>

class Tracker : public rclcpp::Node {
public:
	Tracker() : Node("tracker") {
		auto topic_callback = [this](sensor_msgs::msg::Image::SharedPtr mask) -> void {

			auto object_center = this->get_center_of_gravity(mask);
			
			auto x = object_center.first;
			auto y = object_center.second;

			auto msg = geometry_msgs::msg::Point();
			msg.x = x;
			msg.y = y;
			msg.z = 0.0; // Ignored
			publisher_->publish(msg);

			RCLCPP_INFO(this->get_logger(), "Object at x: %d, y: %d", x, y);
		};

		publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/trackpos", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/mask", 10, topic_callback);
	}

private:
	rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

	/// @brief Calculates the center of gravity from provided masked image
	/// @param mask The black and white image
	/// @return The center of gravity
	std::pair<int, int> get_center_of_gravity(sensor_msgs::msg::Image::SharedPtr mask) const {
		int width = mask->width;
		int height = mask->height;
		int step = mask->step;

		int sum_x = 0;
		int sum_y = 0;
		int pixel_count = 0;

		// Get coordinates of every white pixel, and sum them up
		for (int row = 0; row < height; row++) {
			for (int column = 0; column < width; column++) {
				int index = column + row * step;
				if (mask->data[index] == 255) {
					sum_x += column;
					sum_y += row;
					pixel_count++;
				}
			}
		}

		// If there are no pixels (or very few), assume we do not see our target
		if (pixel_count < 20) {
			return std::make_pair<int, int>(0, 0);
		}

		// And calculate the average
		int center_x = sum_x / pixel_count;
		int center_y = sum_y / pixel_count;
		
		// Get center of image
		int center_image_x = width / 2;
		int center_image_y = height / 2;

		// And calculate relative position compared to center of image
		int relative_center_x = center_x - center_image_x;
		int relative_center_y = center_y - center_image_y;
		return std::make_pair(relative_center_x, relative_center_y);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Tracker>());
	rclcpp::shutdown();
	return 0;
}