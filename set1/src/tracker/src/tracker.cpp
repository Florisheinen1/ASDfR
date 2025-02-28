#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
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

			RCLCPP_INFO(this->get_logger(), "Object at x: %d, y: %d", x, y);
		};

		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/mask", 10, topic_callback);
	}

private:
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

		if (pixel_count == 0) {
			return std::make_pair<int, int>(0, 0);
		}

		// And calculate the average
		int center_x = sum_x / pixel_count;
		int center_y = sum_y / pixel_count;
		return std::make_pair(center_x, center_y);
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Tracker>());
	rclcpp::shutdown();
	return 0;
}