#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

class LightDetector : public rclcpp::Node {
public:
	LightDetector() : Node("lightdetector") {

	auto topic_callback = [this](sensor_msgs::msg::Image::SharedPtr image) -> void {
		RCLCPP_INFO(this->get_logger(), "Received an image with width: '%d' and height: '%d'", image->width, image->height);
		int brightness = this->calculate_brightness(image);
		RCLCPP_INFO(this->get_logger(), "Average brightness: %d", brightness);

	};
	subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, topic_callback);

}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

	int calculate_brightness(sensor_msgs::msg::Image::SharedPtr image) const {
		int width = image->width;
		int height = image->height;
		int total_pixels = width * height;

		int brightness_sum = 0;

		for (int i = 0; i < total_pixels; i++) {
			int data_index = i * 3; // R, G, B per pixel, each their own data integer
			int red = image->data[data_index];
			int green = image->data[data_index+1];
			int blue = image->data[data_index+2];

			int average_pixel_brightness = (red + green + blue) / 3;
			brightness_sum += average_pixel_brightness;
		}

		int average_brightness = brightness_sum / total_pixels;
		return average_brightness;
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LightDetector>());
	rclcpp::shutdown();
	return 0;
}