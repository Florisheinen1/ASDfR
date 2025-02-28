#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cmath>
#include <algorithm>
#include <float.h>

// r: 60, g: 120, b: 90

class GreyScaler : public rclcpp::Node {
public:
	GreyScaler() : Node("greyscaler") {

		// Default value to target is green // TODO: For later, this can target specific colors
		auto topic_callback = [this](sensor_msgs::msg::Image::SharedPtr colored) -> void {
			auto greyscale = color_to_greyscale(colored);
			publisher_->publish(*greyscale);
		};

		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/greyscale", 10);
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 10, topic_callback);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

	/// @brief Gets luminance of given pixel
	/// @param red Red value of pixel
	/// @param green Green value of pixel
	/// @param blue Blue value of pixel
	/// @return The luminance
	uint8_t get_luminance_of_pixel(uint8_t red, uint8_t green, uint8_t blue) const {
		return static_cast<uint8_t>(0.299 * red + 0.587 * green + 0.114 * blue);
	}

	/// @brief Creates greyscale image of given image
	/// @param rgb8_image The colored image
	/// @return The mono8 greyscale of this image
	sensor_msgs::msg::Image::SharedPtr color_to_greyscale(const sensor_msgs::msg::Image::SharedPtr rgb8_image) const {
		auto mono = std::make_shared<sensor_msgs::msg::Image>();
		
		mono->header = rgb8_image->header;
		mono->height = rgb8_image->height;
		mono->width = rgb8_image->width;
		mono->encoding = "mono8";
		mono->is_bigendian = rgb8_image->is_bigendian;
		mono->step = rgb8_image->width;

		int total_pixels = mono->width * mono->height;

		mono->data.resize(total_pixels);

		for (int i = 0; i < total_pixels; i++) {
			int rgb8_index = i * 3;
			uint8_t red = rgb8_image->data[rgb8_index + 0];
			uint8_t green = rgb8_image->data[rgb8_index + 1];
			uint8_t blue = rgb8_image->data[rgb8_index + 2];

			int luminance = this->get_luminance_of_pixel(red, green, blue);
			mono->data[i] = luminance;
		}

		return mono;
	}
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GreyScaler>());
	rclcpp::shutdown();
	return 0;
}