#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cmath>
#include <algorithm>
#include <float.h>

// r: 60, g: 120, b: 90

class Masker : public rclcpp::Node
{
public:
	Masker() : Node("masker")
	{

		// Default value to target is green // TODO: For later, this can target specific colors
		this->declare_parameter("minimum_green_to_red", 1.3);
		this->declare_parameter("minimum_green_to_blue", 1.5);
		this->declare_parameter("minimum_brightness", 50);
		this->declare_parameter("use_target_color", true);

		this->declare_parameter("image_topic", "/image");

		this->minimum_green_to_red = this->get_parameter("minimum_green_to_red").as_double();
		this->minimum_green_to_blue = this->get_parameter("minimum_green_to_blue").as_double();
		this->minimum_brightness = this->get_parameter("minimum_brightness").as_int();

		this->resulting_image = std::make_shared<sensor_msgs::msg::Image>();

		auto topic_callback = [this](sensor_msgs::msg::Image::SharedPtr colored) -> void
		{
			auto masked = mask_image(colored);
			publisher_->publish(*masked);
			RCLCPP_INFO(this->get_logger(), "Masked image");
		};

		publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);

		std::string image_topic = this->get_parameter("image_topic").as_string();
		subscription_ = this->create_subscription<sensor_msgs::msg::Image>(image_topic, 10, topic_callback);
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

	double minimum_green_to_red;
	double minimum_green_to_blue;
	uint8_t minimum_brightness;

	/// @brief Will return a greyscale pixel value that represents if this pixel is green
	/// @param red The red value of current pixel
	/// @param green The green value of current pixel
	/// @param blue The blue value of current pixel
	/// @return How much this pixel is green
	uint8_t is_pixel_green(uint8_t red, uint8_t green, uint8_t blue) const
	{
		uint8_t brightness = this->get_luminance_of_pixel(red, green, blue);
		if (brightness < minimum_brightness)
		{
			return 0;
		}

		double current_red = static_cast<double>(red);
		double current_green = static_cast<double>(green);
		double current_blue = static_cast<double>(blue);

		double ratio_green_to_red = current_red <= 0.5 ? DBL_MAX : current_green / current_red;
		double ratio_green_to_blue = current_blue <= 0.5 ? DBL_MAX : current_green / current_blue;

		if (
			ratio_green_to_red > minimum_green_to_red &&
			ratio_green_to_blue > minimum_green_to_blue)
		{
			return 255;
		}
		else
		{
			// We return dark gray instead of black to indicate that the reason for not
			// being green is too strict minimum ratios, rather than too strict minimum brightness
			return 50;
		}
	}

	/// @brief Gets luminance of given pixel
	/// @param red Red value of pixel
	/// @param green Green value of pixel
	/// @param blue Blue value of pixel
	/// @return The luminance
	uint8_t get_luminance_of_pixel(uint8_t red, uint8_t green, uint8_t blue) const
	{
		return static_cast<uint8_t>(0.299 * red + 0.587 * green + 0.114 * blue);
	}

	/// @brief Masks the given image. Green pixels become white, all others become black
	/// @param rgb8_image The colored image to mask
	/// @return A green-masked image
	sensor_msgs::msg::Image::SharedPtr mask_image(const sensor_msgs::msg::Image::SharedPtr rgb8_image) const
	{
		auto resulting_image = std::make_shared<sensor_msgs::msg::Image>();

		resulting_image->header = rgb8_image->header;
		resulting_image->height = rgb8_image->height;
		resulting_image->width = rgb8_image->width;
		resulting_image->encoding = "mono8";
		resulting_image->is_bigendian = rgb8_image->is_bigendian;
		resulting_image->step = rgb8_image->width;

		int total_pixels = resulting_image->width * resulting_image->height;

		resulting_image->data.resize(total_pixels);

		for (int i = 0; i < total_pixels; i++)
		{
			int rgb8_index = i * 3;
			uint8_t red = rgb8_image->data[rgb8_index + 0];
			uint8_t green = rgb8_image->data[rgb8_index + 1];
			uint8_t blue = rgb8_image->data[rgb8_index + 2];

			uint8_t luminance = this->is_pixel_green(red, green, blue);
			resulting_image->data[i] = luminance;
		}

		return resulting_image;
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Masker>());
	rclcpp::shutdown();
	return 0;
}