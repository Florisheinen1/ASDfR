Package tracker
-----------------------------------------------
Description: This package calculates the center of
gravity of white pixels in the received masked image,
and therefore tracking the position of the green ball.

----------------------------------------------
1. Node tracker
Calculates the center of gravity of received white pixels

Inputs:
- /mask
	Type: sensor_msgs::msg::Image
	A green-masked black and white image

Outputs:
- /trackpos
	Type: geometry_msgs::msg::Point
	The center of gravity of white pixels, in screen coordinates,
	with the center of the image being 0, 0.

Run tracker:
	ros2 run tracker tracker
