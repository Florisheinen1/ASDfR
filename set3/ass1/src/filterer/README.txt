Package filterer
-----------------------------------------------
Description: This package filters and processes images
Contains two separate nodes, each with their own filter
functionality.

----------------------------------------------
1. Node masker
Masks received image based on green pixels. Green input
results in white output pixel. Black otherwise.

Inputs:
- /image
	Type: sensor_msgs::msg::Image
	A normal, colored image

Outputs:
- /mask
	Type: sensor_msgs::msg::Image
	A green-mask of received image

Run masker:
	ros2 run filterer masker
-----------------------------------------------
2. Node greyscaler
Converts received image into greyscale

Inputs:
- /image
	Type: sensor_msgs::msg::Image
	A normal, colored image

Outputs:
- /greyscale
	Type: sensor_msgs::msg::Image
	A greyscale image

Run greyscaler:
	ros2 run filterer greyscaler