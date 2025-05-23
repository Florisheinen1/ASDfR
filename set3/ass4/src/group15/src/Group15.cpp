#include "Group15.hpp"
#include <math.h>

#define MAX_ENCODER_VALUE 16384.0
#define GEAR_RATIO 15.58
#define FULL_CIRCLE_ENCODER (4.0 * 1024.0)

Group15::Group15(uint write_decimator_freq, uint monitor_freq) : XenoFrt20Sim(write_decimator_freq, monitor_freq, file, &data_to_be_logged),
																 file(1, "/home/asdfr-15/logs/log", "bin"), controller()
{
	printf("%s: Constructing rampio\n", __FUNCTION__);
	// Add variables to logger to be logged, has to be done before you can log data
	logger.addVariable("this_is_a_int", integer);
	logger.addVariable("this_is_a_double", double_);
	logger.addVariable("this_is_a_float", float_);
	logger.addVariable("this_is_a_char", character);
	logger.addVariable("this_is_a_bool", boolean);

	// To infinite run the controller, uncomment line below
	controller.SetFinishTime(0.0);

	this->last_left_encoder_value = 0;
	this->last_right_encoder_value = 0;
	this->corrected_left_encoder_value = 0;
	this->corrected_right_encoder_value = 0;
}

Group15::~Group15()
{
}

int Group15::initialising()
{
	// Set physical and cyber system up for use in a
	// Return 1 to go to initialised state

	monitor.printf("Hello from initialising\n"); // Do something

	// The logger has to be initialised at only once
	logger.initialise();
	// The FPGA has to be initialised at least once
	ico_io.init();

	return 1;
}

int Group15::initialised()
{
	// Keep the physical syste in a state to be used in the run state
	// Call start() or return 1 to go to run state

	monitor.printf("Hello from initialised\n"); // Do something

	return 0;
}

int Group15::get_corrected_encoder_value_difference(int new_value, int old_value) const {
	int half_max_encoder_value = MAX_ENCODER_VALUE / 2;

	int raw_difference = new_value - old_value;
	int corrected_difference = raw_difference;

	// Correct difference
	if (raw_difference > half_max_encoder_value) {
		corrected_difference -= MAX_ENCODER_VALUE;
	} else if (raw_difference < -half_max_encoder_value) {
		corrected_difference += MAX_ENCODER_VALUE;
	}

	return corrected_difference;
}

double Group15::encoder_to_radians(int corrected_encoder_value) const {
	double encoder_to_radians_ratio = (2*M_PI) / (GEAR_RATIO * 4 * 1024);
	double radians = corrected_encoder_value * encoder_to_radians_ratio;
	return radians;
}

int Group15::run()
{
	// Do what you need to do
	// Return 1 to go to stopping state

	// Start logger
	logger.start();

	// Fix encoder wrapping
	auto right_wheel_encoder = this->sample_data.channel1;
	auto left_wheel_encoder = this->sample_data.channel2;
	// Undo wrapping
	int corrected_left_diff = get_corrected_encoder_value_difference(left_wheel_encoder, this->last_left_encoder_value);
	int corrected_right_diff = get_corrected_encoder_value_difference(right_wheel_encoder, this->last_right_encoder_value);
	// Calculate total amount of rotations in radians
	this->corrected_left_encoder_value += corrected_left_diff;
	this->corrected_right_encoder_value += corrected_right_diff;
	// Update old encoder values
	this->last_left_encoder_value = left_wheel_encoder;
	this->last_right_encoder_value = right_wheel_encoder;

	// Calculate the angle of the wheels (can be multiple rotations)
	auto left_wheel_radians = encoder_to_radians(this->corrected_left_encoder_value);
	// And negate the right wheel for being inverted
	auto right_wheel_radians = encoder_to_radians(-this->corrected_right_encoder_value);

	auto target_left_wheel_speed = this->ros_data.left_wheel_speed;
	auto target_right_wheel_speed = this->ros_data.right_wheel_speed;

	// Set the PID controller input
	u[0] = left_wheel_radians;
	u[1] = right_wheel_radians;
	u[2] = target_left_wheel_speed;
	u[3] = target_right_wheel_speed;

	controller.Calculate(u, y);

	// Read the output
	auto controlled_left_speed = u[0];
	auto controlled_right_speed = u[1];

	// And send the motor power
	actuate_data.pwm1 = controlled_right_speed; // TODO: Check if this needs a minus sign
	actuate_data.pwm2 = controlled_left_speed;

	// For debugging only
	double left_power_percentage = (controlled_left_speed / 2048.0) * 100.0;
	double right_power_percentage = (controlled_right_speed / 2048.0) * 100.0;
	
	monitor.printf("Target speed: %f, %f\n", target_left_wheel_speed, target_right_wheel_speed);
	
	evl_printf("Encoder left: %d, right: %d\n", this->corrected_left_encoder_value, this->corrected_right_encoder_value);
	evl_printf("Angle left: %f, right: %f\n", left_wheel_radians, right_wheel_radians);
	evl_printf("Power left: %f, right: %f\n", left_power_percentage, right_power_percentage);
	
	if (controller.IsFinished())
		return 1;

	return 0;
}

int Group15::stopping()
{
	// Bring the physical system to a stop and set it in a state that the system can be deactivated
	// Return 1 to go to stopped state
	logger.stop();							 // Stop logger
	monitor.printf("Hello from stopping\n"); // Do something

	return 1;
}

int Group15::stopped()
{
	// A steady state in which the system can be deactivated whitout harming the physical system

	monitor.printf("Hello from stopping\n"); // Do something

	return 0;
}

int Group15::pausing()
{
	// Bring the physical system to a stop as fast as possible without causing harm to the physical system

	monitor.printf("Hello from pausing\n"); // Do something
	return 1;
}

int Group15::paused()
{
	// Keep the physical system in the current physical state

	monitor.printf("Hello from paused\n"); // Do something
	return 0;
}

int Group15::error()
{
	// Error detected in the system
	// Can go to error if the previous state returns 1 from every other state function but initialising

	monitor.printf("Hello from error\n"); // Do something

	return 0;
}
