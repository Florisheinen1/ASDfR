#ifndef GROUP15
#define GROUP15

#include "XenoFrt20Sim.hpp"
#include "LoopController.h"

#pragma pack(1) // https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct
{
	int this_is_a_int = 0;
	double this_is_a_double = 100.0;
	float this_is_a_float = 10.0;
	char this_is_a_char = 'R';
	bool this_is_a_bool = false;
};

#pragma pack(0)

class Group15 : public XenoFrt20Sim
{
public:
	Group15(uint write_decimator_freq, uint monitor_freq);
	~Group15();

private:
	XenoFileHandler file;
	struct ThisIsAStruct data_to_be_logged;
	LoopController controller;

	int last_left_encoder_value;
	int last_right_encoder_value;

	int corrected_left_encoder_value;
	int corrected_right_encoder_value;

	double u[4];
	double y[2];

	int get_corrected_encoder_value_difference(int new_value, int old_value) const;
	double encoder_to_radians(int corrected_encoder_value) const;

protected:
	// Functions
	int initialising() override;
	int initialised() override;
	int run() override;
	int stopping() override;
	int stopped() override;
	int pausing() override;
	int paused() override;
	int error() override;

	// current error
	int current_error = 0;
};

#endif // GROUP15