#ifndef STEPPER_PROGRAM_HPP
#define STEPPER_PROGRAM_HPP

#include "Stepper.hpp"
#include "../config.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace stepper
		{
			struct stepper_program
			{
			private:
				Stepper& stepper;
				int iterations = 1;
				int yaw = 0;
				int pitch = 0;
				int current_range_yaw = 0;
				int current_range_pitch = 0;
				int max_range_yaw = 0;
				int max_range_pitch = 0;
				bool is_reset = false;
				bool alternate = true;
				const int max_step = 99;
				void set_settings(config& cfg);
			public:
				operating_mode mode = YAW_ONLY;
				stepper_program(Stepper& stepper, config& cfg);
				std::pair<bool, Eigen::Matrix4d> next_step();
				void reset(int p_axis, int current_range);
			};
		} // namespace stepper
	} // namespace reconstruction
} // namespace logic

#endif // !STEPPER_PROGRAM_HPP
