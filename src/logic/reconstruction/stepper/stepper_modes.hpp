#ifndef STEPPER_MODES_HPP
#define STEPPER_MODES_HPP

namespace logic
{
	namespace reconstruction
	{
		namespace stepper
		{
			enum operating_mode
			{
				YAW_ONLY,
				PITCH_ONLY,
				YAW_THEN_PITCH,
				PITCH_THEN_YAW
			};
		} // namespace stepper
	} // namespace reconstruction
} // namespace logic

#endif // !STEPPER_MODES_HPP
