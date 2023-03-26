#include "stepper_program.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace stepper
		{
			stepper_program::stepper_program(Stepper& stepper, config& cfg) 
				: stepper(stepper) 
			{
				set_settings(cfg);
			}

			std::pair<bool, Eigen::Matrix4d> stepper_program::next_step()
			{
				bool success = false;
				Eigen::Matrix4d rot;
				int rot_angle = 0;
				int max_range = 0;
				operating_mode mode = this->mode;

				switch (mode)
				{
				case YAW_THEN_PITCH:
					if (std::abs(current_range_yaw) <= std::abs(max_range_yaw) || !alternate)
					{
						mode = YAW_ONLY;
					}
					else
					{
						mode = PITCH_ONLY;
						current_range_pitch += pitch;
						alternate = false;
					}
				case PITCH_ONLY:
				case YAW_ONLY:
					if (mode == PITCH_ONLY)
					{
						rot_angle = pitch;
						max_range = max_range_pitch;
					}
					else
					{
						rot_angle = yaw;
						max_range = max_range_yaw;
					}

					if (rot_angle > 0)
						if (mode == YAW_ONLY)
							rot_angle = std::min(current_range_yaw, rot_angle);
						else
							rot_angle = std::min(current_range_pitch, rot_angle);
					else
						if (mode == YAW_ONLY)
							rot_angle = std::max(current_range_yaw, rot_angle);
						else
							rot_angle = std::max(current_range_pitch, rot_angle);

					if (mode == YAW_ONLY && std::abs(current_range_yaw) <= std::abs(max_range))
					{
						std::tie(success, rot) = stepper.rotate_yaw(rot_angle);
					}
					else if (mode == PITCH_ONLY && std::abs(current_range_pitch) <= std::abs(max_range))
					{
						std::tie(success, rot) = stepper.rotate_pitch(rot_angle);
					}
					else if (iterations > 1)
					{
						iterations--;

						if (mode == YAW_ONLY)
						{
							max_range_yaw = -max_range_yaw;
							current_range_yaw = yaw = -yaw;
						}
						else
						{
							max_range_pitch = -max_range_pitch;
							current_range_pitch = pitch = -pitch;
						}

						if (mode == YAW_ONLY && std::abs(current_range_yaw) <= std::abs(max_range))
						{
							std::tie(success, rot) = stepper.rotate_yaw(yaw);
						}
						else if (mode == PITCH_ONLY && std::abs(current_range_pitch) <= std::abs(max_range))
						{
							std::tie(success, rot) = stepper.rotate_pitch(pitch);
						}

						alternate = true;
					}
					else if (is_reset)
					{
						if (this->mode == YAW_THEN_PITCH)
						{
							current_range_pitch -= pitch;
							reset(1, current_range_pitch); // reset pitch
							current_range_yaw -= yaw;
							reset(0, current_range_yaw); // reset yaw
						}
						else if (mode == YAW_ONLY)
						{
							current_range_yaw -= yaw;
							reset(0, current_range_yaw); // reset yaw
						}
						else if (mode == PITCH_ONLY)
						{
							current_range_pitch -= pitch;
							reset(1, current_range_pitch); // reset pitch
						}
					}

					if (mode == YAW_ONLY)
					{
						current_range_yaw += yaw;
					}
					else if (this->mode != YAW_THEN_PITCH)
						current_range_pitch += pitch;

					break;

				}

				return std::make_pair(success, rot);
			}

			/// <summary>
			/// 
			/// </summary>
			/// <param name="p_axis">0 yaw, 1 pitch, 2 roll</param>
			void stepper_program::reset(int p_axis, int current_range)
			{
				int subtrahend;
				bool success = false;
				Eigen::Matrix4d rot;

				if (current_range > 0)
				{
					while (current_range > 0)
					{
						if (current_range - max_step >= 0)
							subtrahend = max_step;
						else
							subtrahend = current_range;

						current_range -= subtrahend;

						if (p_axis == 0) // yaw
							std::tie(success, rot) = stepper.rotate_yaw(-subtrahend);
						else if (p_axis == 1) // pitch
							std::tie(success, rot) = stepper.rotate_pitch(-subtrahend);
						//else if (p_axis == 2) roll
					}
				}
				else
				{
					while (current_range < 0)
					{
						if (current_range + max_step >= 0)
							subtrahend = max_step;
						else
							subtrahend = -current_range;

						current_range += subtrahend;

						if (p_axis == 0) // yaw
							std::tie(success, rot) = stepper.rotate_yaw(-subtrahend);
						else if (p_axis == 1) // pitch
							std::tie(success, rot) = stepper.rotate_pitch(-subtrahend);
						//else if (p_axis == 2) roll
					}
				}
			}

			void stepper_program::set_settings(config& cfg)
			{
				max_range_pitch = cfg.max_range_pitch;
				max_range_yaw = cfg.max_range_yaw;
				yaw = cfg.yaw;
				pitch = cfg.pitch;
				mode = cfg.stepper_operating_mode;

				if (mode == YAW_THEN_PITCH)
				{
					iterations = max_range_pitch / pitch + 1;
				}
				else
				{
					iterations = cfg.stepper_iterations;
				}

				is_reset = !(iterations % 2 == 0);
			}
		} // namespace stepper
	} // namespace reconstruction
} // namespace logic
