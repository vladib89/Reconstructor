#ifndef RECONSTRUCTOR_TEST_MENU_HPP
#define RECONSTRUCTOR_TEST_MENU_HPP

#include "../logic/reconstruction/config.hpp"
#include "../common/menu.hpp"

#include <iostream>

namespace logic
{
	namespace reconstruction
	{
		namespace testing
		{
			extern logic::reconstruction::config config;
			void process_stepper_odometry_selection();
			void process_visual_odometry_selection();
			void start_menu();
		} // namespace testing
	} // namespace reconstruction
} // namespace logic

#endif // !RECONSTRUCTOR_TEST_MENU_HPP
