#include "reconstructor_test_menu.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace testing
		{
			logic::reconstruction::config config;

			void start_menu()
			{
				common::Menu_Option main_menu[] =
				{
				  {'1', "1 Stepper Odometry", process_stepper_odometry_selection},
				  {'2', "2 Visual Odometry", process_visual_odometry_selection},
				};

				size_t quantity_selections =
					sizeof(main_menu) / sizeof(main_menu[0]);

				std::string menu_title =
					"\n"
					"------------------------------\n"
					"         Reconstruction Menu\n"
					"------------------------------\n"
					;

				std::cout << menu_title;

				for (size_t i = 0; i < quantity_selections; ++i)
				{
					std::cout << main_menu[i].p_selection_text << "\n";
				}

				std::cout << "Enter selection, 0 to quit: ";
				char choice;
				std::cin >> choice;
				for (size_t i = 0; i < quantity_selections; ++i)
				{
					if (choice == main_menu[i].choice)
					{
						common::Menu_Processing_Function_Pointer p_function = main_menu[i].p_processing_function;
						common::clear_console();
						(p_function)();
						break;
					}
				}
			}

			void process_stepper_odometry_selection()
			{
				auto yaw_and_pitch = []() -> void
				{
					config.stepper_operating_mode = 
						logic::reconstruction::stepper::operating_mode::YAW_THEN_PITCH;
					int value;
					std::cout << "Enter max yaw (current position is 0)" << std::endl;
					std::cin >> value;
					config.max_range_yaw = value;
					std::cout << "Enter yaw step" << std::endl;
					std::cin >> value;
					config.yaw = value;
					std::cout << "Enter max pitch (current position is 0)" << std::endl;
					std::cin >> value;
					config.max_range_pitch = value;
					std::cout << "Enter pitch step" << std::endl;
					std::cin >> value;
					config.pitch = value;
				};

				auto yaw_only = []() -> void
				{
					config.stepper_operating_mode =
						logic::reconstruction::stepper::operating_mode::YAW_ONLY;
					int value;
					std::cout << "Enter max yaw (current position is 0)" << std::endl;
					std::cin >> value;
					config.max_range_yaw = value;
					std::cout << "Enter yaw step" << std::endl;
					std::cin >> value;
					config.yaw = value;
					std::cout << "Enter number of iterations" << std::endl;
					std::cin >> value;
					config.stepper_iterations = value;
				};

				auto pitch_only = []() -> void
				{
					int value;
					config.stepper_operating_mode =
						logic::reconstruction::stepper::operating_mode::PITCH_ONLY;
					std::cout << "Enter max pitch (current position is 0)" << std::endl;
					std::cin >> value;
					config.max_range_pitch = value;
					std::cout << "Enter pitch step" << std::endl;
					std::cin >> value;
					config.pitch = value;
					std::cout << "Enter number of iterations" << std::endl;
					std::cin >> value;
					config.stepper_iterations = value;
				};

				common::Menu_Option main_menu[] =
				{
				  {'1', "1 Yaw and pitch.", yaw_and_pitch},
				  {'2', "2 Yaw only.", yaw_only },
				  {'3', "3 Pitch only.", pitch_only},
				};

				size_t quantity_selections =
					sizeof(main_menu) / sizeof(main_menu[0]);

				std::string menu_title =
					"\n"
					"------------------------------\n"
					"         Stepper Odometry\n"
					"------------------------------\n"
					;

				std::cout << menu_title;
				config.operating_mode = logic::reconstruction::config::mode::ENCODER_ROTATION_ONLY;

				for (size_t i = 0; i < quantity_selections; ++i)
				{
					std::cout << main_menu[i].p_selection_text << "\n";
				}

				char choice;
				std::cin >> choice;

				for (size_t i = 0; i < quantity_selections; ++i)
				{
					if (choice == main_menu[i].choice)
					{
						common::Menu_Processing_Function_Pointer p_function = 
							main_menu[i].p_processing_function;
						(p_function)();
						break;
					}
				}
			}

			void process_visual_odometry_selection()
			{
				std::string menu_title =
					"\n"
					"------------------------------\n"
					"         Visual Odometry\n"
					"------------------------------\n"
					;

				std::cout << menu_title;
				config.operating_mode = 
					logic::reconstruction::config::mode::VISUAL_ODOMETRY;
			}
		} // namespace testing
	} // namespace reconstruction
} // namespace logic