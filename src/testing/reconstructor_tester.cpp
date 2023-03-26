#include "reconstructor_tester.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace testing
		{
			void reconstructor_tester::start_tester(mode op_mode)
			{
				auto arch = depth::realsense::RsDepthDevice::get_available_devices();
				// reconstructor config resolution only
				config.height = 180;
				config.width = 320;

				switch (op_mode)
				{
				case CLI:
					start_menu();
					break;
				case REG:
					manual_config();
					break;
				}

				// depth device config
				depth::depth_stream_config depth_dev_config;
				depth_dev_config.color_format =
					logic::data_structures::stream_config::ColorFormat::RGB8;
				depth_dev_config.color_width = 960;
				depth_dev_config.color_height = 540;
				depth_dev_config.color_fps = 30;

				depth_dev_config.depth_format =
					depth::depth_stream_config::DepthFormat::Z16;
				depth_dev_config.depth_width = 640;
				depth_dev_config.depth_height = 480;
				depth_dev_config.depth_fps = 30;
				depth_dev_config.enable_accelerometer = true;
				depth_dev_config.enable_gyro = true;

				// init depth device
				depth_device =
					new depth::realsense::RsDepthDevice();

				class TestCallback : public common::CallBack, public common::Loggable
				{
				private:
					reconstructor_tester* rt;
					callback_function func;
				public:

					TestCallback(callback_function foo, 
						reconstructor_tester* rt) 
						: func(foo), rt(rt) {  }
					void on_success()
					{
						(rt->*func)();
					}
					void on_error(int result, std::string reason)
					{
						logger->log(reason);
					}
				}test_cb(&reconstructor_tester::main_thread_loop, this);

				depth_device->start(test_cb, depth_dev_config);

			}

			// THIS IS WHERE YOU FIDDLE WITH THE SETTINGS MANUALLY!!!
			void reconstructor_tester::manual_config()
			{
				config.operating_mode = 
					logic::reconstruction::config::mode::ENCODER_ROTATION_ONLY;
				config.stepper_iterations = 3;
				config.stepper_operating_mode = logic::reconstruction::stepper::operating_mode::YAW_ONLY;
				config.max_range_yaw = 90;
				config.yaw = 15;
			}

			void reconstructor_tester::main_thread_loop()
			{
				is_running = true;
				worker = std::thread(&reconstructor_tester::execution, this);

				while (is_running)
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(5));
				}

				if (worker.joinable())
				{
					worker.join();
				}
			}

			void reconstructor_tester::execution()
			{
				bool success = false;
				bool is_reconstructor_running = true;
				cv::Mat out_img;
				out_img = cv::Mat(config.height, config.width, CV_8UC3, cv::Scalar(0, 0, 0));
				void* out_img_ptr = out_img.data;
				logic::reconstruction::Reconstructor* reconstructor = 
					new logic::reconstruction::Reconstructor(depth_device, config);
				reconstructor->start();
				reconstructor->set_image(&out_img_ptr);
				reconstructor->set_running_indicator(&is_reconstructor_running);

				while (is_reconstructor_running)
				{
					if (logic::reconstruction::opencv::opencv_utils::render_image(out_img, "out"))
					{
						break;
					}
				}

				delete reconstructor;
				delete depth_device;
				is_running = false;
			}
		} // namespace testing
	} // namespace reconstruction
} // namespace logic