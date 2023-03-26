#ifndef RECONSTRUCTOR_TESTER_HPP
#define RECONSTRUCTOR_TESTER_HPP

#include "opencv2/core.hpp"
#include "../common/Loggable.hpp"
#include "../depth/depth_sensor.hpp"
#include "reconstructor_test_menu.hpp"
#include "../depth/depth_stream_config.hpp"
#include "../depth/realsense/RsDepthDevice.hpp"
#include "../logic/reconstruction/Reconstructor.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace testing
		{
			enum mode
			{
				CLI,
				REG
			};

			class reconstructor_tester : public common::Loggable
			{
			private:
				bool is_running;
				int final_width;
				int final_height;
				std::thread worker;
				void manual_config();
				void execution();
				void main_thread_loop();
				depth::depth_sensor* depth_device;
				typedef void (reconstructor_tester::*callback_function)(void);
			public:
				void start_tester(mode op_mode);
			};
		} // namespace testing
	} // namespace reconstruction
} // namespace logic

#endif // !RECONSTRUCTOR_TESTER_HPP
