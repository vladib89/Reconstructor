#ifndef RSDEPTHDEVICE_HPP
#define RSDEPTHDEVICE_HPP

#include "filter_options.hpp"
#include "../depth_sensor.hpp"
#include "../../common/CallBack.hpp"
#include "../../common/RsDevice.hpp"
#include "../depth_stream_config.hpp"
#include "../../common/FrameFixedQueue.hpp"
#include "../../logic/data-structures/frame.hpp"
#include "../../logic/data-structures/intrinsic.hpp"
#include "../../logic/data-structures/motion_frame.hpp"
#include "../../logic/data-structures/rotation_estimator.hpp"
#include "../../logic/data-structures/conversion_proxy.hpp"

#include <map>

namespace depth
{
	namespace realsense
	{
		class RsDepthDevice : public common::RsDevice, public depth_sensor
		{
		private:
			rs2::align* align;
			float depth_scale;
			logic::data_structures::intrinsic intrinsic;
			common::FrameFixedQueue<10> f_queue;
			rs2::pipeline_profile profile;
			std::vector<filter_options> filters;
			logic::data_structures::rotation_estimator rot_estimator;
			void processing_action() {}
			void polling_action();
			rs2::config convert_to_rs_config(depth_stream_config& cfg);
			rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
		public:
			static std::vector<RsDepthDevice> get_available_devices();
			RsDepthDevice();
			void start(common::CallBack& cb, depth_stream_config& cfg);
			void start(common::CallBack& cb, bool relocalisation);
			void start(common::CallBack& cb);
			float get_depth_scale();
			logic::data_structures::conversion_proxy get_frame();
			logic::data_structures::conversion_proxy get_frame(long time_stamp);
			logic::data_structures::intrinsic get_intrinsic();
			bool get_frames(
				logic::data_structures::frame& color,
				logic::data_structures::frame& depth,
				logic::data_structures::motion_frame& motion);
		};
	} // namespace realsense
} // namespace depth

#endif // !RSDEPTHDEVICE_HPP

