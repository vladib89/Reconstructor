#ifndef DETPH_STREAM_CONFIG_HPP
#define DETPH_STREAM_CONFIG_HPP

#include "../logic/data-structures/stream_config.hpp"

namespace depth
{
	struct depth_stream_config : public logic::data_structures::stream_config
	{
		enum DepthFormat
		{
			Z16
		};
		
		int depth_fps;
		int depth_height;
		int depth_width;
		DepthFormat depth_format;
		ColorFormat color_format;
		bool enable_gyro = false;
		bool enable_accelerometer = false;
	};
} // namespace depth

#endif // !DETPH_STREAM_CONFIG_HPP
