#ifndef STREAM_CONFIG_HPP
#define STREAM_CONFIG_HPP

#include <iostream>

namespace logic
{
	namespace data_structures
	{
		struct stream_config
		{
			enum ColorFormat
			{
				RGB8,
				BGR8
			};


			std::string device_name;
			std::string serial_number;
			int color_height;
			int color_width;
			int color_fps;
		};
	} // namespace data_structures
} // namespace logic

#endif // !STREAM_CONFIG_HPP
