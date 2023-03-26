#ifndef FILTER_OPTIONS_HPP
#define FILTER_OPTIONS_HPP

#include "librealsense2/rs.hpp"
#include <iostream>
#include <map>

namespace depth
{
	namespace realsense
	{
		class filter_options
		{
		public:
			filter_options(const std::string name, rs2::filter& filter);
			filter_options(filter_options&& other);
			std::string filter_name;
			rs2::filter& filter;
			std::atomic_bool is_enabled;
		};
	} // namespace realsense
} // namespace depth

#endif // !FILTER_OPTIONS_HPP