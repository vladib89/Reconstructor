#include "filter_options.hpp"

namespace depth
{
	namespace realsense
	{
        filter_options::filter_options(const std::string name, rs2::filter& flt) :
            filter_name(name),
            filter(flt),
            is_enabled(true)
        {
        }

        filter_options::filter_options(filter_options&& other) :
            filter_name(std::move(other.filter_name)),
            filter(other.filter),
            is_enabled(other.is_enabled.load())
        {
        }
	} // namespace realsense
} // namespace depth