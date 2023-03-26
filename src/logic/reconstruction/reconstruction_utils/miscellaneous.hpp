#ifndef MISCELLANEOUS_HPP
#define MISCELLANEOUS_HPP

#include "../config.hpp"
#include "open3d/Open3D.h"

namespace logic
{
	namespace reconstruction
	{
		namespace miscellaneous
		{
			std::shared_ptr<open3d::geometry::RGBDImage> create_rgbd_image(
				const open3d::geometry::Image& color_image,
				const open3d::geometry::Image& depth_image,
				bool convert_rgb_to_intensity, 
				const logic::reconstruction::config& cfg);
		} // namespace miscellaneous
	} // namespace reconstruction
} // namespace logic

#endif // !MISCELLANEOUS