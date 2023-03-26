#include "miscellaneous.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace miscellaneous
		{
			std::shared_ptr<open3d::geometry::RGBDImage> create_rgbd_image(
				const open3d::geometry::Image& color_image,
				const open3d::geometry::Image& depth_image,
				bool convert_rgb_to_intensity, const logic::reconstruction::config& cfg)
			{
				std::shared_ptr<open3d::geometry::RGBDImage> rgbd_image =
					open3d::geometry::RGBDImage::CreateFromColorAndDepth(
						color_image, depth_image, 1.0f / cfg.depth_scale, cfg.max_depth, convert_rgb_to_intensity);
				return rgbd_image;

			}
		} // namespace miscellaneous
	} // namespace reconstruction
} // namespace logic
