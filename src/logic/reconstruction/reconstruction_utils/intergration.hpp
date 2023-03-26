#ifndef INTERGRATION_HPP
#define INTERGRATION_HPP

#include "../config.hpp"
#include "miscellaneous.hpp"
#include "../../../common/Loggable.hpp"
#include "open3d/Open3D.h"

namespace logic
{
	namespace reconstruction
	{
		namespace integration
		{
			open3d::geometry::TriangleMesh integrate_rgb_frames_into_ScalableTSDFVolume(
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragment_images,
				open3d::pipelines::registration::PoseGraph& pose_graph,
				open3d::camera::PinholeCameraIntrinsic& intrinsic,
				config& cfg);

			open3d::geometry::TriangleMesh relative_integrate_rgb_frames_into_ScalableTSDFVolume(
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragment_images,
				open3d::pipelines::registration::PoseGraph& scene_pose_graph,
				open3d::pipelines::registration::PoseGraph& refined_scene_pose_graph,
				open3d::camera::PinholeCameraIntrinsic& intrinsic,
				config& cfg);
		} // namespace integration
	} // namespace reconstruction
} // namespace logic

#endif // !INTERGRATION_HPP
