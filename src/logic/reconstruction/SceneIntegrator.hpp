#ifndef SCENEINTEGRATOR_HPP
#define SCENEINTEGRATOR_HPP

#include "config.hpp"
#include "../../common/Loggable.hpp"
#include "reconstruction_utils/intergration.hpp"
#include "reconstruction_utils/miscellaneous.hpp"
#include "reconstruction_utils/image_transforms.hpp" 

#include <queue>

namespace logic
{
	namespace reconstruction
	{
		class SceneIntegrator : public common::Loggable
		{
		public:
			SceneIntegrator(config& cfg);
			~SceneIntegrator();
			bool run(
				int fragment_id, 
				Eigen::Matrix4d& candidate,
				Eigen::Matrix4d& last_pose, 
				open3d::geometry::Image& rgb,
				open3d::geometry::Image& depth,
				open3d::pipelines::registration::PoseGraph& fragment_pose_graph,
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>* fragment_frames,
				config& cfg);
			bool get_image_and_pose(
				void** color_img,
				void** depth_img,
				Eigen::Matrix4d& img_pose);
			open3d::pipelines::registration::PoseGraph get_scene_pose_grpah(
				config& cfg);
			std::vector<std::tuple<Eigen::Matrix4d, void*, void*>> get_scene_rgbds();
		private:
			bool is_running = false;
			std::mutex m_rendering;
			std::thread worker_fragment_processing;
			open3d::pipelines::registration::PoseGraph scene_pose_graph;
			std::queue<std::tuple<std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>*,
				open3d::pipelines::registration::PoseGraph, std::pair<int, config>>> fragment_queue;
			std::vector<std::tuple<Eigen::Matrix4d, void*, void*>> poses_and_frames;
			open3d::camera::PinholeCameraIntrinsic intrinsic;
			bool check_if_scene_needs_update_and_update(
				int fragment_id, 
				Eigen::Matrix4d& candidate,
				Eigen::Matrix4d& last_pose, 
				open3d::geometry::Image& rgb,
				open3d::geometry::Image& depth,
				config& cfg);
			void update_scene(
				int fragment_id,
				Eigen::Matrix4d transform,
				open3d::geometry::Image& rgb,
				open3d::geometry::Image& depth);
			void process_fragment();
			void init(config& cfg);
		};
	} // namespace reconstruction
} // namespace logic

#endif // !SCENEINTEGRATOR_HPP
