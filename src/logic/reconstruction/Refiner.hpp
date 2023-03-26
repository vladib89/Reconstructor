#ifndef REFINER_HPP
#define REFINER_HPP

#include "config.hpp"
#include "../server.hpp"
#include "open3d/Open3D.h"
#include "matching_result.hpp"
#include "../../common/Loggable.hpp"
#include "reconstruction_utils/optimization.hpp"
#include "open3d/pipelines/registration/ColoredICP.h"
#include "open3d/pipelines/registration/FastGlobalRegistration.h"

#include <vector>

namespace logic
{
	namespace reconstruction
	{
		class Refiner : public common::Loggable
		{
		public:
			Refiner();
			open3d::pipelines::registration::PoseGraph run(
				config& cfg,
				open3d::pipelines::registration::PoseGraph& pose_graph);
			std::pair<Eigen::Matrix4d, Eigen::Matrix6d> multiscale_icp(
				open3d::geometry::PointCloud& source_down,
				open3d::geometry::PointCloud& target_down,
				std::vector<float> voxel_size,
				std::vector<int> max_iter,
				config& cfg,
				Eigen::Matrix4d init_transformation = Eigen::Matrix4d::Identity(),
				bool local = false
			);
		private:
			open3d::pipelines::registration::PoseGraph make_posegraph_for_refined_scene(
				std::vector<std::string>& ply_file_names,
				open3d::pipelines::registration::PoseGraph& pose_graph,
				config& cfg
			);
			void sort_alphanum(std::vector<std::string>& str);
			bool string_comparator(const std::string& a, const std::string& b);
			std::pair<Eigen::Matrix4d, Eigen::Matrix6d> register_point_cloud_pair(
				std::vector<std::string>& ply_file_names, int s, int t,
				Eigen::Matrix4d transformation_init, config& cfg
			);
			std::pair<Eigen::Matrix4d, Eigen::Matrix6d> local_refinement(
				open3d::geometry::PointCloud& source,
				open3d::geometry::PointCloud& target,
				Eigen::Matrix4d& transformation_init,
				config& cfg
			);
			void update_posegraph_for_scene(
				int s, int t, Eigen::Matrix4d& transformation,
				Eigen::Matrix6d information, Eigen::Matrix4d& odometry,
				open3d::pipelines::registration::PoseGraph& pose_graph
			);
			void optimize_posegraph_for_refined_scene(config& cfg,
				open3d::pipelines::registration::PoseGraph& pose_graph);
		};
	} // namespace reconstruction
} // namespace logic

#endif // !REFINER_HPP
