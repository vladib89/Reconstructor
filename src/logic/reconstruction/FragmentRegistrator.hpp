#ifndef FRAGMENTREGISTRATOR_HPP
#define FRAGMENTREGISTRATOR_HPP

#include "config.hpp"
#include "Refiner.hpp"
#include "matching_result.hpp"
#include "../server.hpp"
#include "../../common/Loggable.hpp"
#include "../LogicCommons.hpp"
#include "../data-structures/task.hpp"
#include "open3d/Open3D.h"
#include <sstream>
#include <unordered_map>

namespace logic
{
	namespace reconstruction
	{
		class FragmentRegistrator : public common::Loggable
		{
		public:
			FragmentRegistrator();
			void run(config& cfg);
		private:
			std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> register_point_cloud_pair(
				std::vector<std::string>& ply_file_names,
				int s, int t,
				config& cfg);
			std::pair<open3d::geometry::PointCloud, open3d::pipelines::registration::Feature>
				preprocess_point_cloud(
					open3d::geometry::PointCloud& pcd, 
					config& cfg);
			std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> compute_initial_registration(
				int s, int t,
				open3d::geometry::PointCloud& source_down, 
				open3d::geometry::PointCloud& target_down,
				open3d::pipelines::registration::Feature& source_fpfh,
				open3d::pipelines::registration::Feature& target_fpfh, 
				config& cfg);
			std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> register_point_cloud_fpfh(
				open3d::geometry::PointCloud& source,
				open3d::geometry::PointCloud& target,
				open3d::pipelines::registration::Feature& source_fpfh,
				open3d::pipelines::registration::Feature& target_fpfh,
				config& cfg
			);
			std::pair<Eigen::Matrix4d, open3d::pipelines::registration::PoseGraph> update_posegraph_for_scene(
				int s, int t,
				Eigen::Matrix4d& transformation, 
				Eigen::Matrix6d& information,
				Eigen::Matrix4d& odometry, 
				open3d::pipelines::registration::PoseGraph& pose_graph);
			void optimize_pose_graph_for_scene(
				const config& cfg);
			void sort_alphanum(
				std::vector<std::string>& str);
			bool string_comparator(
				const std::string& a,
				const std::string& b);
			void make_posegraph_for_scene(
				std::vector<std::string>& ply_file_names,
				config& cfg);
		};
	} // namespace reconstruction
} // namespace logic

#endif // !FRAGMENTREGISTRATOR_HPP
