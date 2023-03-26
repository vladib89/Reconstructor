#include "FragmentRegistrator.hpp"

namespace logic
{
	namespace reconstruction
	{
		FragmentRegistrator::FragmentRegistrator()
		{}

		void FragmentRegistrator::run(config& cfg)
		{
			std::vector<std::string> ply_file_names;
			logic_commons::get_files_list_with_extension(cfg.folder_fragment + "/", "ply", ply_file_names);
			sort_alphanum(ply_file_names);
			int success = mkdir(cfg.folder_scene.c_str());
			make_posegraph_for_scene(ply_file_names, cfg);
			optimize_pose_graph_for_scene(cfg);
		}

		void FragmentRegistrator::make_posegraph_for_scene(std::vector<std::string>& ply_file_names, config& cfg)
		{
			open3d::pipelines::registration::PoseGraph pose_graph;
			Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();
			std::unordered_map<int, matching_result> matching_results;
			pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odometry));

			int n_files = ply_file_names.size();

			for (int s = 0; s < n_files; s++)
			{
				for (int t = s + 1; t < n_files; t++)
				{
					matching_results.emplace(s * n_files + t, matching_result(s, t));
				}
			}

			if (cfg.multi_threading)
			{
				logic::server* serv = logic::server::get_instance();

				struct register_point_cloud_pair_task : public logic::data_structures::task
				{
					config cfg;
					matching_result* mr;
					FragmentRegistrator f_this;
					std::vector<std::string> ply_file_names;

					register_point_cloud_pair_task(matching_result* mr,
						std::vector<std::string>& ply_file_names,
						config& cfg) :
						mr(mr), ply_file_names(ply_file_names), cfg(cfg)
					{}

					void exectue_task()
					{
						std::tie(mr->success, mr->transformation, mr->information) =
							f_this.register_point_cloud_pair(ply_file_names, mr->s, mr->t, cfg);
					}
				};

				for (auto& r : matching_results)
				{
					data_structures::task* rpcpt = new register_point_cloud_pair_task(
						&r.second, ply_file_names, cfg
					);

					serv->add_task(rpcpt);
				}

				serv->wait_all_complete();
			}
			else
			{
				for (auto& r : matching_results)
				{
					std::tie(r.second.success, r.second.transformation, r.second.information) =
						register_point_cloud_pair(ply_file_names, r.second.s, r.second.t, cfg);
				}
			}

			for (auto& r : matching_results)
			{
				if (r.second.success)
				{
					std::tie(odometry, pose_graph) =
						update_posegraph_for_scene(r.second.s, r.second.t, r.second.transformation,
							r.second.information, odometry, pose_graph);
				}
			}

			open3d::io::WritePoseGraph(cfg.path_dataset + "/" + cfg.template_global_posegraph, pose_graph);
		}

		std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> FragmentRegistrator::register_point_cloud_pair(
			std::vector<std::string>& ply_file_names, int s, int t, config& cfg)
		{
			std::stringstream ss;
			ss << "reading " << ply_file_names[s] << " ...";
			logger->log(ss.str());
			ss.str(std::string());
			ss.clear();
			open3d::geometry::PointCloud source;
			open3d::io::ReadPointCloud(ply_file_names[s], source);

			ss << "reading " << ply_file_names[t] << " ...";
			logger->log(ss.str());
			ss.str(std::string());
			ss.clear();
			open3d::geometry::PointCloud target;
			open3d::io::ReadPointCloud(ply_file_names[t], target);

			open3d::geometry::PointCloud source_down;
			open3d::geometry::PointCloud target_down;
			open3d::pipelines::registration::Feature source_fpfh;
			open3d::pipelines::registration::Feature target_fpfh;

			std::tie(source_down, source_fpfh) = preprocess_point_cloud(source, cfg);
			std::tie(target_down, target_fpfh) = preprocess_point_cloud(target, cfg);

			bool success;
			Eigen::Matrix4d transformation;
			Eigen::Matrix6d information;

			std::tie(success, transformation, information) = compute_initial_registration(
				s, t, source_down, target_down, source_fpfh, target_fpfh, cfg
			);

			if (t != s + 1 && !success)
			{
				return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
					false, Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Identity()
					);
			}

			return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
				success, transformation, information
				);
		}

		std::pair<open3d::geometry::PointCloud, open3d::pipelines::registration::Feature>
			FragmentRegistrator::preprocess_point_cloud(open3d::geometry::PointCloud& pcd, config& cfg)
		{
			float voxel_size = cfg.voxel_size;
			open3d::geometry::PointCloud pcd_down = *pcd.VoxelDownSample(voxel_size);
			pcd_down.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2.0f, 30));
			open3d::pipelines::registration::Feature pcd_fpfh = *open3d::pipelines::registration::ComputeFPFHFeature(
				pcd_down, open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5.0f, 100));

			return std::pair<open3d::geometry::PointCloud, open3d::pipelines::registration::Feature>
				(pcd_down, pcd_fpfh);
		}

		std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> FragmentRegistrator::compute_initial_registration(int s, int t,
			open3d::geometry::PointCloud& source_down, open3d::geometry::PointCloud& target_down,
			open3d::pipelines::registration::Feature& source_fpfh,
			open3d::pipelines::registration::Feature& target_fpfh, config& cfg)
		{
			Refiner refiner;
			Eigen::Matrix4d transformation;
			Eigen::Matrix6d information;
			bool success;

			if (t == s + 1) // odometry case
			{
				logger->log("Using RGBD odometry");
				open3d::pipelines::registration::PoseGraph pose_graph_frag;
				std::string pose_graph_name = cfg.path_dataset + "/" + cfg.template_fragment_posegraph_optimized +
					std::to_string(s) + ".json";
				open3d::io::ReadPoseGraph(pose_graph_name, pose_graph_frag);

				int n_nodes = pose_graph_frag.nodes_.size();
				Eigen::Matrix4d_u transformation_init = pose_graph_frag.nodes_[n_nodes - 1].pose_.inverse();
				std::vector<float> voxel_size = { cfg.voxel_size };
				std::vector<int> max_iter = { 50 };
				std::tie(transformation, information) =
					refiner.multiscale_icp(source_down, target_down, voxel_size, max_iter, cfg, transformation_init);
			}
			else // loop closure case
			{
				std::tie(success, transformation, information) =
					register_point_cloud_fpfh(source_down, target_down,
						source_fpfh, target_fpfh, cfg);

				if (!success)
				{
					logger->log("No reasonable solution. Skip this pair");
					return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
						false, Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Zero()
						);
				}
			}

			std::stringstream ss;
			ss << transformation;
			logger->log(ss.str());

			return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
				true, transformation, information);
		}

		std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d> FragmentRegistrator::register_point_cloud_fpfh(
			open3d::geometry::PointCloud& source,
			open3d::geometry::PointCloud& target,
			open3d::pipelines::registration::Feature& source_fpfh,
			open3d::pipelines::registration::Feature& target_fpfh,
			config& cfg
		)
		{
			float distance_threshold = cfg.voxel_size * 1.4f;
			open3d::pipelines::registration::RegistrationResult result;

			if (cfg.global_registration.compare("ransac") == 0)
			{
				std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>>
					checkers;
				auto correspondence_checker_edge_length =
					open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(
						0.9);
				auto correspondence_checker_distance =
					open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(
						distance_threshold);
				checkers.push_back(correspondence_checker_edge_length);
				checkers.push_back(correspondence_checker_distance);

				/*result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(
					source, target, source_fpfh, target_fpfh, distance_threshold,
					open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 4,
					checkers, open3d::pipelines::registration::RANSACConvergenceCriteria(4000000, 500));*/
			}

			if (result.transformation_.trace() == 4.0)
			{
				return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
					false, Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Identity()
					);
			}

			Eigen::Matrix6d information = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
				source, target, distance_threshold, result.transformation_
			);

			if (information(5, 5) / std::min(source.points_.size(), target.points_.size()) < 0.3)
			{
				return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
					false, Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Zero()
					);
			}

			return std::tuple<bool, Eigen::Matrix4d, Eigen::Matrix6d>(
				true, result.transformation_, information
				);
		}

		std::pair<Eigen::Matrix4d, open3d::pipelines::registration::PoseGraph> FragmentRegistrator::update_posegraph_for_scene(
			int s, int t, Eigen::Matrix4d& transformation, Eigen::Matrix6d& information,
			Eigen::Matrix4d& odometry, open3d::pipelines::registration::PoseGraph& pose_graph)
		{
			if (t == s + 1) // odometry case
			{
				odometry = transformation * odometry;
				Eigen::Matrix4d odometry_inv = odometry.inverse();
				pose_graph.nodes_.push_back(
					open3d::pipelines::registration::PoseGraphNode(odometry_inv)
				);
				pose_graph.edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(
						s, t, transformation, information, false
					)
				);
			}
			else // loop closure
			{
				pose_graph.edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(
						s, t, transformation, information, true
					)
				);
			}

			return std::make_pair(odometry, pose_graph);
		}

		void FragmentRegistrator::optimize_pose_graph_for_scene(const config& cfg)
		{
			std::string pose_graph_name = cfg.path_dataset + "/" + cfg.template_global_posegraph;
			std::string pose_graph_optimized_name = cfg.path_dataset + "/" +
				cfg.template_global_posegraph_optimized;

		/*	optimization::run_pose_graph_optimization(pose_graph_name, pose_graph_optimized_name,
				cfg.voxel_size * 1.4f, cfg.preference_loop_closure_registration);*/
		}

		void FragmentRegistrator::sort_alphanum(std::vector<std::string>& str)
		{
			std::sort(str.begin(), str.end(), std::bind(&FragmentRegistrator::string_comparator, this,
				std::placeholders::_1, std::placeholders::_2));
		}

		bool FragmentRegistrator::string_comparator(const std::string& a, const std::string& b)
		{
			size_t idx_a;
			size_t idx_b;
			idx_a = a.find_last_of("_") + 1;
			idx_b = b.find_last_of("_") + 1;
			int num_a = std::stoi(a.substr(idx_a, a.length() - idx_a - 4));
			int num_b = std::stoi(b.substr(idx_b, b.length() - idx_b - 4));

			return num_a < num_b;
		}
	}
}