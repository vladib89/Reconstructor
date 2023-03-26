#include "Refiner.hpp"

namespace logic
{
	namespace reconstruction
	{
		Refiner::Refiner()
		{}

		open3d::pipelines::registration::PoseGraph Refiner::run(
			config& cfg,
			open3d::pipelines::registration::PoseGraph& pose_graph)
		{
			logger->log("Refine rough registration of fragments.");
			open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
			std::vector<std::string> ply_file_names;
			logic_commons::get_files_list_with_extension(cfg.folder_fragment + "/", "ply", ply_file_names);
			sort_alphanum(ply_file_names);
			open3d::pipelines::registration::PoseGraph refined_pose_graph =
				make_posegraph_for_refined_scene(ply_file_names, pose_graph, cfg);
			optimize_posegraph_for_refined_scene(cfg, refined_pose_graph);

			return refined_pose_graph;
		}

		open3d::pipelines::registration::PoseGraph Refiner::make_posegraph_for_refined_scene(
			std::vector<std::string>& ply_file_names,
			open3d::pipelines::registration::PoseGraph& pose_graph,
			config& cfg)
		{
			size_t n_files = ply_file_names.size();
			std::unordered_map<int, matching_result> matching_results;
			int s, t;

			for (auto& edge : pose_graph.edges_)
			{
				s = edge.source_node_id_;
				t = edge.target_node_id_;
				matching_results.emplace(s * n_files + t, matching_result(s, t, edge.transformation_));
			}

			if (cfg.multi_threading)
			{
				logic::server* serv = logic::server::get_instance();

				struct register_point_cloud_pair_task : public logic::data_structures::task
				{
					config cfg;
					matching_result* mr;
					Refiner* f_this;
					std::vector<std::string> ply_file_names;

					register_point_cloud_pair_task(matching_result* mr,
						std::vector<std::string>& ply_file_names,
						config& cfg, Refiner* f_this) :
						mr(mr), ply_file_names(ply_file_names), cfg(cfg), f_this(f_this)
					{}

					void exectue_task()
					{
						std::tie(mr->transformation, mr->information) =
							f_this->register_point_cloud_pair(ply_file_names, mr->s, mr->t, mr->transformation, cfg);
					}
				};

				for (auto& r : matching_results)
				{
					data_structures::task* rpcpt = new register_point_cloud_pair_task(
						&r.second, ply_file_names, cfg, this
					);

					serv->add_task(rpcpt);
				}

				serv->wait_all_complete();
			}
			else
			{
				for (auto& r : matching_results)
				{
					std::tie(r.second.transformation, r.second.information) =
						register_point_cloud_pair(ply_file_names, r.second.s, r.second.t,
							r.second.transformation, cfg);
				}
			}

			open3d::pipelines::registration::PoseGraph pose_graph_new;
			Eigen::Matrix4d odometry = Eigen::Matrix4d::Identity();
			pose_graph_new.nodes_.push_back(
				open3d::pipelines::registration::PoseGraphNode(odometry)
			);

			for (auto& r : matching_results)
			{
				update_posegraph_for_scene(
					r.second.s, r.second.t, r.second.transformation,
					r.second.information, odometry, pose_graph_new);
			}

			open3d::io::WritePoseGraph(cfg.path_dataset + cfg.template_refined_posegraph,
				pose_graph_new);
			return pose_graph_new;
		}

		void Refiner::optimize_posegraph_for_refined_scene(config& cfg,
			open3d::pipelines::registration::PoseGraph& pose_graph)
		{
			optimization::run_pose_graph_optimization(&pose_graph,
				cfg.voxel_size * 1.4f, cfg.preference_loop_closure_registration);
		}

		void Refiner::update_posegraph_for_scene(
			int s, int t, Eigen::Matrix4d& transformation,
			Eigen::Matrix6d information, Eigen::Matrix4d& odometry,
			open3d::pipelines::registration::PoseGraph& pose_graph
		)
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
			else // loop closure case
			{
				pose_graph.edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(
						s, t, transformation, information, true
					)
				);
			}
		}

		std::pair<Eigen::Matrix4d, Eigen::Matrix6d> Refiner::register_point_cloud_pair(
			std::vector<std::string>& ply_file_names, int s, int t,
			Eigen::Matrix4d transformation_init, config& cfg
		)
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

			Eigen::Matrix4d transformation;
			Eigen::Matrix6d information;
			std::tie(transformation, information) =
				local_refinement(source, target, transformation_init,
					cfg);

			return std::make_pair(transformation, information);
		}

		std::pair<Eigen::Matrix4d, Eigen::Matrix6d> Refiner::local_refinement(
			open3d::geometry::PointCloud& source,
			open3d::geometry::PointCloud& target,
			Eigen::Matrix4d& transformation_init,
			config& cfg
		)
		{
			float voxel_size = cfg.voxel_size;
			Eigen::Matrix4d transformation;
			Eigen::Matrix6d information;
			std::vector<float> voxel_sizes = { voxel_size, voxel_size / 2.0f, voxel_size / 4.0f };
			std::vector<int> max_iter = { 100, 60 ,28 };

			std::tie(transformation, information) =
				multiscale_icp(source, target, voxel_sizes, max_iter,
					cfg, transformation_init, true);

			return std::make_pair(transformation, information);
		}

		std::pair<Eigen::Matrix4d, Eigen::Matrix6d> Refiner::multiscale_icp(
			open3d::geometry::PointCloud& source_down, open3d::geometry::PointCloud& target_down,
			std::vector<float> voxel_size, std::vector<int> max_iter, config& cfg,
			Eigen::Matrix4d init_transformation, bool local)
		{
			Eigen::Matrix4d current_transformation = init_transformation;
			Eigen::Matrix6d information_matrix = Eigen::Matrix6d::Identity();
			open3d::pipelines::registration::RegistrationResult result_icp;

			for (int i = 0; i < voxel_size.size(); i++)
			{
				float distance_threshold = cfg.voxel_size * 1.4f;
				logger->log("voxel_size " + std::to_string(voxel_size[i]));
				int iter = max_iter[i];

				// TODO: make sure the pcds are downsampled!!!
				if (local)
				{
					source_down = *std::get<0>(source_down.RemoveRadiusOutliers(20, 0.05));
					source_down = *source_down.VoxelDownSample(voxel_size[i]);
					open3d::geometry::AxisAlignedBoundingBox source_bb = source_down.GetAxisAlignedBoundingBox();

					target_down = *std::get<0>(target_down.RemoveRadiusOutliers(20, 0.05));
					target_down = *target_down.VoxelDownSample(voxel_size[i]);
					open3d::geometry::AxisAlignedBoundingBox target_bb = target_down.GetAxisAlignedBoundingBox();
					
					source_down = *source_down.Crop(target_bb);
					target_down = *target_down.Crop(source_bb);
				}

				if (cfg.icp_method.compare("point_to_point") == 0)
				{
					result_icp =
						open3d::pipelines::registration::RegistrationICP(source_down, target_down,
							distance_threshold, current_transformation,
							open3d::pipelines::registration::TransformationEstimationPointToPoint(),
							open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, iter));
				}
				else
				{
					source_down.EstimateNormals(
						open3d::geometry::KDTreeSearchParamHybrid(voxel_size[i] * 2.0f, 30));
					target_down.EstimateNormals(
						open3d::geometry::KDTreeSearchParamHybrid(voxel_size[i] * 2.0f, 30));

					if (cfg.icp_method.compare("point_to_plane") == 0)
					{
						result_icp =
							open3d::pipelines::registration::RegistrationICP(
								source_down, target_down, distance_threshold,
								current_transformation,
								open3d::pipelines::registration::TransformationEstimationPointToPlane(),
								open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, iter)
							);
					}
					else if (cfg.icp_method.compare("color") == 0)
					{
						result_icp =
							open3d::pipelines::registration::RegistrationColoredICP(
								source_down, target_down, voxel_size[i], current_transformation,
								open3d::pipelines::registration::TransformationEstimationForColoredICP(),
								open3d::pipelines::registration::ICPConvergenceCriteria(
									1e-6, 1e-6, iter)
							);
					}
				}

				//std::cout << 
				current_transformation = result_icp.transformation_;

				if (i == max_iter.size() - 1)
				{
					information_matrix = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(
						source_down, target_down, voxel_size[i] * 1.4f, result_icp.transformation_
					);
				}
			}

			return std::pair<Eigen::Matrix4d, Eigen::Matrix6d>(result_icp.transformation_, information_matrix);
		}

		void Refiner::sort_alphanum(std::vector<std::string>& str)
		{
			std::sort(str.begin(), str.end(), std::bind(&Refiner::string_comparator, this,
				std::placeholders::_1, std::placeholders::_2));
		}

		bool Refiner::string_comparator(const std::string& a, const std::string& b)
		{
			size_t idx_a;
			size_t idx_b;
			idx_a = a.find_last_of("_") + 1;
			idx_b = b.find_last_of("_") + 1;
			int num_a = std::stoi(a.substr(idx_a, a.length() - idx_a - 4));
			int num_b = std::stoi(b.substr(idx_b, b.length() - idx_b - 4));

			return num_a < num_b;
		}
	} // namespace reconstruction
} // namespace logic