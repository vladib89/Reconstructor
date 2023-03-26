#include "SceneIntegrator.hpp"

namespace logic
{
	namespace reconstruction
	{
		SceneIntegrator::SceneIntegrator(config& cfg)
		{
			init(cfg);
		}

		void SceneIntegrator::init(config& cfg)
		{
			intrinsic.SetIntrinsics(cfg.intrinsic.width, cfg.intrinsic.height, cfg.intrinsic.fx,
				cfg.intrinsic.fy, cfg.intrinsic.cx, cfg.intrinsic.cy);

			is_running = true;
			worker_fragment_processing = std::thread(&SceneIntegrator::process_fragment, this);
		}

		SceneIntegrator::~SceneIntegrator()
		{
			is_running = false;

			if (worker_fragment_processing.joinable())
			{
				worker_fragment_processing.join();
			}
		}

		bool SceneIntegrator::run(
			int fragment_id,
			Eigen::Matrix4d& candidate,
			Eigen::Matrix4d& last_pose,
			open3d::geometry::Image& rgb,
			open3d::geometry::Image& depth,
			open3d::pipelines::registration::PoseGraph& fragment_pose_graph,
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>* fragment_frames,
			config& cfg)
		{
			bool success =
				check_if_scene_needs_update_and_update(fragment_id, candidate,
					last_pose, rgb, depth, cfg);

			if (success)
			{
				fragment_queue.push({ fragment_frames, fragment_pose_graph, {fragment_id, cfg} });
			}

			return success;
		}

		bool SceneIntegrator::check_if_scene_needs_update_and_update(
			int fragment_id,
			Eigen::Matrix4d& candidate,
			Eigen::Matrix4d& last_pose,
			open3d::geometry::Image& rgb,
			open3d::geometry::Image& depth,
			config& cfg
		)
		{
			config::mode mode = cfg.operating_mode;
			bool success = false;

			if (poses_and_frames.empty() || mode == config::mode::ENCODER_ROTATION_ONLY)
			{
				success = true;
				update_scene(fragment_id, candidate, rgb, depth);
			}
			else
			{
				float angle_diff, translation_diff;
				Eigen::MatrixXd last_pose_rotation = last_pose.topLeftCorner(3, 3);
				Eigen::MatrixXd candidate_rotation = candidate.topLeftCorner(3, 3);
				translation_diff =
					(candidate.topRightCorner(3, 1) - last_pose.topRightCorner(3, 1)).norm();
				std::tie(success, angle_diff)
					= utils::camera_transforms::distance_between_rot_mats(last_pose_rotation, candidate_rotation);
				angle_diff *= rad_to_deg;

				if (success && (angle_diff > cfg.fragment_threshold_angle
					|| translation_diff > cfg.fragment_threshold_translation))
				{
					Eigen::MatrixXd r;

					for (auto& p : poses_and_frames)
					{
						r = std::get<0>(p).topLeftCorner(3, 3);
						angle_diff =
							utils::camera_transforms::distance_between_rot_mats(candidate_rotation, r).second * rad_to_deg;
						translation_diff = (candidate.topRightCorner(3, 1) - std::get<0>(p).topRightCorner(3, 1)).norm();

						if (angle_diff < cfg.scene_threshold_angle &&
							translation_diff < cfg.scene_threshold_translation)
						{
							success = false;
							break;
						}
					}

					if (success) // transform not found in scene pose graph
					{
						update_scene(fragment_id, candidate, rgb, depth);
					}
				}
				else
				{
					success = false;
				}
			}

			return success;
		}

		void SceneIntegrator::update_scene(
			int fragment_id,
			Eigen::Matrix4d transform,
			open3d::geometry::Image& rgb,
			open3d::geometry::Image& depth)
		{
			void* scene_rgb_image;
			void* scene_depth_image;
			scene_rgb_image = malloc(rgb.data_.size());
			scene_depth_image = malloc(depth.data_.size());
			memcpy(scene_rgb_image, rgb.data_.data(), rgb.data_.size());
			memcpy(scene_depth_image, depth.data_.data(), depth.data_.size());

			// start critical section
			m_rendering.lock();
			poses_and_frames.push_back({ transform, scene_rgb_image, scene_depth_image });
			m_rendering.unlock();
			// end critical section

			scene_pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(
				transform));

			if (scene_pose_graph.nodes_.size() > 1)
			{
				// edge is identity as the fragments are generated in scene transform already
				scene_pose_graph.edges_.push_back(
					open3d::pipelines::registration::PoseGraphEdge(fragment_id - 1, fragment_id,
						Eigen::Matrix4d::Identity(), Eigen::Matrix6d::Identity(), false));
			}
		}

		void SceneIntegrator::process_fragment()
		{
			config cfg;
			int fragment_id;
			open3d::pipelines::registration::PoseGraph pose_graph;
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>* fragment_images;

			while (is_running || !fragment_queue.empty())
			{
				if (!fragment_queue.empty())
				{
					auto f = fragment_queue.front();
					fragment_queue.pop();
					fragment_id = std::get<2>(f).first;
					cfg = std::get<2>(f).second;
					pose_graph = std::get<1>(f);
					fragment_images = std::get<0>(f);

					logger->log("Integrate RGBD sequence using estimated camera pose.");
					logger->log("Fragment " + std::to_string(fragment_id) + " is unique, generating pcd.");

					open3d::geometry::TriangleMesh mesh =
						integration::integrate_rgb_frames_into_ScalableTSDFVolume(*fragment_images,
							pose_graph, intrinsic, cfg);
					delete fragment_images;
					open3d::geometry::PointCloud pcd = open3d::geometry::PointCloud();
					pcd.points_ = mesh.vertices_;
					pcd.colors_ = mesh.vertex_colors_;

					std::string pcd_name = cfg.path_dataset + "/" + cfg.template_fragment_pointcloud +
						std::to_string(fragment_id) + ".ply";
					open3d::io::WritePointCloudOption options;
					options.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Binary;
					options.compressed = open3d::io::WritePointCloudOption::Compressed::Compressed;
					options.print_progress = false;
					open3d::io::WritePointCloud(pcd_name, pcd, options);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(5));
				}
			}
		}

		bool SceneIntegrator::get_image_and_pose(
			void** color_img,
			void** depth_img,
			Eigen::Matrix4d& img_pose)
		{
			bool success = false;
			Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

			m_rendering.lock();
			if (poses_and_frames.size() > 0)
			{
				*color_img = std::get<1>(poses_and_frames[0]);
				*depth_img = std::get<2>(poses_and_frames[0]);
				img_pose = std::get<0>(poses_and_frames[0]);
				success = true;
			}
			m_rendering.unlock();

			return success;
		}

		open3d::pipelines::registration::PoseGraph SceneIntegrator::get_scene_pose_grpah(config& cfg)
		{
			/*	optimization::run_pose_graph_optimization(&scene_pose_graph,
					cfg.voxel_size * 1.4f, cfg.preference_loop_closure_odometry);*/
			open3d::io::WritePoseGraph(cfg.path_dataset + cfg.template_global_posegraph_optimized, scene_pose_graph);

			/*	Eigen::Matrix4d odometry = scene_pose_graph.nodes_[0].pose_.inverse();

				for (int i = 0; i < scene_pose_graph.edges_.size(); i++)
				{
					odometry = scene_pose_graph.edges_[i].transformation_ * odometry;
					std::cout << odometry.inverse() << std::endl;
				}*/

			return scene_pose_graph;
		}


		std::vector<std::tuple<Eigen::Matrix4d, void*, void*>> SceneIntegrator::get_scene_rgbds()
		{
			return poses_and_frames;
		}
	} // naemspace reconstruction
} // namespace logic