#include "SceneFragmentizer.hpp"

namespace logic
{
	namespace reconstruction
	{
		SceneFragmentizer::~SceneFragmentizer()
		{
			is_running = false;

			if (worker_sampling.joinable())
			{
				worker_sampling.join();
			}
		}

		SceneFragmentizer::SceneFragmentizer(
			config& cfg,
			depth::depth_sensor* depth_sensor,
			SceneIntegrator& scene_integrator)
			: depth_sensor(depth_sensor), scene_integrator(scene_integrator)
		{
			init(cfg);
		}

		void SceneFragmentizer::init(config& cfg)
		{
			intrinsic.SetIntrinsics(cfg.intrinsic.width, cfg.intrinsic.height, cfg.intrinsic.fx,
				cfg.intrinsic.fy, cfg.intrinsic.cx, cfg.intrinsic.cy);
			std::string path = cfg.folder_fragment;
			int success = mkdir(path.c_str());
			last_pose.setZero();

			is_running = true;
			fragment_images = new std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>();

			if (cfg.operating_mode == config::mode::VISUAL_ODOMETRY)
			{
				pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(
					odometry_transform.inverse()));
				worker_sampling = std::thread(&SceneFragmentizer::rgbd_odometry_sampling_action, this, cfg);
			}
			else if (cfg.operating_mode == config::mode::ENCODER_ROTATION_ONLY)
				worker_sampling = std::thread(&SceneFragmentizer::encoder_odometry_sampling_action, this, cfg);
		}

		void SceneFragmentizer::rgbd_odometry_sampling_action(config& cfg)
		{
			bool success;
			int alt = 0, frame_counter = 0,
				width = cfg.width, height = cfg.height,
				number_frames_to_dispose = 10;
			cv::Mat color_resized;
			cv::Mat depth_resized;
			logic::data_structures::frame color;
			logic::data_structures::frame depth;
			logic::data_structures::motion_frame motion;
			std::vector<open3d::geometry::Image> color_input;
			std::vector<open3d::geometry::Image> depth_input;
			std::vector<Eigen::Vector3d> rot_input;
			float rot[3];

			for (int i = 0; i < 2; i++)
			{
				color_input.push_back(*std::make_shared<open3d::geometry::Image>());
				depth_input.push_back(*std::make_shared<open3d::geometry::Image>());
				rot_input.push_back(Eigen::Vector3d::Identity());
				color_input[i].Prepare(width, height, 3, 1);
				depth_input[i].Prepare(width, height, 1, 2);
			}

			while (is_running)
			{

				if (depth_sensor->get_frames(color, depth, motion))
				{
					if (number_frames_to_dispose > 0)
					{
						number_frames_to_dispose--;
						continue;
					}

					opencv::opencv_utils::resize_and_copy_image_pair(color.get_data(), depth.get_data(),
						color.width, color.height,
						CV_8UC3, CV_16UC1,
						width, height,
						color_input[alt].data_.data(), depth_input[alt].data_.data());
					memcpy(rot, motion.get_data(), 3 * sizeof(float));
					rot_input[alt] = Eigen::Vector3d(rot[0], rot[1], rot[2]);

					if (frame_counter >= 1)
					{
						run_rgbd_odometry_based(std::tie(rot_input[0], color_input[0], depth_input[0]),
							std::tie(rot_input[1], color_input[1], depth_input[1]), cfg);

						rot_input[0] = rot_input[1];
						color_input[0] = color_input[1];
						depth_input[0] = depth_input[1];
					}
					else
					{
						alt++;
					}

					color.free();
					depth.free();
					motion.free();

					frame_counter++;
				}
			}
		}

		void SceneFragmentizer::encoder_odometry_sampling_action(config& cfg)
		{
			int cport_n = stepper::Stepper::find_device();
			stepper::Stepper stepper(cport_n);
			auto success = stepper.start();
			stepper::stepper_program program(stepper, cfg);
		
			if (!success)
				is_running = false; // basically terminates everything

			int width = cfg.width,
				number_frames_to_dispose = 40,
				height = cfg.height;
			Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
			cv::Mat color_resized;
			cv::Mat depth_resized;
			logic::data_structures::frame color;
			logic::data_structures::frame depth;
			logic::data_structures::motion_frame motion;
			open3d::geometry::Image color_input;
			color_input.Prepare(width, height, 3, 1);
			open3d::geometry::Image depth_input;
			depth_input.Prepare(width, height, 1, 2);

			while (is_running)
			{
				std::tie(success, trans) =
					program.next_step();

				if (success)
				{
					while (number_frames_to_dispose > 0)
					{
						color.free();
						depth.free();
						motion.free();

						if (depth_sensor->get_frames(color, depth, motion))
						{
							number_frames_to_dispose--;
						}
					}

					opencv::opencv_utils::resize_and_copy_image_pair(color.get_data(), depth.get_data(), 
						color.width, color.height, 
						CV_8UC3, CV_16UC1, 
						width, height, 
						color_input.data_.data(), depth_input.data_.data());
					run_encoder_rotation_based(color_input, depth_input, trans, cfg);

					color.free();
					depth.free();
					motion.free();

					number_frames_to_dispose = 40;
				}
				else
				{
					is_running = false;
				}
			}
		}

		bool SceneFragmentizer::get_current_pose(
			Eigen::Matrix4d& current_pose)
		{
			m_rendering.lock();
				current_pose = odometry_transform;
			m_rendering.unlock();

			return is_running;
		}

		void SceneFragmentizer::run_rgbd_odometry_based(
			std::tuple<const Eigen::Vector3d&,
			open3d::geometry::Image&,
			open3d::geometry::Image> source,
			std::tuple<const Eigen::Vector3d&,
			open3d::geometry::Image&,
			open3d::geometry::Image> target,
			config& cfg
		)
		{
			Eigen::Vector3d source_transform, target_transform;
			open3d::geometry::Image source_rgb, source_depth, target_rgb, target_depth;
			std::tie(source_transform, source_rgb, source_depth) =
				source;
			std::tie(target_transform, target_rgb, target_depth) =
				target;

			bool success;

			// start critical section
			m.lock();
			if (t == 0)
			{
				fragment_images->push_back(std::make_pair(source_rgb, source_depth));
			}
			fragment_images->push_back(std::make_pair(target_rgb, target_depth));
			m.unlock();
			// end critical section

			success = make_posegraph_for_rgbd_fragment(
				source_rgb, target_rgb, source_depth, target_depth,
				source_transform, target_transform, &s, &t, cfg);
			Eigen::Matrix4d candidate =
				pose_graph.nodes_[pose_graph.nodes_.size() - 1].pose_;

			// start critical section
			m.lock();
			if (success && t == cfg.n_frames_per_fragment)
			{
				s = t = 0;
				//optimize_posegraph_for_fragment(&pose_graph, cfg);
				//candidate = pose_graph.nodes_[pose_graph.nodes_.size() - 1].pose_;
				success = scene_integrator.run(
					fragment_id, candidate, last_pose, 
					target_rgb, target_depth,
					pose_graph, fragment_images, cfg);
				//odometry_transform = candidate.inverse();

				if (success)
				{
					last_pose = candidate;
					std::string pose_graph_optimized_name = cfg.path_dataset + "/" + 
						cfg.template_fragment_posegraph_optimized +
						std::to_string(fragment_id) + ".json";
					open3d::io::WritePoseGraph(pose_graph_optimized_name, pose_graph);
					fragment_id++;
				}
				else
				{
					delete fragment_images;
				}

				fragment_images = new std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>();
				pose_graph = open3d::pipelines::registration::PoseGraph();
				pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(
					candidate));
			}
			m.unlock();
			// start critical section
		}

		void SceneFragmentizer::run_encoder_rotation_based(
			open3d::geometry::Image& color,
			open3d::geometry::Image& depth,
			Eigen::Matrix4d& transform,
			config& cfg)
		{
			m.lock();
			fragment_images->push_back(std::make_pair(color, depth));
			m.unlock();

			make_posegraph_for_encoder_fragment(transform, &s, &t, cfg);

			m.lock();
			if (t == cfg.n_frames_per_fragment)
			{
				last_pose =
					pose_graph.nodes_[pose_graph.nodes_.size() - 1].pose_;
				s = t = 0;
				scene_integrator.run(
					fragment_id, last_pose, last_pose,
					color, depth, pose_graph, 
					fragment_images, cfg);

				std::string pose_graph_optimized_name = cfg.path_dataset + "/" + 
					cfg.template_fragment_posegraph_optimized +
					std::to_string(fragment_id) + ".json";
				open3d::io::WritePoseGraph(pose_graph_optimized_name, pose_graph);
				fragment_id++;
				fragment_images = new std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>();
				pose_graph = open3d::pipelines::registration::PoseGraph();
			}
			m.unlock();
		}

		bool SceneFragmentizer::make_posegraph_for_rgbd_fragment(
			open3d::geometry::Image& source_rgb,
			open3d::geometry::Image& target_rgb,
			open3d::geometry::Image& source_depth,
			open3d::geometry::Image& target_depth,
			Eigen::Vector3d& source_transform,
			Eigen::Vector3d& target_transform,
			int* s, int* t,
			config& cfg
		)
		{
			open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
			bool success;
			std::stringstream ss;
			Eigen::Matrix4d trans;
			Eigen::Matrix6d info;
			Eigen::Matrix4d trans_odometry_inv;

			ss.str(std::string());
			ss.clear();
			ss << "Fragment " << fragment_id << " :: matching consecutive frames.";
			logger->log(ss.str());
			ss.str(std::string());
			ss.clear();

			open3d::geometry::RGBDImage source_rgbd_image = *miscellaneous::create_rgbd_image(source_rgb,
				source_depth, true, cfg);
			open3d::geometry::RGBDImage target_rgbd_image = *miscellaneous::create_rgbd_image(target_rgb,
				target_depth, true, cfg);
			auto option = open3d::pipelines::odometry::OdometryOption();
			option.max_depth_diff_ = cfg.max_depth_diff;

			Eigen::Quaterniond source_t_quat = utils::camera_transforms::
				euler_2_quaternion(source_transform.y(),
					source_transform.z(), source_transform.x());
			Eigen::Quaterniond target_t_quat = utils::camera_transforms::
				euler_2_quaternion(target_transform.y(),
					target_transform.z(), target_transform.x());

			Eigen::Quaterniond q = utils::camera_transforms::
				quaternion_difference(source_t_quat, target_t_quat).normalized();
			Eigen::MatrixXd rot_gyro = q.toRotationMatrix();
			Eigen::MatrixXd trans_3by3 = Eigen::Matrix3d::Identity();
			std::tie(success, trans, info) =
				open3d::pipelines::odometry::ComputeRGBDOdometry(source_rgbd_image, target_rgbd_image,
					intrinsic, Eigen::Matrix4d::Identity(),
					open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(), option);

			if (success)
			{
				// odometry transform is the fragments odometry
				odometry_transform = trans * odometry_transform;
				// camera's odometry
				trans_odometry_inv = odometry_transform.inverse();
				trans_3by3 = trans.topLeftCorner(3, 3);
				double diff = utils::camera_transforms::distance_between_rot_mats(rot_gyro, trans_3by3).second * rad_to_deg;
				ss << "Gyro and calculated rotation diff " << diff << " in fragment number " << fragment_id << ".";
				logger->log(ss.str());

				if (diff < 5.0f)
				{
					(*t)++;
					*s = *t - 1;

					// camera's transform
					pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(trans_odometry_inv));
					pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(*s, *t, trans, info, false));

					if (*t % cfg.n_keyframes_per_n_frame == 0)// loop closure edge
					{
						int s = *t - cfg.n_keyframes_per_n_frame;

						source_rgbd_image = *miscellaneous::create_rgbd_image((*fragment_images)[s].first,
							(*fragment_images)[s].second, true, cfg);

						std::tie(success, trans, info) =
							open3d::pipelines::odometry::ComputeRGBDOdometry(source_rgbd_image, target_rgbd_image,
								intrinsic, Eigen::Matrix4d::Identity(),
								open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(), option);

						if (success)
						{
							pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(s, *t, trans, info, true));
						}
					}
				}
				else
				{
					success = false;
				}
			}

			return success;
		}

		void SceneFragmentizer::make_posegraph_for_encoder_fragment(
			Eigen::Matrix4d& transform,
			int* s, int* t,
			config& cfg)
		{
			std::stringstream ss;
			ss.str(std::string());
			ss.clear();
			ss << "Fragment " << fragment_id << " :: matching consecutive frames.";
			logger->log(ss.str());
			ss.str(std::string());
			ss.clear();

			Eigen::Matrix4d trans_odometry_inv;
			// odometry transform is the fragments odometry
			odometry_transform = transform * odometry_transform;
			// camera's odometry
			trans_odometry_inv = odometry_transform.inverse();

			(*t)++;
			*s = *t - 1;

			// camera's transform
			pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(trans_odometry_inv));

			if (pose_graph.nodes_.size() > 1)
				pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(*s, *t, transform,
					Eigen::Matrix6d::Identity(), false));
		}

		void SceneFragmentizer::optimize_posegraph_for_fragment(
			open3d::pipelines::registration::PoseGraph* pose_graph,
			config& cfg)
		{
			optimization::run_pose_graph_optimization(pose_graph, cfg.voxel_size * 1.4,
				cfg.preference_loop_closure_odometry);
		}
	} //namespace reconstruction
} // namespace logic