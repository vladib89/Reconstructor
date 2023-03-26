#include "Reconstructor.hpp"
#include "opencv/opencv_utils.hpp"

namespace logic
{
	namespace reconstruction
	{
		Reconstructor::Reconstructor(depth::depth_sensor* depth_sensor,
			config& cfg) : depth_sensor(depth_sensor), cfg(cfg)
		{
			init_config_and_folders();
		}

		Reconstructor::~Reconstructor()
		{
			is_running = false;

			if (worker.joinable())
			{
				worker.join();
			}
		}

		void Reconstructor::init_config_and_folders()
		{
			cfg.multi_threading = true;
			cfg.debug = false;
			cfg.n_frames_per_fragment = 1;
			cfg.n_keyframes_per_n_frame = 2;
			cfg.max_depth = 3.f;
			cfg.depth_scale = depth_sensor->get_depth_scale();
			cfg.clipping_distance = (unsigned short)(cfg.max_depth / cfg.depth_scale);
			cfg.min_depth = 0.3f;
			cfg.voxel_size = 0.005;
			cfg.max_depth_diff = 0.07;
			cfg.preference_loop_closure_odometry = 0.1;
			cfg.preference_loop_closure_registration = 5.0;
			cfg.tsdf_cubic_size = 3.0;
			cfg.fragment_threshold_angle = 30.0;
			cfg.fragment_threshold_translation = 0.5;
			cfg.scene_threshold_angle = 5.0;
			cfg.scene_threshold_translation = 0.1;
			cfg.path_dataset = "dataset";
			cfg.icp_method = "point_to_point";
			cfg.depth_map_type = "redwood";
			cfg.global_registration = "ransac";
			cfg.folder_fragment = cfg.path_dataset + "/fragments";
			cfg.folder_scene = cfg.path_dataset + "/scene/";
			cfg.template_fragment_posegraph = "fragments/fragment_";
			cfg.template_fragment_pointcloud = "fragments/fragment_";
			cfg.template_fragment_posegraph_optimized = "fragments/fragment_optimized_";
			cfg.template_global_posegraph = "/scene/global_registration.json";
			cfg.template_global_posegraph_optimized = "/scene/global_registration_optimized.json";
			cfg.template_refined_posegraph = "/scene/refined_registration.json";
			cfg.template_refined_posegraph_optimized = "scene/refined_registration_optimized.json";
			cfg.template_global_point_cloud = "scene/integrated.ply";
			logic::logic_commons::mkdir(cfg.path_dataset.c_str());
			logic::logic_commons::mkdir(cfg.folder_scene.c_str());

			auto intr = depth_sensor->get_intrinsic();
			float resize_along_x = (float)cfg.width / intr.width;
			float resize_along_y = (float)cfg.height / intr.height;
			intr.cx *= resize_along_x;
			intr.cy *= resize_along_y;
			intr.fx *= resize_along_x;
			intr.fy *= resize_along_y;
			intr.height = cfg.height;
			intr.width = cfg.width;
			cfg.intrinsic = intr;

			intrinsic.SetIntrinsics(cfg.width,
				cfg.height,
				cfg.intrinsic.fx,
				cfg.intrinsic.fy,
				cfg.intrinsic.cx,
				cfg.intrinsic.cy);
		}

		bool Reconstructor::set_image(void** img)
		{
			out_img_external_ptr = *img;
			return true;
		}

		bool Reconstructor::start()
		{
			is_running = true;
			scene_integrator = new SceneIntegrator(cfg);
			scene_fragmentizer = new SceneFragmentizer(cfg, depth_sensor, *scene_integrator);
			worker = std::thread(&Reconstructor::execution, this);
			return is_running;
		}

		std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>
			Reconstructor::convert_to_open3d_images(
				const std::vector<std::tuple<Eigen::Matrix4d, void*, void*>>& poses_and_fragment_frames)
		{
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>> res;

			for (int i = 0; i < poses_and_fragment_frames.size(); i++)
			{
				auto x = poses_and_fragment_frames[i];
				open3d::geometry::Image color;
				open3d::geometry::Image depth;
				color.Prepare(cfg.width, cfg.height, 3, 1);
				depth.Prepare(cfg.width, cfg.height, 1, 2);
				memcpy(color.data_.data(), std::get<1>(x), color.data_.size());
				memcpy(depth.data_.data(), std::get<2>(x), depth.data_.size());
				res.push_back({ color, depth });
			}

			return res;
		}

		void Reconstructor::execution()
		{
			bool success = false;
			void* color_ptr;
			void* depth_ptr;
			Eigen::Matrix4d img_pose;
			Eigen::Matrix4d current_pose;

			while (is_running)
			{
				success = scene_integrator->get_image_and_pose(&color_ptr, &depth_ptr, img_pose);
				is_running = scene_fragmentizer->get_current_pose(current_pose);

				if (success)
				{
					cv::Mat color = cv::Mat(cv::Size(cfg.width, cfg.height), CV_8UC3, color_ptr);
					cv::Mat depth = cv::Mat(cv::Size(cfg.width, cfg.height), CV_16UC1, depth_ptr);
					Eigen::MatrixXd image_transform_inv =
						logic::reconstruction::utils::camera_transforms::transform_matrix_difference(img_pose, current_pose);
					logic::reconstruction::opencv::opencv_utils::transform_2d_in_3d_space(
						color,
						depth,
						out_img, cfg.intrinsic,
						cfg.depth_scale,
						convert_eigen_matrix_to_cv<double>(image_transform_inv, CV_64F));
					memcpy(out_img_external_ptr, out_img.data, out_img.total() * out_img.elemSize());
				}
			}

			depth_sensor->stop();
			open3d::pipelines::registration::PoseGraph fragments_pose_graph =
				scene_integrator->get_scene_pose_grpah(cfg);

			std::vector<std::tuple<Eigen::Matrix4d, void*, void*>> poses_and_fragment_frames =
				scene_integrator->get_scene_rgbds();
			delete scene_fragmentizer;
			delete scene_integrator;
			auto frames = convert_to_open3d_images(poses_and_fragment_frames);
			generate_final_pcds(frames, fragments_pose_graph);

			*main_is_running = false;
		}

		void Reconstructor::generate_final_pcds(
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragments,
			open3d::pipelines::registration::PoseGraph& fragments_pose_graph
		)
		{
			open3d::io::WritePointCloudOption options;
			options.write_ascii = open3d::io::WritePointCloudOption::IsAscii::Binary;
			options.compressed = open3d::io::WritePointCloudOption::Compressed::Compressed;
			options.print_progress = false;
			open3d::geometry::PointCloud pcd = open3d::geometry::PointCloud();
			std::string pcd_name = cfg.path_dataset + "/integrated.ply";
			std::string pcd_name_refined = cfg.path_dataset + "/integrated_refined.pcd";

			logger->log("Generating scene mesh.");
			auto mesh = integration::integrate_rgb_frames_into_ScalableTSDFVolume(
				fragments,
				fragments_pose_graph,
				intrinsic, cfg);
			pcd.points_ = mesh.vertices_;
			pcd.colors_ = mesh.vertex_colors_;
			pcd = *std::get<0>(pcd.RemoveRadiusOutliers(20, 0.05));
			open3d::io::WritePointCloud(pcd_name, pcd, options);

		/*	Refiner refiner;
			logger->log("Generating refined scene mesh.");
			open3d::pipelines::registration::PoseGraph refined_pose_graph =
				refiner.run(cfg, fragments_pose_graph);

			mesh = integration::relative_integrate_rgb_frames_into_ScalableTSDFVolume(
				fragments,
				fragments_pose_graph,
				refined_pose_graph,
				intrinsic, cfg);
			pcd.points_ = mesh.vertices_;
			pcd.colors_ = mesh.vertex_colors_;
			pcd = *std::get<0>(pcd.RemoveRadiusOutliers(20, 0.05));
			open3d::io::WritePointCloud(pcd_name_refined, pcd, options);*/
		}

		void Reconstructor::set_running_indicator(bool* is_running)
		{
			main_is_running = is_running;
		}
	} // namespace reconstrucion
} // namespace logic