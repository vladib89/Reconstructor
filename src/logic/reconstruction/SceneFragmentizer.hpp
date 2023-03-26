#ifndef FRAGMENTIZER_HPP
#define FRAGMENTIZER_HPP

#include "config.hpp"
#include "../server.hpp"
#include "SceneIntegrator.hpp"
#include "../IDisplayable.hpp"
#include "../LogicCommons.hpp"
#include "../filters/kalman.hpp"
#include "../../common/Loggable.hpp"
#include "stepper/stepper_program.hpp"
#include "../data-structures/task.hpp"
#include "../../common/FixedQueue.hpp"
#include "../../depth/depth_sensor.hpp"
#include "opencv/OpencvPoseEstimator.hpp"
#include "reconstruction_utils/intergration.hpp"
#include "reconstruction_utils/optimization.hpp"
#include "reconstruction_utils/miscellaneous.hpp"

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cmath>
#include <iomanip>
#include <sstream>
#include <memory>
#include <mutex>

namespace logic
{
	namespace reconstruction
	{
		class SceneFragmentizer : public common::Loggable
		{
		public:
			~SceneFragmentizer();
			SceneFragmentizer(
				config& cfg,
				depth::depth_sensor* depth_sensor, 
				SceneIntegrator& scene_integrator);
			bool get_current_pose(
				Eigen::Matrix4d& current_pose);
		private:
			bool is_running;
			int fragment_id = 0;
			int s = 0, t = 0;
			SceneIntegrator& scene_integrator;
			depth::depth_sensor* depth_sensor;
			stepper::Stepper* stepper;
			std::mutex m;
			std::mutex m_rendering;
			std::thread worker_sampling;
			Eigen::Matrix4d last_pose = Eigen::Matrix4d::Identity();
			Eigen::Matrix4d odometry_transform = Eigen::Matrix4d::Identity();
			open3d::camera::PinholeCameraIntrinsic intrinsic;
			filters::transform::KalmanFilter t_kf;
			open3d::pipelines::registration::PoseGraph pose_graph;
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>> *fragment_images;
			bool make_posegraph_for_rgbd_fragment(
				open3d::geometry::Image& source_rgb,
				open3d::geometry::Image& target_rgb,
				open3d::geometry::Image& source_depth,
				open3d::geometry::Image& target_depth,
				Eigen::Vector3d& source_transform,
				Eigen::Vector3d& target_transform,
				int* s, int* t,
				config& cfg);
			void optimize_posegraph_for_fragment(
				open3d::pipelines::registration::PoseGraph* pose_graph,
				config& cfg);
			void rgbd_odometry_sampling_action(config& cfg);
			void encoder_odometry_sampling_action(config& cfg);
			void run_rgbd_odometry_based(
				std::tuple<const Eigen::Vector3d&, open3d::geometry::Image&,
				open3d::geometry::Image> source,
				std::tuple<const Eigen::Vector3d&, open3d::geometry::Image&,
				open3d::geometry::Image> target,
				config& cfg);
			void run_encoder_rotation_based(
				open3d::geometry::Image& color,
				open3d::geometry::Image& depth,
				Eigen::Matrix4d& transform,
				config& cfg);
			void make_posegraph_for_encoder_fragment(
				Eigen::Matrix4d& transform,
				int* s, int* t,
				config& cfg);
			void init(config& cfg);
		};
	} // namespace reconstruction
} // namespace logic

#endif // !FRAGMENTIZER_HPP