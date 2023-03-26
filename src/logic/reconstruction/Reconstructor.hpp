#ifndef RECONSTRUCTOR_HPP
#define RECONSTRUCTOR_HPP

#include "config.hpp"
#include "../IDisplayable.hpp"
#include "SceneIntegrator.hpp"
#include "SceneFragmentizer.hpp"
#include "FragmentRegistrator.hpp"
#include "../../common/Loggable.hpp"
#include "stepper/stepper_program.hpp"
#include "../../depth/depth_sensor.hpp"
#include "reconstruction_utils/optimization.hpp"
#include "reconstruction_utils/intergration.hpp"
#include "reconstruction_utils/miscellaneous.hpp"

#include "open3d/Open3D.h"
#include "open3d/geometry/RGBDImage.h"

#include <iostream>

namespace logic
{
	namespace reconstruction
	{
		class Reconstructor : public common::Loggable, public IDisplayable
		{
		private:
			config cfg;
			cv::Mat out_img;
			void* out_img_external_ptr;
			std::thread worker;
			bool is_running;
			bool* main_is_running;
			open3d::camera::PinholeCameraIntrinsic intrinsic;
			SceneFragmentizer* scene_fragmentizer = nullptr;
			SceneIntegrator* scene_integrator;
			depth::depth_sensor* depth_sensor = nullptr;
			std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>
				convert_to_open3d_images(
					const std::vector<std::tuple<Eigen::Matrix4d, void*, void*>>& poses_and_fragment_frames);
			void execution();
			void init_config_and_folders();
			void generate_final_pcds(
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragments,
				open3d::pipelines::registration::PoseGraph& fragments_pose_graph
			);
			friend cv::Mat convert_eigen_matrix_to_cv(Eigen::MatrixXd& m, int type);
		public:
			Reconstructor(depth::depth_sensor* depth_sensor, config& cfg);
			~Reconstructor();
			bool start();
			bool set_image(void** img);
			void set_running_indicator(bool* is_running);
		};

		template <typename T>
		cv::Mat convert_eigen_matrix_to_cv(Eigen::MatrixXd& m, int type)
		{
			cv::Mat res = cv::Mat::eye(m.rows(), m.cols(), type);

			for (int i = 0; i < m.rows(); i++)
				for (int j = 0; j < m.cols(); j++)
					res.at<T>(i, j) = m(i, j);

			return res;
		}
	} // namespace logic
} // namespace reconstruction

#endif // !RECONSTRUCTOR_HPP
