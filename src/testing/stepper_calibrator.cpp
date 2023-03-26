#include "stepper_calibrator.hpp"

namespace stepper_calibrator
{
	depth::realsense::RsDepthDevice* depth_device;

	void func1()
	{
		// depth device config
		depth::depth_stream_config depth_dev_config;
		depth_dev_config.color_format =
			logic::data_structures::stream_config::ColorFormat::RGB8;
		depth_dev_config.color_width = 960;
		depth_dev_config.color_height = 540;
		depth_dev_config.color_fps = 30;

		depth_dev_config.depth_format =
			depth::depth_stream_config::DepthFormat::Z16;
		depth_dev_config.depth_width = 640;
		depth_dev_config.depth_height = 480;
		depth_dev_config.depth_fps = 30;
		depth_dev_config.enable_accelerometer = true;
		depth_dev_config.enable_gyro = true;

		// init depth device
		depth_device =
			new depth::realsense::RsDepthDevice();

		class TestCallback : public common::CallBack, public common::Loggable
		{
		private:
			callback_function func;
		public:

			TestCallback(callback_function foo)
				: func(foo) {  }
			void on_success()
			{
				func();
			}
			void on_error(int result, std::string reason)
			{
				logger->log(reason);
			}
		}test_cb(func2);

		depth_device->start(test_cb);
	}

	void func2()
	{
		logic::data_structures::frame color;
		logic::data_structures::frame depth;
		logic::data_structures::motion_frame motion;
		open3d::geometry::Image color_source;
		color_source.Prepare(960, 540, 3, 1);
		open3d::geometry::Image depth_source;
		depth_source.Prepare(960, 540, 1, 2);
		open3d::geometry::Image color_target;
		color_target.Prepare(960, 540, 3, 1);
		open3d::geometry::Image depth_target;
		depth_target.Prepare(960, 540, 1, 2);
		int number_frames_to_dispose = 40;
		logic::reconstruction::config config;
		config.depth_scale = depth_device->get_depth_scale();
		config.operating_mode =
			logic::reconstruction::config::mode::ENCODER_ROTATION_ONLY;
		config.stepper_iterations = 1;
		config.stepper_operating_mode = logic::reconstruction::stepper::operating_mode::YAW_ONLY;
		config.max_range_yaw = 90;
		config.yaw = 15;
		int cport_n = logic::reconstruction::stepper::Stepper::find_device();
		logic::reconstruction::stepper::Stepper stepper(cport_n);
		auto success = stepper.start();
		logic::reconstruction::stepper::stepper_program program(stepper, config);
		Eigen::Matrix4d trans;
		Eigen::Matrix4d odometry_trans;
		Eigen::Matrix6d info;
		config.max_depth = 5.0f;
		int iterations = 3;
		auto option = open3d::pipelines::odometry::OdometryOption();
		option.max_depth_diff_ = 0.07;
		open3d::camera::PinholeCameraIntrinsic intrinsic;
		auto intr = depth_device->get_intrinsic();
		intrinsic.SetIntrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy);

		do
		{
			while (number_frames_to_dispose > 0)
			{
				color.free();
				depth.free();
				motion.free();

				if (depth_device->get_frames(color, depth, motion))
				{
					number_frames_to_dispose--;
				}
			}
			number_frames_to_dispose = 40;

			memcpy(color_source.data_.data(), color.get_data(), color_source.data_.size());
			memcpy(depth_source.data_.data(), depth.get_data(), depth_source.data_.size());

			open3d::geometry::RGBDImage source_rgbd_image = *logic::reconstruction::miscellaneous::create_rgbd_image(color_source,
				depth_source, true, config);

			std::tie(success, trans) =
				program.next_step();

			while (number_frames_to_dispose > 0)
			{
				color.free();
				depth.free();
				motion.free();

				if (depth_device->get_frames(color, depth, motion))
				{
					number_frames_to_dispose--;
				}
			}
			number_frames_to_dispose = 40;

			memcpy(color_target.data_.data(), color.get_data(), color_target.data_.size());
			memcpy(depth_target.data_.data(), depth.get_data(), depth_target.data_.size());
			open3d::geometry::RGBDImage target_rgbd_image = *logic::reconstruction::miscellaneous::create_rgbd_image(color_target,
				depth_target, true, config);

			std::tie(success, odometry_trans, info) =
				open3d::pipelines::odometry::ComputeRGBDOdometry(source_rgbd_image, target_rgbd_image,
					intrinsic, Eigen::Matrix4d::Identity(),
					open3d::pipelines::odometry::RGBDOdometryJacobianFromHybridTerm(), option);
			Eigen::Matrix3d wtf = odometry_trans.inverse().topLeftCorner(3, 3).cast<double>();
			Eigen::Quaterniond q(wtf);
			Eigen::Vector3d euler = logic::reconstruction::utils::camera_transforms::quaternion_2_euler(q);
			Eigen::Matrix4d tc = logic::reconstruction::utils::camera_transforms::transform_matrix_difference(trans, odometry_trans.inverse());
			std::cout << odometry_trans.inverse() << std::endl;
			std::cout << euler << std::endl;
			color.free();
			depth.free();
			motion.free();

			number_frames_to_dispose = 40;
		} while (--iterations > 0);
	}
}