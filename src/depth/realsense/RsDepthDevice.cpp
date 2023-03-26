#include "RsDepthDevice.hpp"

namespace depth
{
	namespace realsense
	{
		RsDepthDevice::RsDepthDevice() : align(nullptr)
		{
		}

		void RsDepthDevice::start(common::CallBack& cb)
		{

		}

		void RsDepthDevice::start(common::CallBack& cb, depth_stream_config& cfg)
		{
			auto rs_config = convert_to_rs_config(cfg);

			try
			{
				callback = &cb;
				// Start pipeline with chosen configuration
				profile = pipe.start(rs_config);
				auto intr = profile.get_stream(rs2_stream::RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
				intrinsic.cx = intr.ppx;
				intrinsic.cy = intr.ppy;
				intrinsic.fx = intr.fx;
				intrinsic.fy = intr.fy;
				intrinsic.height = intr.height;
				intrinsic.width = intr.width;
				intrinsic.distortion = intr.coeffs;

				for (auto& sensor : profile.get_device().query_sensors())
				{
					if (sensor.is<rs2::depth_sensor>())
					{
						depth_scale = sensor.as<rs2::depth_sensor>().get_depth_scale();
						sensor.as<rs2::depth_sensor>().set_option(rs2_option::RS2_OPTION_VISUAL_PRESET,
							rs2_l500_visual_preset::RS2_L500_VISUAL_PRESET_SHORT_RANGE);
					}
					/*else if (sensor.is<rs2::color_sensor>())
					{
						sensor.set_option(rs2_option::RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
					}*/
				}

				is_running = true;
				align = new rs2::align(RS2_STREAM_COLOR);
				worker = std::thread(&RsDepthDevice::polling_action, this);

				callback->on_success();
			}
			catch (const rs2::error& e)
			{
				std::stringstream err;
				err << "RealSense error calling " << e.get_failed_function() <<
					"(" + e.get_failed_args() << "):\n    " << e.what() << std::endl;
				this->callback->on_error(0, err.str());
			}
		}

		void RsDepthDevice::start(common::CallBack& cb, bool relocalisation)
		{
			start(cb);
		}

		logic::data_structures::conversion_proxy RsDepthDevice::get_frame()
		{
			return nullptr;
		}

		logic::data_structures::conversion_proxy RsDepthDevice::get_frame(long time_stamp)
		{
			return nullptr;
		}

		bool RsDepthDevice::get_frames(
			logic::data_structures::frame& color,
			logic::data_structures::frame& depth,
			logic::data_structures::motion_frame& motion)
		{
			bool success = false;

			if (!f_queue.empty())
			{
				auto r = f_queue.dequeue();
				color = std::get<0>(r);
				depth = std::get<1>(r);
				motion = std::get<2>(r);
				success = true;
			}

			return success;
		}

		void RsDepthDevice::polling_action()
		{
			const std::string disparity_filter_name = "Disparity";
			rs2::temporal_filter temp_filter;
			temp_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.0f);
			temp_filter.set_option(rs2_option::RS2_OPTION_FILTER_SMOOTH_DELTA, 100.0f);
			rs2::disparity_transform depth_to_disparity(true);
			rs2::disparity_transform disparity_to_depth(false);
			filters.emplace_back(disparity_filter_name, depth_to_disparity);

			while (is_running)
			{
				rs2::frameset frames = pipe.wait_for_frames();
				auto processed = align->process(frames);
				rs2::video_frame vf = processed.first_or_default(RS2_STREAM_COLOR);
				rs2::motion_frame af = processed.first_or_default(RS2_STREAM_ACCEL);
				rs2::motion_frame gf = processed.first_or_default(RS2_STREAM_GYRO);
				rs2::depth_frame df = processed.get_depth_frame();

				if (!vf || !df || !af || !gf)
				{
					continue;
				}

				auto now = std::chrono::system_clock::now().time_since_epoch();
				double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
				double pose_time_ms1 = af.get_timestamp();
				double pose_time_ms2 = vf.get_timestamp();
				double pose_time_ms3 = gf.get_timestamp();
				double pose_time_ms4 = df.get_timestamp();
				float dt_s = static_cast<float>(std::max(0., (now_ms - pose_time_ms1) / 1000.));

				double ts = gf.get_timestamp();
				rs2_vector gyro_data = gf.get_motion_data();
				rot_estimator.process_gyro(logic::data_structures::Vector(gyro_data.x, gyro_data.y, gyro_data.z), ts);
				rs2_vector accel_data = af.get_motion_data();
				rot_estimator.process_accel(logic::data_structures::Vector(accel_data.x, accel_data.y, accel_data.z));

				rs2::depth_frame filtered = df;
				bool revert_disparity = false;
				for (auto&& filter : filters)
				{
					if (filter.is_enabled)
					{
						filtered = filter.filter.process(filtered);
						if (filter.filter_name == disparity_filter_name)
						{
							revert_disparity = true;
						}
					}
				}

				if (revert_disparity)
				{
					filtered = disparity_to_depth.process(filtered);
				}

				int width = vf.get_width();
				int height = vf.get_height();
				std::tuple<logic::data_structures::frame,
					logic::data_structures::frame, logic::data_structures::motion_frame>
					p({ vf, width, height }, { filtered, width, height }, rot_estimator.get_theta());
				f_queue.enqueue(p);
			}
		}

		rs2_stream RsDepthDevice::find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
		{
			rs2_stream align_to = RS2_STREAM_ANY;
			bool depth_stream_found = false;
			bool color_stream_found = false;

			for (rs2::stream_profile sp : streams)
			{
				rs2_stream profile_stream = sp.stream_type();

				if (profile_stream != RS2_STREAM_DEPTH)
				{
					if (!color_stream_found)         //Prefer color
						align_to = profile_stream;

					if (profile_stream == RS2_STREAM_COLOR)
					{
						color_stream_found = true;
					}
				}
				else
				{
					depth_stream_found = true;
				}
			}

			if (!depth_stream_found)
				throw std::runtime_error("No Depth stream available");

			if (align_to == RS2_STREAM_ANY)
				throw std::runtime_error("No stream found to align with Depth");

			return align_to;
		}

		logic::data_structures::intrinsic RsDepthDevice::get_intrinsic()
		{
			return intrinsic;
		}

		float RsDepthDevice::get_depth_scale()
		{
			return depth_scale;
		}

		rs2::config RsDepthDevice::convert_to_rs_config(depth_stream_config& cfg)
		{
			rs2::config rs_config;
			rs2_format depth_format;
			rs2_format color_format;

			switch (cfg.depth_format)
			{
			case depth_stream_config::DepthFormat::Z16:
				depth_format = rs2_format::RS2_FORMAT_Z16;
				break;
			}

			switch (cfg.color_format)
			{
			case logic::data_structures::stream_config::ColorFormat::RGB8:
				color_format = rs2_format::RS2_FORMAT_RGB8;
				break;
			case logic::data_structures::stream_config::ColorFormat::BGR8:
				color_format = rs2_format::RS2_FORMAT_BGR8;
				break;
			}

			rs_config.enable_stream(
				rs2_stream::RS2_STREAM_DEPTH,
				cfg.depth_width, cfg.depth_height,
				depth_format, cfg.depth_fps);
			rs_config.enable_stream(
				rs2_stream::RS2_STREAM_COLOR,
				cfg.color_width, cfg.color_height,
				color_format, cfg.color_fps);

			if (cfg.enable_gyro)
				rs_config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

			if (cfg.enable_accelerometer)
				rs_config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);

			return rs_config;
		}

		std::vector<RsDepthDevice> RsDepthDevice::get_available_devices()
		{
			rs2::context ctx;

			for (auto& x : ctx.query_devices())
			{
				//x.get_info();
				std::string arch = x.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
				std::cout << arch << std::endl;
			}

			return std::vector<RsDepthDevice>();
		}
	} // namespace realsense
} // namespace depth