#ifndef CONFIG_HPP
#define CONFIG_HPP

#include "stepper/stepper_modes.hpp"
#include "../data-structures/intrinsic.hpp"

#include <iostream>

namespace logic
{
	namespace reconstruction
	{
		struct config
		{
		public:
			enum mode
			{
				VISUAL_ODOMETRY,
				ENCODER_ROTATION_ONLY
			};

			mode operating_mode;
			std::string name;
			std::string path_dataset;
			std::string icp_method;
			std::string global_registration;
			std::string depth_map_type;
			std::string folder_fragment;
			std::string folder_scene;
			std::string template_fragment_posegraph;
			std::string template_fragment_pointcloud;
			std::string template_fragment_posegraph_optimized;
			std::string template_global_posegraph;
			std::string template_global_posegraph_optimized;
			std::string template_refined_posegraph;
			std::string template_refined_posegraph_optimized;
			std::string template_global_point_cloud;
			data_structures::intrinsic intrinsic;
			int height;
			int width;
			int yaw = 0;
			int pitch = 0;
			int max_range_yaw = 0;
			int max_range_pitch = 0;
			int stepper_iterations = 1;
			stepper::operating_mode stepper_operating_mode;
			float depth_scale;
			float max_depth;
			float min_depth;
			float voxel_size;
			float max_depth_diff;
			float preference_loop_closure_odometry;
			float preference_loop_closure_registration;
			float tsdf_cubic_size;
			float fragment_threshold_angle;
			float fragment_threshold_translation;
			float scene_threshold_angle;
			float scene_threshold_translation;
			unsigned short clipping_distance;
			int n_frames_per_fragment;
			int n_keyframes_per_n_frame;
			bool multi_threading;
			bool debug;
		};
	} // namespace reconstruction
} // namespace logic

#endif // !CONFIG_HPP
