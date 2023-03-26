#include "optimization.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace optimization
		{
			void run_pose_graph_optimization(open3d::pipelines::registration::PoseGraph* pose_graph,
				float max_correspondence_distance,
				float preference_loop_closure)
			{
				// to display messages from o3d.pipelines.registration.global_optimization
				open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
				open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt method =
					open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt();
				open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria =
					open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria();
				open3d::pipelines::registration::GlobalOptimizationOption option =
					open3d::pipelines::registration::GlobalOptimizationOption(max_correspondence_distance, 0.25,
						preference_loop_closure, 0);
				open3d::pipelines::registration::GlobalOptimization(*pose_graph, method, criteria, option);
				open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Error);
			}
		} // namespace optimization
	} // namespace reconstruction
} // namespace logic