#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/GlobalOptimizationMethod.h"
#include "open3d/pipelines/registration/GlobalOptimizationConvergenceCriteria.h"

namespace logic
{
	namespace reconstruction
	{
		namespace optimization
		{
			void run_pose_graph_optimization(
				open3d::pipelines::registration::PoseGraph* pose_graph,
				float max_correspondence_distance,
				float preference_loop_closure);
		} // namespace optimization
	} // namespace reonstruction
} // namespace logic

#endif // !OPTIMIZATION_HPP
