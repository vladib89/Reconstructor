#include "intergration.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace integration
		{
			open3d::geometry::TriangleMesh integrate_rgb_frames_into_ScalableTSDFVolume(
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragment_images,
				open3d::pipelines::registration::PoseGraph& pose_graph,
				open3d::camera::PinholeCameraIntrinsic& intrinsic,
				config& cfg)
			{
				open3d::pipelines::integration::ScalableTSDFVolume volume =
					open3d::pipelines::integration::ScalableTSDFVolume(cfg.tsdf_cubic_size / 512.0, 0.04,
						open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
				std::stringstream ss;

				class lg : public common::Loggable
				{
				public:
					void log(std::string s) { logger->log(s); }
				}local_logger;

				for (int i = 0; i < pose_graph.nodes_.size(); i++)
				{
					ss.str(std::string());
					ss.clear();
					ss << "Integrate rgbd frame "
						<< i << " of " << pose_graph.nodes_.size() << ".";
					local_logger.log(ss.str());
					open3d::geometry::RGBDImage rgbd =
						*miscellaneous::create_rgbd_image(fragment_images[i].first,  fragment_images[i].second, false, cfg).get();
					Eigen::Matrix4d_u pose = pose_graph.nodes_[i].pose_;
					volume.Integrate(rgbd, intrinsic, pose.inverse());
				}

				// TODO: try to save time by trying to avoid triangulation
				open3d::geometry::TriangleMesh mesh = *volume.ExtractTriangleMesh().get();
				mesh.ComputeVertexNormals();

				return mesh;
			}

			open3d::geometry::TriangleMesh relative_integrate_rgb_frames_into_ScalableTSDFVolume(
				std::vector<std::pair<open3d::geometry::Image, open3d::geometry::Image>>& fragment_images,
				open3d::pipelines::registration::PoseGraph& scene_pose_graph,
				open3d::pipelines::registration::PoseGraph& refined_scene_pose_graph,
				open3d::camera::PinholeCameraIntrinsic& intrinsic,
				config& cfg)
			{
				open3d::pipelines::integration::ScalableTSDFVolume volume =
					open3d::pipelines::integration::ScalableTSDFVolume(cfg.tsdf_cubic_size / 512.0, 0.04,
						open3d::pipelines::integration::TSDFVolumeColorType::RGB8);
				std::stringstream ss;

				class lg : public common::Loggable
				{
				public:
					void log(std::string s) { logger->log(s); }
				}local_logger;

				for (int i = 0; i < scene_pose_graph.nodes_.size(); i++)
				{
					ss.str(std::string());
					ss.clear();
					ss << "Integrate rgbd frame "
						<< i << " of " << scene_pose_graph.nodes_.size() << ".";
					local_logger.log(ss.str());
					open3d::geometry::RGBDImage rgbd =
						*miscellaneous::create_rgbd_image(fragment_images[i].first, fragment_images[i].second, false, cfg).get();
					Eigen::Matrix4d_u pose = refined_scene_pose_graph.nodes_[i].pose_ * scene_pose_graph.nodes_[i].pose_;
					volume.Integrate(rgbd, intrinsic, pose.inverse());
				}

				// TODO: try to save time by trying to avoid triangulation
				open3d::geometry::TriangleMesh mesh = *volume.ExtractTriangleMesh().get();
				mesh.ComputeVertexNormals();

				return mesh;
			}
		} // namespace integration
	} // namespace reconstruction
} // namespace logic