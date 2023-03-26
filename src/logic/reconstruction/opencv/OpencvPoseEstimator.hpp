#ifndef OPENCVPOSEESTIMATOR_HPP
#define  OPENCVPOSEESTIMATOR_HPP

#include "../../data-structures/intrinsic.hpp"
#include "open3d/Open3D.h"
#include "opencv_utils.hpp"
#include "Eigen/Dense"
#include <memory>
#include <random>
#include <unordered_set>

namespace logic
{
	namespace reconstruction
	{
		class OpencvPoseEstimator
		{
		private:
			std::unordered_set<int> BobFloydAlgo(
				int sampleSize, int rangeUpperBound, 
				std::unordered_set<int>& sample);
			cv::Point3f get_xyz_from_pts(
				cv::Point2f& pts_row, cv::Mat& depth, 
				double px, double py, double focal_point);
			cv::Point3f get_xyz_from_uv(
				float u, float v, 
				float d, double px,
				double py, double focal);
			void estimate_3D_transform(
				Eigen::Matrix<float, Eigen::Dynamic, 
				Eigen::Dynamic>& input_xyz_s,
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& input_xyz_t, 
				Eigen::MatrixXf& R_approx, Eigen::Vector3f& t_approx);
			bool estimate_3D_transform_RANSAC(
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& pts_xyz_s,
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& pts_xyz_t, 
				int n_points, std::vector<int>& inlier_vec_good, 
				Eigen::Matrix4f& transform_good);
		public:
			OpencvPoseEstimator() {}
			std::pair<bool, Eigen::Matrix4d> pose_estimation(
				open3d::geometry::RGBDImage& source_rgbd_image,
				open3d::geometry::RGBDImage& target_rgbd_image, 
				open3d::camera::PinholeCameraIntrinsic& intrinsic, 
				bool debug = false);
		};
	} // namespace reconstruction
} // namespace logic

#endif // !OPENCVPOSEESTIMATOR_HPP
