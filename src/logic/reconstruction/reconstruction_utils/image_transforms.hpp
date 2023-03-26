#ifndef IMAGE_TRANSFORMS_HPP
#define IMAGE_TRANSFORMS_HPP

#include "../../data-structures/intrinsic.hpp"
#include "Eigen/Dense"
#include <iostream>

namespace logic
{
	namespace reconstruction
	{
#ifndef PI
		const double PI = 3.14159265358979323846;
#endif
		const float rad_to_deg = 180 / PI;
		const float deg_to_rad = PI / 180;
		namespace utils
		{
			namespace camera_transforms
			{
				template <typename T>
				T clip(const T& n, const T& lower, const T& upper);
				std::pair<bool, double> distance_between_rot_mats(
					const Eigen::MatrixXd& rot_mat1, 
					const Eigen::MatrixXd& rot_mat2);
				Eigen::Quaterniond euler_2_quaternion(const double roll,
					const double pitch,
					const double yaw);
				Eigen::Vector3d quaternion_2_euler(Eigen::Quaterniond& q);
				Eigen::Quaterniond quaternion_difference(
					const Eigen::Quaterniond& source,
					const Eigen::Quaterniond& target);
				Eigen::Matrix4d transform_matrix_difference(
					const Eigen::Matrix4d& source,
					const Eigen::Matrix4d& target);
				inline Eigen::Quaterniond avg_quaternion_markley(Eigen::MatrixXd Q);
				inline Eigen::Quaterniond wavg_quaternion_markley(Eigen::MatrixXd Q, Eigen::VectorXd weights);
				inline Eigen::Vector3f deproject(float u, float v, float depth, 
					data_structures::intrinsic& intrinsic);
			} // naemspace camera_transforms
		} // namespace utils
	} // namespace reonstruction
} // namespace logic

#endif // !IMAGE_TRANSFORMS_HPP
