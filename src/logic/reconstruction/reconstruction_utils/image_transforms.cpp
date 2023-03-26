#include "image_transforms.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace utils
		{
			namespace camera_transforms
			{
				std::pair<bool, double> distance_between_rot_mats(
					const Eigen::MatrixXd& rot_mat1, const Eigen::MatrixXd& rot_mat2)
				{
					std::pair<bool, double> res;
					Eigen::MatrixXd diff_rot_mat = rot_mat1 * rot_mat2.transpose();
					double cos_angle = (diff_rot_mat.trace() - 1) / 2;

					if (!(cos_angle <= 1.001f && cos_angle >= -1.001f))
					{
						res = std::make_pair(false, 0.0f);
					}
					else
					{
						cos_angle = clip<double>(cos_angle, -1.0f, 1.0f);
						res = std::make_pair(true, acos(cos_angle));
					}

					return res;
				}

				template <typename T>
				T clip(const T& n, const T& lower, const T& upper) {
					return std::max(lower, std::min(n, upper));
				}

				Eigen::Quaterniond euler_2_quaternion(const double roll,
					const double pitch,
					const double yaw)
				{
					Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
					Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
					Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

					Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
 					return q;
				}

				Eigen::Vector3d quaternion_2_euler(Eigen::Quaterniond& q)
				{
					return q.toRotationMatrix().eulerAngles(0, 1, 2) * rad_to_deg;
				}

				Eigen::Quaterniond quaternion_difference(
					const Eigen::Quaterniond& source,
					const Eigen::Quaterniond& target)
				{
					Eigen::Quaterniond transition =
						target * source.inverse();

					return transition;
				}

				Eigen::Matrix4d transform_matrix_difference(
					const Eigen::Matrix4d& source,
					const Eigen::Matrix4d& target)
				{
					Eigen::Matrix4d transform = source.inverse() * target;
					return transform;
				}

				Eigen::Vector3f deproject(float u, float v, float depth,
					data_structures::intrinsic& intrinsic)
				{
					float x = depth * (u - intrinsic.cx) / intrinsic.fx;
					float y = depth * (v - intrinsic.cy) / intrinsic.fy;

					return Eigen::Vector3f(x, y, depth);
				}

				Eigen::Quaterniond avg_quaternion_markley(Eigen::MatrixXd Q)
				{
					Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
					int M = Q.rows();

					for (int i = 0; i < M; i++) {
						Eigen::Vector4d q = Q.row(i);
						if (q[0] < 0)
							q = -q;
						A = q * q.adjoint() + A;
					}

					A = (1.0 / M) * A;

					Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(A);
					Eigen::Vector4d qavg = eig.eigenvectors().col(3);

					return Eigen::Quaterniond(qavg);
				}

				Eigen::Quaterniond wavg_quaternion_markley(Eigen::MatrixXd Q, Eigen::VectorXd weights)
				{

				}
			} // naemspace camera_transforms
		} // namespace utils
	} // namespace reonstruction
} // namespace logic
