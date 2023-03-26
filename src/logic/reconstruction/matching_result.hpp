#ifndef MATCHINGRESULT_HPP
#define MATCHINGRESULT_HPP

#include "Eigen/Dense"

namespace logic
{
	namespace reconstruction
	{
		struct matching_result
		{
			int s;
			int t;
			bool success;
			Eigen::Matrix4d transformation;
			Eigen::Matrix6d information;

			matching_result(int s, int t)
				: s(s), t(t), transformation(Eigen::Matrix4d::Identity()),
				information(Eigen::Matrix6d::Identity()), success(false) {}

			matching_result(int s, int t, Eigen::Matrix4d_u& transformation)
				: s(s), t(t), success(false), transformation(transformation),
				information(Eigen::Matrix6d::Identity()) {}
		};
	} // namespace reconstruction
} // namespace logic

#endif // !MATCHINGRESULT_HPP
