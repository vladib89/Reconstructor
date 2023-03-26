#include "OpencvPoseEstimator.hpp"

namespace logic
{
	namespace reconstruction
	{
		std::pair<bool, Eigen::Matrix4d> OpencvPoseEstimator::pose_estimation(open3d::geometry::RGBDImage& source_rgbd_image,
			open3d::geometry::RGBDImage& target_rgbd_image, open3d::camera::PinholeCameraIntrinsic& intrinsic, bool debug)
		{
			bool success = false;
			std::pair<bool, Eigen::Matrix4d> res;
			Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

			int source_color_width = source_rgbd_image.color_.width_;
			int source_color_height = source_rgbd_image.color_.height_;
			cv::Mat color_cv_s(cv::Size(source_color_width, source_color_height), CV_32FC1,
				(void*)source_rgbd_image.color_.data_.data(), cv::Mat::AUTO_STEP);
			
			int target_color_width = target_rgbd_image.color_.width_;
			int target_color_height = target_rgbd_image.color_.height_;
			cv::Mat color_cv_t(cv::Size(target_color_width, target_color_height), CV_32FC1,
				(void*)target_rgbd_image.color_.data_.data(), cv::Mat::AUTO_STEP);

			// transform double array to uint8 array
			color_cv_s *= 255.0;
			color_cv_s.convertTo(color_cv_s, CV_8UC1);
			color_cv_t *= 255.0;
			color_cv_t.convertTo(color_cv_t, CV_8UC1);
		
			auto orb = cv::ORB::create(100, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
			std::vector<cv::KeyPoint> kp_s;
			std::vector<cv::KeyPoint> kp_t;
			cv::Mat des_s;
			cv::Mat des_t;
			orb->detectAndCompute(color_cv_s, cv::Mat(), kp_s, des_s);
			orb->detectAndCompute(color_cv_t, cv::Mat(), kp_t, des_t);

			if (kp_s.size() == 0 || kp_t.size() == 0)
			{
				res.first = success;
				res.second = trans.cast<double>();
			}
			else
			{
				std::vector<cv::DMatch> matches;
				auto bf = cv::BFMatcher(cv::NORM_HAMMING, true);
				bf.match(des_s, des_t, matches);
				std::vector<cv::Point2f> pts_s;
				std::vector<cv::Point2f> pts_t;
				cv::Mat out_img;
				for (int i = 0; i < matches.size(); i++)
				{
					pts_t.push_back(kp_t[matches[i].trainIdx].pt);
					pts_s.push_back(kp_s[matches[i].queryIdx].pt);
				}

				double focal_input = (intrinsic.intrinsic_matrix_(0, 0) + intrinsic.intrinsic_matrix_(1, 1)) / 2.0f;
				double pp_x = intrinsic.intrinsic_matrix_(0, 2);
				double pp_y = intrinsic.intrinsic_matrix_(1, 2);
				cv::Point2d pp(pp_x, pp_y);

				std::vector<cv::Point2i> pts_s_int;
				std::vector<cv::Point2i> pts_t_int;

				for (int i = 0; i < pts_s.size(); i++)
				{
					cv::Point2i s(pts_s[i].x + 0.5, pts_s[i].y + 0.5);
					pts_s_int.push_back(s);
					cv::Point2i t(pts_t[i].x + 0.5, pts_t[i].y + 0.5);
					pts_t_int.push_back(t);
				}

				cv::Mat mask;
				auto E = cv::findEssentialMat(pts_s_int, pts_t_int, focal_input, pp, 8, 0.999, 1, mask);
				
				if (debug)
				{
					opencv::opencv_utils::draw_correspondences(color_cv_s, color_cv_t,
						kp_s, kp_t, matches, mask, "correspondences");
				}

				if (mask.empty())
				{
					res.first = success;
					res.second = trans.cast<double>();
				}
				else
				{
					// make 3D correspondences
					int source_depth_width = source_rgbd_image.depth_.width_;
					int source_depth_height = source_rgbd_image.depth_.height_;
					cv::Mat depth_s(cv::Size(source_depth_width, source_depth_height), CV_32FC1, (void*)source_rgbd_image.depth_.data_.data(), cv::Mat::AUTO_STEP);
																														 
					int target_depth_width = target_rgbd_image.depth_.width_;											 
					int target_depth_height = target_rgbd_image.depth_.height_;											 
					cv::Mat depth_t(cv::Size(target_depth_width, target_depth_height), CV_32FC1, (void*)target_rgbd_image.depth_.data_.data(), cv::Mat::AUTO_STEP);
				
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pts_xyz_s(3, pts_s.size());
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> pts_xyz_t(3, pts_s.size());

					int cnt = 0;

					for (int i = 0; i < pts_s.size(); i++)
					{
						if (mask.data[i])
						{
							auto xyz_s = get_xyz_from_pts(pts_s[i], depth_s, pp_x, pp_y, focal_input);
							pts_xyz_s(0, cnt) = xyz_s.x;
							pts_xyz_s(1, cnt) = xyz_s.y;
							pts_xyz_s(2, cnt) = xyz_s.z;

							auto xyz_t = get_xyz_from_pts(pts_t[i], depth_t, pp_x, pp_y, focal_input);
							pts_xyz_t(0, cnt) = xyz_t.x;
							pts_xyz_t(1, cnt) = xyz_t.y;
							pts_xyz_t(2, cnt) = xyz_t.z;
							cnt++;
						}
					}

					std::vector<int> inlier_vec_good;
					Eigen::Matrix4f transform_good = Eigen::Matrix4f::Identity();

					auto ind = Eigen::seq(0, cnt - 1);
					pts_xyz_s = pts_xyz_s(Eigen::all, ind);
					pts_xyz_t = pts_xyz_t(Eigen::all, ind);
					success = estimate_3D_transform_RANSAC(pts_xyz_s, pts_xyz_t, pts_xyz_s.cols(), inlier_vec_good, transform_good);
					res.first = success;
					res.second = transform_good.cast<double>();
				}
			}

			return res;
		}

		bool OpencvPoseEstimator::estimate_3D_transform_RANSAC(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& pts_xyz_s,
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& pts_xyz_t, int n_points, std::vector<int>& inlier_vec_good, Eigen::Matrix4f& transform_good)
		{
			int max_iter = 1000;
			float max_distance = 0.5f;
			int n_sample = 5;
			int max_inlier = n_sample;
			bool success = false;

			if (n_points < n_sample)
			{
				return false;
			}

			for (int i = 0; i < max_iter; i++)
			{
				std::unordered_set<int> rand_idx;
				BobFloydAlgo(n_sample, n_points, rand_idx);
				std::vector<Eigen::Vector3f> sample_xyz_vec_s;
				std::vector<Eigen::Vector3f> sample_xyz_vec_t;

				for (const int& idx : rand_idx)
				{
					Eigen::Vector3f s(pts_xyz_s(0, idx), pts_xyz_s(1, idx), pts_xyz_s(2, idx));
					sample_xyz_vec_s.push_back(s);
					Eigen::Vector3f t(pts_xyz_t(0, idx), pts_xyz_t(1, idx), pts_xyz_t(2, idx));
					sample_xyz_vec_t.push_back(t);
				}

				Eigen::MatrixXf sample_xyz_s(3, n_sample);
				Eigen::MatrixXf sample_xyz_t(3, n_sample);

				for (int j = 0; j < sample_xyz_vec_s.size(); j++)
				{
					sample_xyz_s.col(j) = sample_xyz_vec_s[j];
					sample_xyz_t.col(j) = sample_xyz_vec_t[j];
				}

				Eigen::MatrixXf R_approx;
				Eigen::Vector3f t_approx;
				estimate_3D_transform(sample_xyz_s, sample_xyz_t, R_approx, t_approx);
				// evalutation
				Eigen::MatrixXf diff_mat = pts_xyz_t - ((R_approx * pts_xyz_s).colwise() + t_approx);
				Eigen::MatrixXf diff(1, n_points);
				int n_inlier = 0;

				for (int i = 0; i < n_points; i++)
				{
					diff(0, i) = diff_mat.col(i).norm();

					if (diff(0, i) < max_distance)
						n_inlier++;
				}

				// note: diag(R_approx) > 0 prevents ankward transformation between
				// RGBD pair of relatively small amount of baseline.
				if (n_inlier > max_inlier && R_approx.determinant() != 0.0f &&
					R_approx(0, 0) > 0.0f && R_approx(1, 1) > 0.0f && R_approx(2, 2) > 0.0f)
				{
					float fourth_column[3] = { t_approx.x(), t_approx.y(), t_approx.z() };

					for (int i = 0; i < 3; i++)
					{
						for (int j = 0; j < 3; j++)
							transform_good(i, j) = R_approx(i, j);

						transform_good(i, 3) = fourth_column[i];
					}

					max_inlier = n_inlier;

					for (int i = 0; i < n_sample; i++)
					{
						if (diff(0, i) < max_distance)
							inlier_vec_good.push_back(i);
					}

					success = true;
				}
			}

			return success;
		}

		cv::Point3f OpencvPoseEstimator::get_xyz_from_pts(cv::Point2f& pt, cv::Mat& depth, double px, double py, double focal_point)
		{
			float u = pt.x;
			float v = pt.y;
			int u0 = (int)u;
			int v0 = (int)v;
			int height = depth.size().height;
			int width = depth.size().width;

			// bilinear depth interpolation

			if (u0 > 0 && u0 < width - 1 && v0 > 0 && v0 < height - 1)
			{
				float up = pt.x - u0;
				float vp = pt.y - v0;
				float d0 = depth.at<float>(v0, u0);
				float d1 = depth.at<float>(v0, u0 + 1);
				float d2 = depth.at<float>(v0 + 1, u0);
				float d3 = depth.at<float>(v0 + 1, u0 + 1);
				float d = (1 - vp) * (d1 * up + d0 * (1 - up)) + vp * (d3 * up + d2 * (1 - up));
				return get_xyz_from_uv(u, v, d, px, py, focal_point);
			}
			else
			{
				return cv::Point3f(0, 0, 0);
			}
		}

		cv::Point3f OpencvPoseEstimator::get_xyz_from_uv(float u, float v, float d, double px, double py, double focal)
		{
			double x = 0, y = 0;

			if (focal != 0)
			{
				x = (u - px) / focal * d;
				y = (v - py) / focal * d;
			}

			return cv::Point3f(x, y, d);
		}

		std::unordered_set<int> OpencvPoseEstimator::BobFloydAlgo(int sampleSize, int rangeUpperBound, std::unordered_set<int>& sample)
		{
			while (sample.size() < sampleSize)
			{
				int t = std::rand() % rangeUpperBound;

				if (sample.find(t) == sample.end())
				{
					sample.insert(t);
				}
			}

			return sample;
		}

		void OpencvPoseEstimator::estimate_3D_transform(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& input_xyz_s,
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& input_xyz_t, Eigen::MatrixXf& R_approx, Eigen::Vector3f& t_approx)
		{
			Eigen::MatrixXf xyz_s = input_xyz_s;
			Eigen::MatrixXf xyz_t = input_xyz_t;
			int n_points = xyz_s.cols();
			Eigen::Vector3f mean_s = xyz_s.colwise().mean();
			Eigen::Vector3f mean_t = xyz_t.colwise().mean();
			Eigen::MatrixXf xyz_diff_s = xyz_s.colwise() - mean_s;
			Eigen::MatrixXf xyz_diff_t = xyz_t.colwise() - mean_t;
			Eigen::MatrixXf H = xyz_diff_s * xyz_diff_t.transpose();
			Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);

			R_approx = svd.matrixV().transpose() * svd.matrixU().transpose();
			if (R_approx.determinant() < 0.0f)
			{
				float det = (svd.matrixU() * svd.matrixV()).determinant();
				Eigen::Matrix3f D = Eigen::Matrix3f::Identity();
				D(2, 2) = det;
				R_approx = svd.matrixU() * (D * svd.matrixV());
			}

			t_approx = mean_t - (R_approx * mean_s);
		}
	} // namespace reconstruction
} // namespace logic