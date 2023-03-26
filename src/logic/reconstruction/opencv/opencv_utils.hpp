#ifndef OPENCV_UTILS_HPP
#define OPENCV_UTILS_HPP

#include "../../data-structures/intrinsic.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <iomanip>
#include <mutex>
#include <random>

namespace logic
{
	namespace reconstruction
	{
		namespace opencv
		{
			namespace opencv_utils
			{
				const float PI = 3.14159265358979323846;
				const float deg_to_rad = PI / 180;
				extern cv::Mat out_img;
				extern std::mutex mtx;
				void filter2(
					const cv::Mat& image, 
					cv::Mat& out);
				void resize_and_copy_image_pair(
					void* src1,
					void* src2,
					int width,
					int height,
					int type1,
					int type2,
					int new_width,
					int new_height,
					void* dst1,
					void* dst2);
				bool render_depth_and_color(
					const cv::Mat& color, 
					const cv::Mat& depth,
					unsigned short clipping_distance, 
					std::string title);
				bool render_depth_and_color_pairwise(
					const cv::Mat& color_s, 
					const cv::Mat& depth_s,
					const cv::Mat& color_t, 
					const cv::Mat& depth_t,
					unsigned short clipping_distance, 
					std::string title);
				std::string type2str(int type);
				void draw_correspondences(
					cv::Mat& img_s, 
					cv::Mat& img_t,
					std::vector<cv::KeyPoint>& kp_s,
					std::vector<cv::KeyPoint>& kp_t,
					std::vector<cv::DMatch>& matches,
					cv::Mat& mask,
					const std::string& title);
				void create_opaque_rgbd_fragment_frame(
					const cv::Mat& color,
					const cv::Mat& depth,
					cv::Mat& dst 
				);
				void perspective_transform(
					cv::Mat& src, cv::Mat& dst,
					cv::Mat& T, float f);
				void affine_transform(
					cv::Mat& src, 
					cv::Mat& out, 
					cv::Mat& rotation);
				void two_d_to_3d(
					const cv::Mat& src, 
					std::vector<cv::Point3f>& dst,
					data_structures::intrinsic& intrinsic,
					float depth_scale,
					cv::Mat transform = cv::Mat::eye(4, 4, CV_32F));
				void three_d_to_2d(
					cv::Mat& src_color,
					std::vector<cv::Point3f>& src_depth,
					cv::Mat& dst,
					data_structures::intrinsic& intrinsic);
				cv::Vec3f operator*(cv::Mat M, const cv::Vec3f& p);
				void deproject_depth_map(
					cv::Mat& src,
					cv::Mat& dst,
					data_structures::intrinsic& intrinsic,
					float alpha = 0.5f);
				void transform_2d_in_3d_space(
					const cv::Mat& src_color,
					const cv::Mat& src_depth,
					cv::Mat& dst,
					data_structures::intrinsic& intrinsic,
					float depth_scale, cv::Mat transform);
				void bilinear_depth_interpolation(
					const cv::Mat& src, 
					cv::Mat& dst);
				void cvtDepth2Cloud(
					const cv::Mat& depth,
					cv::Mat& cloud,
					const cv::Mat& cameraMatrix);
				bool render_image(
					const cv::Mat& src,
					std::string title);
				void warp_and_blend_RGBD_frames(
					cv::Mat& img1, 
					cv::Mat& img1_transform, 
					cv::Mat& img2, 
					cv::Mat& img2_transform,
					logic::data_structures::intrinsic& intrinsic,
					cv::Mat& dst);
				cv::Mat transform_matrix_difference(
					const cv::Mat& source,
					const cv::Mat& target);
				template<class ImageElemType>
				void warpImage(
					const cv::Mat& image, 
					const cv::Mat& depth,
					const cv::Mat& Rt,
					const cv::Mat& cameraMatrix, 
					const cv::Mat& distCoeff,
					cv::Mat& warpedImage)
				{
					const cv::Rect rect = cv::Rect(0, 0, image.cols, image.rows);

					std::vector<cv::Point2f> points2d;
					cv::Mat cloud, transformedCloud;

					cvtDepth2Cloud(depth, cloud, cameraMatrix);
					cv::perspectiveTransform(cloud, transformedCloud, Rt);
					cv::projectPoints(transformedCloud.reshape(3, 1), cv::Mat::eye(3, 3, CV_64FC1), 
						cv::Mat::zeros(3, 1, CV_64FC1), cameraMatrix, distCoeff, points2d);

					cv::Mat pointsPositions(points2d);
					pointsPositions = pointsPositions.reshape(2, image.rows);

					warpedImage.create(image.size(), image.type());
					warpedImage = cv::Scalar::all(0);

					cv::Mat zBuffer(image.size(), CV_32FC1, FLT_MAX);
					for (float y = 0; y < image.rows; y++)
					{
						for (float x = 0; x < image.cols; x++)
						{
							const cv::Point3f p3d = transformedCloud.at<cv::Point3f>(y, x);
							const cv::Point p2d = pointsPositions.at<cv::Point2f>(y, x);
							if (!cvIsNaN(cloud.at<cv::Point3f>(y, x).z) && cloud.at<cv::Point3f>(y, x).z > 0 &&
								rect.contains(p2d) && zBuffer.at<float>(p2d) > p3d.z)
							{
								warpedImage.at<ImageElemType>(p2d) = image.at<ImageElemType>(y, x);
								zBuffer.at<float>(p2d) = p3d.z;
							}
						}
					}
				}
			} // namespace opencv_utils
		} // namespace opencv
	} // neamsepace reconstruction
} // namespace logic

#endif // !OPENCV_UTILS_HPP
