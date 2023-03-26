#include "opencv_utils.hpp"

namespace logic
{
	namespace reconstruction
	{
		namespace opencv
		{
			namespace opencv_utils
			{
				cv::Mat out_img;
				std::mutex mtx;
				ushort search_depth_prior(const cv::Mat& depth, int i, int j);
				ushort search_depth_subsequent(const cv::Mat& depth, int i, int j);
				ushort interpolate_depth_pixel_prior(const cv::Mat& depth, int u, int v);
				/*cv::Vec3b*/ void interpolate_rgb_pixel(const cv::Mat& src, cv::Mat& dst, int i, int j, double u, double v);
				ushort interpolate_depth_pixel_subsequent(const cv::Mat& depth, int u, int v);
				float get_random();

				void draw_correspondences(cv::Mat& img_s, cv::Mat& img_t,
					std::vector<cv::KeyPoint>& kp_s,
					std::vector<cv::KeyPoint>& kp_t,
					std::vector<cv::DMatch>& matches,
					cv::Mat& mask,
					const std::string& title)
				{
					mtx.lock();
					cv::drawMatches(img_s, kp_s, img_t, kp_t, matches,
						out_img, cv::Scalar::all(-1), cv::Scalar::all(-1), mask);
					cv::namedWindow(title, cv::WindowFlags::WINDOW_NORMAL);
					cv::resizeWindow(title, cv::Size(1280, 720));
					cv::imshow(title, out_img);
					cv::waitKey(0);
					cv::destroyAllWindows();
					mtx.unlock();
				}

				void resize_and_copy_image_pair(
					void* src1, void* src2,
					int width, int height,
					int type1, int type2,
					int new_width, int new_height,
					void* dst1, void* dst2)
				{
					cv::Mat src1_resized =
						cv::Mat(cv::Size(width, height), type1, src1, cv::Mat::AUTO_STEP);
					cv::Mat src2_resized =
						cv::Mat(cv::Size(width, height), type2, src2, cv::Mat::AUTO_STEP);
					cv::resize(src1_resized, src1_resized, cv::Size(new_width, new_height));
					cv::resize(src2_resized, src2_resized, cv::Size(new_width, new_height));
					memcpy(dst1, (void*)src1_resized.data, src1_resized.total() * src1_resized.elemSize());
					memcpy(dst2, (void*)src2_resized.data, src2_resized.total() * src2_resized.elemSize());
				}

				bool render_depth_and_color(const cv::Mat& color, const cv::Mat& depth,
					unsigned short clipping_distance, std::string title)
				{
					bool stop = false;
					cv::Mat new_depth = depth.clone();
					cv::Mat color_cp = color.clone();;
					unsigned short pixel;

					for (int i = 0; i < new_depth.rows; i++)
					{
						for (int j = 0; j < new_depth.cols; j++)
						{
							pixel = new_depth.at<unsigned short>(i, j);

							if (pixel > clipping_distance ||
								pixel <= 0)
							{
								cv::Vec3b grey(153, 153, 153);
								color_cp.at<cv::Vec3b>(i, j) = grey;
							}
						}
					}

					cv::convertScaleAbs(new_depth, new_depth, 0.09);
					cv::Mat depth_colormap;
					cv::applyColorMap(new_depth, depth_colormap, cv::COLORMAP_JET);
					cv::hconcat(color_cp, depth_colormap, out_img);

					cv::namedWindow(title, cv::WindowFlags::WINDOW_NORMAL);
					cv::resizeWindow(title, cv::Size(1280, 720));
					cv::imshow(title, out_img);

					if (cv::waitKey(1) == 27)
					{
						cv::destroyAllWindows();
						stop = true;
					}

					return stop;
				}

				bool render_image(
					const cv::Mat& src,
					std::string title)
				{
					bool stop = false;
					cv::Mat out;
					cv::cvtColor(src, out, cv::COLOR_RGB2BGR);
					cv::namedWindow(title, cv::WindowFlags::WINDOW_NORMAL);
					cv::resizeWindow(title, src.size());
					cv::imshow(title, out);

					if (cv::waitKey(1) == 27)
					{
						cv::destroyAllWindows();
						stop = true;
					}

					return stop;
				}


				bool render_depth_and_color_pairwise(const cv::Mat& color_s, const cv::Mat& depth_s,
					const cv::Mat& color_t, const cv::Mat& depth_t,
					unsigned short clipping_distance, std::string title)
				{
					bool stop = false;
					cv::Mat new_depth_s = depth_s.clone();
					cv::Mat color_cp_s = color_s.clone();;
					unsigned short pixel;

					for (int i = 0; i < new_depth_s.rows; i++)
					{
						for (int j = 0; j < new_depth_s.cols; j++)
						{
							pixel = new_depth_s.at<unsigned short>(i, j);

							if (pixel > clipping_distance ||
								pixel <= 0)
							{
								cv::Vec3b grey(153, 153, 153);
								color_cp_s.at<cv::Vec3b>(i, j) = grey;
							}
						}
					}


					cv::convertScaleAbs(new_depth_s, new_depth_s, 0.09);
					cv::Mat depth_colormap_s;
					cv::applyColorMap(new_depth_s, depth_colormap_s, cv::COLORMAP_JET);
					cv::hconcat(color_cp_s, depth_colormap_s, out_img);

					cv::namedWindow("frame_0", cv::WindowFlags::WINDOW_NORMAL);
					cv::resizeWindow("frame_0", cv::Size(640, 360));
					cv::imshow("frame_0", out_img);

					cv::namedWindow("frame_1", cv::WindowFlags::WINDOW_NORMAL);
					cv::resizeWindow("frame_1", cv::Size(320, 360));
					cv::imshow("frame_1", color_t);

					cv::waitKey(0);
					if (cv::waitKey(1) == 27)
					{
						cv::destroyAllWindows();
						stop = true;
					}

					return stop;
				}

				void filter2(const cv::Mat& image, cv::Mat& out)
				{
					out = image.clone();

					if (image.type() == CV_16UC1)
					{
						for (int i = 0; i < out.rows; i++) {
							for (int j = 0; j < out.cols; j += 5) {
								out.at<unsigned char>(i, j) = 0;
							}
						}
					}
				}

				std::string type2str(int type) {
					std::string r;

					uchar depth = type & CV_MAT_DEPTH_MASK;
					uchar chans = 1 + (type >> CV_CN_SHIFT);

					switch (depth) {
					case CV_8U:  r = "8U"; break;
					case CV_8S:  r = "8S"; break;
					case CV_16U: r = "16U"; break;
					case CV_16S: r = "16S"; break;
					case CV_32S: r = "32S"; break;
					case CV_32F: r = "32F"; break;
					case CV_64F: r = "64F"; break;
					default:     r = "User"; break;
					}

					r += "C";
					r += (chans + '0');

					return r;
				}

				void create_opaque_rgbd_fragment_frame(
					const cv::Mat& color,
					const cv::Mat& depth,
					cv::Mat& dst
				)
				{
					unsigned short pixel;
					unsigned char grey = 255 - 0.8f * (255 - 153);
					cv::Vec3b grey_rgb_pixel(grey, grey, grey);

					for (int i = 0; i < depth.rows; i++)
					{
						for (int j = 0; j < depth.cols; j++)
						{
							pixel = depth.at<unsigned short>(i, j);

							if (pixel > 0)
							{
								dst.at<cv::Vec3b>(i, j) = grey_rgb_pixel;
							}
						}
					}
				}

				void perspective_transform(
					cv::Mat& src, cv::Mat& dst,
					cv::Mat& T, float f)
				{
					float w = src.cols;
					float h = src.rows;

					// Projection 2D -> 3D matrix
					cv::Mat A1 = (cv::Mat_<float>(4, 3) <<
						1, 0, -w / 2,
						0, 1, -h / 2,
						0, 0, 0,
						0, 0, 1);

					// 3D -> 2D matrix
					cv::Mat A2 = (cv::Mat_<float>(3, 4) <<
						f, 0, w / 2, 0,
						0, f, h / 2, 0,
						0, 0, 1, 0);


					//cv::Mat R = RX * RY * RZ;

					cv::Mat trans = A2 * T * A1;
					std::cout << trans << std::endl;

					cv::warpPerspective(src, dst, trans, src.size(),
						cv::INTER_LINEAR);
				}

				void affine_transform(cv::Mat& src, cv::Mat& out, cv::Mat& rotation)
				{
					cv::warpAffine(src, out, rotation, src.size(), cv::INTER_LINEAR,
						cv::BORDER_CONSTANT, cv::Scalar());
				}

				void two_d_to_3d(
					const cv::Mat& src,
					std::vector<cv::Point3f>& dst,
					data_structures::intrinsic& intrinsic,
					float depth_scale, cv::Mat transform)
				{
					float fx = intrinsic.fx;
					float fy = intrinsic.fy;
					float cx = intrinsic.cx;
					float cy = intrinsic.cy;
					float z;
					unsigned short pixel;
					cv::Vec3f p;

					for (size_t i = 0; i < src.rows; i++)
					{
						for (size_t j = 0; j < src.cols; j++)
						{
							pixel = src.at<unsigned short>(i, j);

							if (pixel > 0)
							{
								z = pixel * depth_scale;
								p = transform * cv::Vec3f(
									z * (j - cx) / fx,
									z * (i - cy) / fy,
									z);
								dst.push_back(p);
							}
						}
					}
				}
				/*	unsigned char grey = 255 - 0.8f * (255 - 153);
					cv::Vec3b rgb_pixel(grey, grey, grey);*/
				void transform_2d_in_3d_space(
					const cv::Mat& src_color,
					const cv::Mat& src_depth,
					cv::Mat& dst,
					data_structures::intrinsic& intrinsic,
					float depth_scale, cv::Mat transform)
				{
					double fx = intrinsic.fx;
					double fy = intrinsic.fy;
					double cx = intrinsic.cx;
					double cy = intrinsic.cy;
					double x, y, z, r2, f, dx, dy;
					int width = src_color.cols;
					int height = src_color.rows;
					cv::Mat p = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
					dst = cv::Mat(src_color.size(), src_color.type(), cv::Scalar(0, 0, 0));

					for (double i = 0; i < src_depth.rows; i += 2)
					{
						for (double j = 0; j < src_depth.cols; j += 2)
						{
							z = src_depth.at<ushort>(i, j) * depth_scale;

							if (z)
							{
								p.at<double>(2, 0) = z;
								p.at<double>(0, 0) = z * (j - cx) / fx;
								p.at<double>(1, 0) = z * (i - cy) / fy;

								p = transform * p;
								x = p.at<double>(0, 0) / p.at<double>(2, 0);
								y = p.at<double>(1, 0) / p.at<double>(2, 0);
								x = x * fx + cx;
								y = y * fy + cy;

								if (x >= 0 && x < width && y >= 0 && y < height)
								{
									dst.at<cv::Vec3b>(y, x) = src_color.at<cv::Vec3b>(i, j);
								}
							}
						}
					}
				}


				void cvtDepth2Cloud(const cv::Mat& depth, cv::Mat& cloud, const cv::Mat& cameraMatrix)
				{
					const float inv_fx = 1.f / cameraMatrix.at<float>(0, 0);
					const float inv_fy = 1.f / cameraMatrix.at<float>(1, 1);
					const float ox = cameraMatrix.at<float>(0, 2);
					const float oy = cameraMatrix.at<float>(1, 2);
					cloud.create(depth.size(), CV_32FC3);
					for (int y = 0; y < cloud.rows; y++)
					{
						cv::Point3f* cloud_ptr = (cv::Point3f*)cloud.ptr(y);
						const float* depth_prt = (const float*)depth.ptr(y);
						for (int x = 0; x < cloud.cols; x++)
						{
							float z = depth_prt[x];
							cloud_ptr[x].x = (x - ox) * z * inv_fx;
							cloud_ptr[x].y = (y - oy) * z * inv_fy;
							cloud_ptr[x].z = z;
						}
					}
				}

				float get_random()
				{
					static std::default_random_engine e;
					static std::uniform_real_distribution<> dis(0, 1); // rage 0 - 1
					return dis(e);
				}

				void bilinear_depth_interpolation(const cv::Mat& src, cv::Mat& dst)
				{
					int height = src.rows;
					int width = src.cols;
					ushort depth;
					dst = cv::Mat(src.size(), src.type(), cv::Scalar(0));

					for (int i = 0; i < height; i++)
					{
						for (int j = 0; j < width; j++)
						{
							depth = src.at<ushort>(i, j);

							if (depth <= 0)
							{
								if (j > 0 && j < width - 1 && i > 0 && i < height - 1)
								{
									depth = interpolate_depth_pixel_subsequent(src, j, i);

									if (depth <= 0)
									{
										depth = search_depth_subsequent(src, i, j);

										if (depth <= 0)
										{
											depth = search_depth_prior(dst, i, j);
										}
									}
								}
								else
								{
									depth = search_depth_prior(dst, i, j);

									if (depth <= 0)
									{
										depth = search_depth_subsequent(src, i, j);
									}
								}
							}


							dst.at<ushort>(i, j) = depth;
						}
					}
				}

				inline ushort search_depth_prior(const cv::Mat& depth, int i, int j)
				{
					ushort d = 0;
					int width = depth.cols;
					int height = depth.rows;

					for (; i >= 0; i--)
					{
						for (; j >= 0; j--)
						{
							d = depth.at<ushort>(i, j);

							if (d > 0)
							{
								if (j > 0 && j < width - 1 && i > 0 && i < height - 1)
								{
									d = interpolate_depth_pixel_prior(depth, j, i);
								}

								break;
							}
						}
					}

					return d;
				}

				inline ushort search_depth_subsequent(const cv::Mat& depth, int i, int j)
				{
					ushort d = 0;
					int width = depth.cols;
					int height = depth.rows;

					for (; i < height; i++)
					{
						for (; j < width; j++)
						{
							d = depth.at<ushort>(i, j);

							if (d > 0)
							{
								if (j > 0 && j < width - 1 && i > 0 && i < height - 1)
								{
									d = interpolate_depth_pixel_subsequent(depth, j, i);
								}

								break;
							}
						}
					}

					return d;
				}

				inline ushort interpolate_depth_pixel_prior(const cv::Mat& depth, int u, int v)
				{
					int height = depth.rows;
					int width = depth.cols;

					ushort d0 = depth.at<ushort>(v, u);
					ushort d1 = depth.at<ushort>(v, u - 1);
					ushort d2 = depth.at<ushort>(v - 1, u);
					ushort d3 = depth.at<ushort>(v - 1, u - 1);
					short d = (1 - v) * (d1 * u + d0 * (1 - u)) + v * (d3 * u + d2 * (1 - u));

					return d;
				}

				inline void interpolate_rgb_pixel(const cv::Mat& src, cv::Mat& dst, int u0, int v0, double u, double v)
				{
					int un = floor(u);
					int vn = floor(v);
					double delta_c = u - un;
					double delta_r = v - vn;

					cv::Vec3b d0 = src.at<cv::Vec3b>(v0, u0);
					cv::Vec3b d1 = src.at<cv::Vec3b>(v0, u0 + 1);
					cv::Vec3b d2 = src.at<cv::Vec3b>(v0 + 1, u0);
					cv::Vec3b d3 = src.at<cv::Vec3b>(v0 + 1, u0 + 1);
					if (d0 == cv::Vec3b(0, 0, 0))
					{
						std::cout << "black" << std::endl;
					}
					cv::Vec3b color;

					ushort r = round(
						d0[0] * (1 - delta_r) * (1 - delta_c) +
						d1[0] * delta_r * (1 - delta_c) +
						d3[0] * (1 - delta_r) * delta_c +
						d2[0] * delta_r * delta_c
					);

					ushort g = round(
						d0[1] * (1 - delta_r) * (1 - delta_c) +
						d1[1] * delta_r * (1 - delta_c) +
						d3[1] * (1 - delta_r) * delta_c +
						d2[1] * delta_r * delta_c
					);

					ushort b = round(
						d0[2] * (1 - delta_r) * (1 - delta_c) +
						d1[2] * delta_r * (1 - delta_c) +
						d3[2] * (1 - delta_r) * delta_c +
						d2[2] * delta_r * delta_c
					);

					dst.at<cv::Vec3b>(vn, un) += cv::Vec3b(r, g, b);


					//r = round(
					//	d0[0] * (1 - delta_r) * delta_c
					//);

					//g = round(
					//	d0[1] * (1 - delta_r) * delta_c
					//);

					//b = round(
					//	d0[2] * (1 - delta_r) * delta_c
					//);

					//dst.at<cv::Vec3b>(vn, un + 1) += cv::Vec3b(r, g, b);

					//r = round(
					//	d0[0] * delta_r * delta_c
					//);

					//g = round(
					//	d0[1] * delta_r * delta_c
					//);

					//b = round(
					//	d0[2] * delta_r * delta_c
					//);
					//dst.at<cv::Vec3b>(vn + 1, un + 1) += cv::Vec3b(r, g, b);


					//r = round(
					//	d0[0] * (1 - delta_r) * delta_c
					//);

					//g = round(
					//	d0[1] * (1 - delta_r) * delta_c
					//);

					//b = round(
					//	d0[2] * (1 - delta_r) * delta_c
					//);

					//dst.at<cv::Vec3b>(vn + 1, un) += cv::Vec3b(r, g, b);
					//return  cv::Vec3b(r, g, b);
				}

				inline ushort interpolate_depth_pixel_subsequent(const cv::Mat& depth, int u, int v)
				{
					int height = depth.rows;
					int width = depth.cols;

					ushort d0 = depth.at<ushort>(v, u);
					ushort d1 = depth.at<ushort>(v, u + 1);
					ushort d2 = depth.at<ushort>(v + 1, u);
					ushort d3 = depth.at<ushort>(v + 1, u + 1);
					short d = (1 - v) * (d1 * u + d0 * (1 - u)) + v * (d3 * u + d2 * (1 - u));

					return d;
				}

				void deproject_depth_map(
					cv::Mat& src, cv::Mat& dst,
					data_structures::intrinsic& intrinsic,
					float alpha)
				{
					unsigned short pixel;
					unsigned char grey = 255 - alpha * (255 - 153);
					cv::Vec3b grey_rgb_pixel(grey, grey, grey);
					cv::Vec3b blank_rgb_pixel(0, 0, 0);

					for (size_t i = 0; i < src.rows; i++)
					{
						for (size_t j = 0; j < src.cols; j++)
						{
							pixel = src.at<unsigned short>(i, j);

							if (pixel > 0)
							{
								dst.at<cv::Vec3b>(i, j) = grey_rgb_pixel;
							}
							else
							{
								dst.at<cv::Vec3b>(i, j) = blank_rgb_pixel;
							}
						}
					}
				}

				void three_d_to_2d(
					cv::Mat& src_color,
					std::vector<cv::Point3f>& src_depth,
					cv::Mat& dst,
					data_structures::intrinsic& intrinsic)
				{
					cv::Mat camera_matrix = (cv::Mat_<float>(3, 3) <<
						intrinsic.fx, 0, intrinsic.cx,
						0, intrinsic.fy, intrinsic.cy,
						0, 0, 1);
					cv::Mat distortion_mat = cv::Mat::zeros(1, 5, cv::DataType<float>::type);
					cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
					cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);

					std::vector<cv::Point2f> result;
					cv::projectPoints(src_depth, rvec, tvec, camera_matrix, distortion_mat, result);

					cv::Point2f p;
					unsigned char grey = 255 - 0.8f * (255 - 153);
					cv::Vec3b rgb_pixel(grey, grey, grey);
					int width = src_color.cols;
					int height = src_color.rows;
					dst = src_color.clone();

					for (int i = 0; i < result.size(); i++)
					{
						p = result[i];

						if (p.y > 0 && p.y < height && p.x > 0 && p.x < width)
							dst.at<cv::Vec3b>(p.y, p.x) += 0.5 * rgb_pixel;
					}
				}

				cv::Vec3f operator*(cv::Mat M, const cv::Vec3f& p)
				{
					cv::Mat src = (cv::Mat_<float>(4, 1) << p[0], p[1], p[2], 1);
					cv::Mat_<float> dst = M * src;
					float scale = dst(0, 3);
					return cv::Vec3f(dst(0, 0) / scale, dst(1, 0) / scale, dst(2, 0) / scale);
				}

				void warp_and_blend_RGBD_frames(
					cv::Mat& img1, cv::Mat& img1_transform,
					cv::Mat& img2, cv::Mat& img2_transform,
					logic::data_structures::intrinsic& intrinsic,
					cv::Mat& dst)
				{
					//cv::Rect r(0, 0, 3, 2);
					cv::Rect r_t(3, 0, 1, 3);
					/*	cv::Mat img1_t_2d = cv::Mat::eye(3, 3, CV_32FC1);
						cv::Mat img2_t_2d = cv::Mat::eye(3, 3, CV_32FC1);
						img1_transform(cv::Range(0, 2), cv::Range(0, 3)).copyTo(img1_t_2d(r));
						img2_transform(cv::Range(0, 2), cv::Range(0, 3)).copyTo(img2_t_2d(r));*/
					cv::Mat relative_transform = transform_matrix_difference(img1_transform, img2_transform);
					cv::Mat rotation_matrix = relative_transform(cv::Range(0, 3), cv::Range(0, 3));
					cv::Mat euler_rot;
					cv::Rodrigues(rotation_matrix, euler_rot);
					float d = sqrtf(img1.cols * img1.cols + img1.rows * img1.rows);
					float rx = euler_rot.at<float>(0, 0) * deg_to_rad;
					float ry = euler_rot.at<float>(1, 0) * deg_to_rad;
					float rz = euler_rot.at<float>(2, 0) * deg_to_rad;
					float dx = d / (2 * sin(rx) == 0 ? 1 : sin(rx));
					float dy = d / (2 * sin(ry) == 0 ? 1 : sin(ry));
					float dz = d / (2 * sin(rz) == 0 ? 1 : sin(rz));
					float f = dx;// cv::sqrt(dx * dx + dy * dy + dz * dz);

					//cv::Mat translation = cv::Mat::eye(4, 4, CV_32FC1);
					//relative_transform(cv::Range(0, 3), cv::Range(3, 4)).copyTo(translation(r_t));
					relative_transform.at<float>(2, 3) = dx;
					//relative_transform.at<float>(1, 3) = dy;
					//relative_transform.at<float>(2, 3) = f;
					//cv::warpPerspective(img1, dst, rt, cv::Size(img1.cols + img2.cols, img1.rows));
					perspective_transform(img1, dst, relative_transform, f);
					/*cv::Mat half(dst, cv::Rect(0, 0, img2.cols, img2.rows));
					img2.copyTo(half)*/;

					cv::imshow("dagech", dst);
					cv::waitKey(0);
				}

				cv::Mat transform_matrix_difference(
					const cv::Mat& source,
					const cv::Mat& target)
				{
					cv::Mat res = source.inv() * target;
					return res;
				}
			} // namespace opencv_utils
		} // namespace opencv
	} // namespace reconstruction
} // namespace logic