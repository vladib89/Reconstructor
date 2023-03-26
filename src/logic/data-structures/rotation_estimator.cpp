#include "rotation_estimator.hpp"

namespace logic
{
	namespace data_structures
	{
		void rotation_estimator::process_gyro(Vector gyro_data, double ts)
		{
			if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
			{
				last_ts_gyro = ts;
				return;
			}
			// Holds the change in angle, as calculated from gyro
			Vector gyro_angle;

			// Initialize gyro_angle with data from gyro
			gyro_angle.x = gyro_data.x; // Pitch
			gyro_angle.y = gyro_data.y; // Yaw
			gyro_angle.z = gyro_data.z; // Roll

			// Compute the difference between arrival times of previous and current gyro frames
			double dt_gyro = (ts - last_ts_gyro) / 1000.0;
			last_ts_gyro = ts;

			// Change in angle equals gyro measures * time passed since last measurement
			gyro_angle = gyro_angle * dt_gyro;

			// Apply the calculated change of angle to the current angle (theta)
			std::lock_guard<std::mutex> lock(theta_mtx);
			theta = theta + Vector(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
		}

		void rotation_estimator::process_accel(Vector accel_data)
		{
			// Holds the angle as calculated from accelerometer data
			Vector accel_angle;

			// Calculate rotation angle from accelerometer data
			accel_angle.z = atan2(accel_data.y, accel_data.z);
			accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

			// If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
			std::lock_guard<std::mutex> lock(theta_mtx);
			if (first)
			{
				first = false;
				theta = accel_angle;
				// Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
				theta.y = PI;
			}
			else
			{
				/*
				Apply Complementary Filter:
					- high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
					  that are steady over time, is used to cancel out drift.
					- low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
				*/
				theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
				theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
			}
		}

		Vector rotation_estimator::get_theta()
		{
			std::lock_guard<std::mutex> lock(theta_mtx);
			return theta;
		}
	}
}