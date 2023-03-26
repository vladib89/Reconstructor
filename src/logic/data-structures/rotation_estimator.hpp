#ifndef ROTATION_ESTIMATOR
#define ROTATION_ESTIMATOR

#include "Vector.hpp"
#include <mutex>

namespace logic
{
	namespace data_structures
	{
#ifndef PI
		const double PI = 3.14159265358979323846;
#endif
		class rotation_estimator
		{
			// theta is the angle of camera rotation in x, y and z components
			Vector theta;
			std::mutex theta_mtx;
			/* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
			values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
			float alpha = 0.98;
			bool first = true;
			// Keeps the arrival time of previous gyro frame
			double last_ts_gyro = 0;
		public:
			// Function to calculate the change in angle of motion based on data from gyro
			void process_gyro(Vector gyro_data, double ts);
			void process_accel(Vector accel_data);
			// Returns the current rotation angle
			Vector get_theta();
		};
	} // namespace logic
} // namespace data_structures

#endif // !ROTATION_ESTIMATOR
