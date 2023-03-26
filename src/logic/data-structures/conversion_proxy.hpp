#ifndef CONVERSIONPROXY_H
#define CONVERSIONPROXY_H
#define DATA_LEN 8

#include <librealsense2/rs.hpp>
#include <iostream>

namespace logic
{
	namespace data_structures
	{
		struct conversion_proxy
		{
			float* data = nullptr;
			bool boolean;
			long time_stamp;
			int frame_number = 0;
			conversion_proxy() {}
			conversion_proxy(const rs2_pose& pose);
			conversion_proxy(float* in_data) : data(in_data) {}
			conversion_proxy(bool boolean) :boolean(boolean) {}
			conversion_proxy(float* in_data, long time_stamp) : data(in_data), time_stamp(time_stamp) {}
			conversion_proxy(float* in_data, bool boolean) : data(in_data), boolean(boolean) {}
			conversion_proxy(const rs2_pose& pose, bool boolean);

			operator bool();
			operator float* ();
			void free();

		private:
			void copy_pose_to_array(const rs2_pose& pose);
		};
	}
}

#endif // CONVERSIONPROXY_H
