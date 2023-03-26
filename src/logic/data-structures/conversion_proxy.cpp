#include "conversion_proxy.hpp"

namespace logic
{
namespace data_structures
{
conversion_proxy::conversion_proxy(const rs2_pose &pose)
{
	copy_pose_to_array(pose);
}

conversion_proxy::conversion_proxy(const rs2_pose &pose, bool boolean): boolean(boolean)
{
	copy_pose_to_array(pose);
}

conversion_proxy::operator bool()
{
	return boolean;
}

conversion_proxy::operator float*()
{
	return data;
}

void conversion_proxy::free()
{
	if (data != nullptr)
	{
		delete[] data;
	}
}

//#if defined(RS)
void conversion_proxy::copy_pose_to_array(const rs2_pose &pose)
{
	data = new float[DATA_LEN];
	data[0] = pose.translation.x;
	data[1] = pose.translation.y;
	data[2] = pose.translation.z;
	data[3] = pose.rotation.x;
	data[4] = pose.rotation.y;
	data[5] = pose.rotation.z;
	data[6] = pose.rotation.w;
	data[7] = 0.0f;
}
//#endif // defined RS
}
}
