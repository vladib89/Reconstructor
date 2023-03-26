#include "motion_frame.hpp"

namespace logic
{
	namespace data_structures
	{
		motion_frame::motion_frame(Vector& rotation) 
			: frame()
		{
			size = sizeof(float) * 3;
			float tmp[3];
			handle = new char[size];
			rotation.toArray(tmp);
			memcpy(handle, &tmp, size);
		}
	}
}