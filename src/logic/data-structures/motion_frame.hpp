#ifndef MOTION_FRAME
#define MOTION_FRAME

#include "frame.hpp"
#include "Vector.hpp"

namespace logic
{
	namespace data_structures
	{
		class motion_frame : public frame
		{
		public:
			motion_frame() : frame() {}
			motion_frame(Vector& rotation);
		};
	} // namespace data_structures
} // namespace logic

#endif // !MOTION_FRAME
