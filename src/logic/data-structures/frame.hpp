#ifndef FRAME_HPP
#define FRAME_HPP

#include <memory>
#include <librealsense2/rs.hpp>

namespace logic
{
	namespace data_structures
	{
		class frame
		{
		protected:
			char* handle;
			int size;
		public:
			int width;
			int height;
			frame();
			frame(rs2::frame& f, int width, int height);
			void* get_data();
			void free();
		};
	}
}

#endif // !FRAME_HPP
