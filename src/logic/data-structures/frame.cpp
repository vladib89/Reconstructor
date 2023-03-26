#include "frame.hpp"

namespace logic
{
	namespace data_structures
	{
		frame::frame()
		{
			handle = nullptr;
			size = 0;
		}

		frame::frame(rs2::frame& f, int width, int height)
			: width(width), height(height)
		{
			size = f.get_data_size();
			handle = new char[size];
			memcpy(handle, f.get_data(), size);
		}

		void* frame::get_data()
		{
			return handle;
		}

		void frame::free()
		{
			if (handle != nullptr)
				delete[] handle;
			
			handle = nullptr;
		}
	}
}