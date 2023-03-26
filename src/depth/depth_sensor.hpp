#ifndef DEPTHSENSOR_HPP
#define DEPTHSENSOR_HPP

#include "librealsense2/rs.hpp"
#include "../common/Sensor.hpp"
#include "../common/Loggable.hpp"
#include "depth_stream_config.hpp"
#include "../logic/data-structures/frame.hpp"
#include "../logic/data-structures/intrinsic.hpp"
#include "../logic/data-structures/motion_frame.hpp"

#include <map>

namespace depth
{
	enum PRODUCT_LINE
	{
		REALSESNSE
	};

	static std::map<PRODUCT_LINE, std::map<std::string, std::string>> depth_sensors;

	class depth_sensor : public common::Sensor
	{
	public:
		virtual float get_depth_scale() = 0;
		virtual void start(common::CallBack& cb, depth_stream_config& cfg) = 0;
		virtual logic::data_structures::intrinsic get_intrinsic() = 0;
		virtual bool get_frames(
			logic::data_structures::frame& color,
			logic::data_structures::frame& depth,
			logic::data_structures::motion_frame& motion) = 0;
		static std::map<std::string, std::string> get_available_depth_sensors();
	};
} // namespace depth

#endif // !DEPTHSENSOR_HPP
