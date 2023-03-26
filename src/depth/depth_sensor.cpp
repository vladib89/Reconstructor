#include "depth_sensor.hpp"

namespace depth
{
	std::map<std::string, std::string> depth_sensor::get_available_depth_sensors()
	{
        class static_logger : public common::Loggable
        {
        public:
            void log(std::string str)
            {
                logger->log(str);
            }
        }local_logger;

		rs2::context ctx;
		rs2::device_list devices_list = ctx.query_devices();
		std::vector<rs2::device> devices;
        std::map<std::string, std::string> device_serials_and_names;

		for (int i = 0; i < devices_list.size(); i++)
		{
            try
            {
                auto dev = devices_list[i];
                devices.emplace_back(dev);
            }
            catch (const std::exception& e)
            {
                local_logger.log("Could not create device - " + std::string(e.what()) + " . Check SDK logs for details");
            }
            catch (...)
            {
                local_logger.log("Failed to created device. Check SDK logs for details");
            }
		}

        for (int i = 0; i < devices.size(); i++)
        {
            auto dev = devices[i];
            std::string dev_name = 
                dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
            std::string dev_serial = 
                dev.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER);
            device_serials_and_names.insert({ dev_name, dev_serial });
        }
       
        depth_sensors.insert({ PRODUCT_LINE::REALSESNSE, device_serials_and_names });

        return device_serials_and_names;
	}
}